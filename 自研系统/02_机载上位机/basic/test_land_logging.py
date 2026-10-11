"""land() 降落等待期间飞行日志记录的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic && python -m pytest test_land_logging.py -v
"""
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import mission


class FakeRealsense:
    """伪造T265接口，land()会在等待循环里自己重新采样，不依赖外部传入的旧值。"""
    def __init__(self, pos, yaw, vel=(0.0, 0.0, 0.0), raw_imu=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        self._pos = pos
        self._yaw = yaw
        self._vel = vel
        self._raw_imu = raw_imu

    def get_position(self):
        return list(self._pos)

    def get_orientation(self):
        return [0.0, 0.0, self._yaw]

    def get_velocity(self):
        return list(self._vel)

    def get_raw_imu(self):
        return list(self._raw_imu)


def _make_mission_for_land():
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
    m.t265_ok = True
    m.realsense = FakeRealsense(pos=(0.12, -0.05, 0.03), yaw=0.01)
    m.state = "LAND"
    return m


class FakeSerialFcRef:
    """伪造串口对象，只提供 land() 需要读取的激光高度属性。"""
    def __init__(self, laser_height_m, motor_pwm_mask=None):
        self._last_laser_height_cm = laser_height_m
        self.debug_data = {"motor_pwm_mask": motor_pwm_mask} if motor_pwm_mask is not None else {}


class TestLandLogging:
    def test_land_logs_position_when_immediately_unlocked(self):
        """land()原本从触发到确认/超时全程不写任何飞行日志，导致降落物理下降过程
        完全没有位置数据(2026-07-08真机测试发现)。这里验证哪怕最简单的"一进来就
        已经上锁"这种情况，也至少要记录一条日志。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 已经上锁，land()应该立刻确认退出

        m.land()

        m._log_file.seek(0)
        lines = [line for line in m._log_file.readlines() if line.strip()]
        assert len(lines) >= 1
        entry = json.loads(lines[-1])
        assert entry["state"] == "LAND"
        assert entry["pos"][0] == 0.12
        assert entry["pos"][1] == -0.05

    def test_land_logs_raw_imu(self):
        """2026-07-10问题7新增：给T265加了原始加速度计/陀螺仪接口(get_raw_imu())，
        用于独立验证电机是否真的停转(不依赖凌霄IMU自己上报的unlock_sta/
        motor_pwm_mask)。land()应该把这份数据也记进飞行日志，方便下次真机异常
        时事后核对振动特征。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.realsense = FakeRealsense(pos=(0.12, -0.05, 0.03), yaw=0.01,
                                     raw_imu=(0.1, -0.2, 9.8, 0.01, -0.02, 0.03))

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["raw_imu"] == [0.1, -0.2, 9.8, 0.01, -0.02, 0.03]

    def test_land_logs_laser_height_not_raw_t265_z(self):
        """land()原本自采样时直接用realsense.get_position()的原始Z轴——但loop()
        正常流程里Z轴其实是被激光高度覆盖过的(T265自身Z轴未标定，不是真实高度，
        见 takeoff() 同样直接读 _last_laser_height_cm 而不用T265 Z)。land()这里
        如果继续用原始T265 Z，降落阶段记录的"高度"数据是假的，没法用来验证物理
        降落过程，这正是加这份日志的初衷。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 立刻确认退出，只需要一条日志
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.07)  # 激光真实高度7cm
        # T265原始Z(FakeRealsense pos[2]=0.03)明显不同于激光高度，用来验证没有被误用

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["pos"][2] == 0.07

    def test_land_logs_unlock_sta(self):
        """降落确认依赖的unlock_sta字段本身原来没有写进日志，事后只能靠高度曲线
        间接推断"确认超时"到底是物理没执行还是确认通道滞后。直接记录这个字段能让
        排查更直接。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 已上锁

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["unlock_sta"] == 0

    def test_land_logs_motor_pwm_mask(self):
        """问题7 2026-07-08发现unlock_sta可能假阳性(读到已上锁但电机实际未停转)。
        固件新增了电机PWM非零位掩码字段(帧2)，land()应该把它也记进日志，方便
        下次复现时直接对比unlock_sta和这个掩码是否矛盾。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.07, motor_pwm_mask=0x0F)  # 4个电机都还在转

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["motor_pwm_mask"] == 0x0F

    def test_logs_motor_pwm_mask_arrival_timestamp(self):
        """2026-07-10严重案例(basic_radar)：unlock_sta+motor_pwm_mask双条件都归零、
        land()正常确认退出，但用户现场确认电机实际没有停转。怀疑motor_pwm_mask可能是
        帧2降频(~2.5秒一次)导致的滞后快照。Lprotocol.py已经给debug_data加了到达时间
        戳(motor_pwm_mask_t)，这里验证land()把它也写进飞行日志，方便下次核实"读到的
        0是不是新鲜数据"。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.06, motor_pwm_mask=0)
        m.serial_fc_ref.debug_data["motor_pwm_mask_t"] = 1234.5

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["motor_pwm_mask_t"] == 1234.5

    def test_land_logs_motor_pwm_mask_none_when_unavailable(self):
        """serial_fc_ref没有debug_data(比如刚连上还没收到过帧2)时，字段应该是
        None而不是抛异常。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.07)  # 没传motor_pwm_mask

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["motor_pwm_mask"] is None


class TransientUnlockList(list):
    """模拟re_fc[5](unlock_sta)按预设序列变化，序列用完后保持最后一个值不变。
    用于测试降落确认去抖逻辑，不依赖真实串口数据。"""
    def __init__(self, base, seq):
        super().__init__(base)
        self._seq = seq
        self._calls = 0

    def __getitem__(self, idx):
        if idx == 5:
            call = self._calls
            self._calls += 1
            if call < len(self._seq):
                return self._seq[call]
            return self._seq[-1]
        return super().__getitem__(idx)


class TestLandUnlockDebounce:
    """2026-07-09大范围20点网格测试真机观察到疑似假阳性：终端打印"降落确认：
    已上锁"并退出，但用户确认电机实际没有停转/没有真正降落；飞行日志显示确认
    发生前unlock_sta全程是1，从未记录到0。原逻辑单次读到0就立刻确认退出，容易
    被单帧通信噪声/校验巧合触发误判。这里改成要求连续LAND_UNLOCK_CONFIRM_COUNT
    次都读到0才真正确认。"""

    def test_single_glitch_does_not_confirm(self, monkeypatch):
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.3)  # 加速测试，不用等25秒真实超时
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        # 第3次读到0(瞬间glitch)，之后恢复为1一直到超时
        m.re_fc = TransientUnlockList([0] * 14, seq=[1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") not in logged
        # 2026-07-24：超时后不再打印"强制退出"，改为进入 HOVER_WAIT 等待人工介入
        assert ("warning", "降落确认超时，强制退出") not in logged
        assert m.state == "HOVER_WAIT", f"预期超时后进入 HOVER_WAIT，实际 state={m.state}"

    def test_confirms_after_consecutive_unlock_reads(self, monkeypatch):
        import Mission_GPT as mg
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        # 前2次还是1(未上锁)，之后持续读到0(真实持续上锁)
        m.re_fc = TransientUnlockList([0] * 14, seq=[1, 1, 0, 0, 0, 0, 0, 0])

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") in logged


class TestLandUnlockPwmConfirm:
    """2026-07-10矩形路径到达确认耗时基线测试(basic_radar)复现了问题7描述的假阳性：
    终端打印"降落确认：已上锁"、用户确认是人工接管完成的降落。核对飞行日志发现
    `unlock_sta`确实连续读到0满足了去抖要求，但同一时刻`motor_pwm_mask`全程稳定为15
    (四电机都还在出PWM)，两个字段直接矛盾——说明只看`unlock_sta`的去抖不够，需要
    同时要求`motor_pwm_mask==0`才能真正确认已上锁。`motor_pwm_mask`不可用(None，
    比如还没收到过帧2)时不应阻塞确认，退化成只看`unlock_sta`的旧行为。"""

    def test_unlock_confirmed_but_motor_pwm_nonzero_does_not_confirm(self, monkeypatch):
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.3)  # 加速测试，不用等25秒真实超时
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.06, motor_pwm_mask=0x0F)

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") not in logged
        # 2026-07-24：超时后不再打印"强制退出"，改为进入 HOVER_WAIT
        assert ("warning", "降落确认超时，强制退出") not in logged
        assert m.state == "HOVER_WAIT", f"预期超时后进入 HOVER_WAIT，实际 state={m.state}"

    def test_confirms_when_unlock_and_motor_pwm_both_zero(self, monkeypatch):
        import Mission_GPT as mg
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[0, 0, 0, 0, 0, 0, 0, 0])
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.06, motor_pwm_mask=0)

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") in logged

    def test_motor_pwm_mask_unavailable_falls_back_to_unlock_only(self, monkeypatch):
        import Mission_GPT as mg
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[0, 0, 0, 0, 0, 0, 0, 0])

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") in logged


class TestLandTimeoutGaveupHandling:
    """2026-07-12：固件新增land_timeout_gaveup状态(纯超时兜底判定高度仍偏高、
    已放弃自动锁桨)，land()要能感知并联动调整自己的超时行为——不然Python侧
    25秒后自己先关串口退出，会切断固件那边"等人工介入"期间悬停所需的T265
    速度参考，跟固件的设计意图冲突。"""

    def test_logs_warning_once_when_gaveup_detected(self, monkeypatch):
        """检测到land_timeout_gaveup=True时打印一次warning日志，不重复刷屏。"""
        import Mission_GPT as mg
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)  # 一直不确认(始终为1)
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        m.serial_fc_ref.debug_data["land_timeout_gaveup"] = True

        logged = []
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        # land()检测到gaveup后会跳过25秒超时、无限期循环——用一个计数器在set_speed
        # 里数到N次后抛出特定异常主动打断循环，验证循环确实没有自己退出。
        call_count = {"n": 0}

        class _StopLoop(Exception):
            pass

        def _counting_set_speed(*a, **k):
            call_count["n"] += 1
            if call_count["n"] >= 20:
                raise _StopLoop()
        m.set_speed = _counting_set_speed

        try:
            m.land()
        except _StopLoop:
            pass

        gaveup_warnings = [msg for (_lvl, msg) in logged if "已放弃自动锁桨" in msg]
        assert len(gaveup_warnings) == 1  # 只打一次，不重复

    def test_skips_25s_timeout_when_gaveup_true(self, monkeypatch):
        """检测到gaveup=True后，即使超过LAND_CONFIRM_TIMEOUT_S也不应该走
        "确认超时，强制退出"分支。"""
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.1)  # 很短的超时，方便测试
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        m.serial_fc_ref.debug_data["land_timeout_gaveup"] = True

        logged = []
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        call_count = {"n": 0}

        class _StopLoop(Exception):
            pass

        def _counting_set_speed(*a, **k):
            call_count["n"] += 1
            if call_count["n"] >= 20:  # 20轮*sleep(0.03) ≈ 0.6秒，远超0.1秒的超时阈值
                raise _StopLoop()
        m.set_speed = _counting_set_speed

        try:
            m.land()
        except _StopLoop:
            pass

        timeout_msgs = [msg for (_lvl, msg) in logged if "确认超时，强制退出" in msg]
        assert len(timeout_msgs) == 0  # 不应该触发旧的超时退出分支

    def test_normal_timeout_still_works_when_gaveup_none(self, monkeypatch):
        """字段为None(老固件/未收到帧2，或者serial_fc_ref本身没有这个debug_data键)时，
        行为不变，仍按LAND_CONFIRM_TIMEOUT_S超时进入HOVER_WAIT——回归守卫，确保这次改动
        不破坏旧行为。"""
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.1)
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)  # 始终不确认，逼近超时
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        # 不设置 land_timeout_gaveup 键，debug_data.get("land_timeout_gaveup") 返回 None

        m.land()  # 应该在超时后进入 HOVER_WAIT

        # 2026-07-24：超时后进入 HOVER_WAIT，不再是旧版"强制退出"
        assert m.state == "HOVER_WAIT", f"预期超时后进入 HOVER_WAIT，实际 state={m.state}"


class TestNewLandingFeatures:
    """2026-07-24 新增 DESCEND/HOVER_WAIT/is_near_ground 的单元测试。"""

    def test_is_near_ground_rejects_zero_or_negative(self):
        from Mission_GPT import is_near_ground, DESCEND_LOCK_HEIGHT_CM
        assert is_near_ground(0) is False    # 传感器失效
        assert is_near_ground(-1) is False   # 负数

    def test_is_near_ground_rejects_overflow_garbage(self):
        from Mission_GPT import is_near_ground
        assert is_near_ground(9999) is False   # 错误码
        assert is_near_ground(51) is False     # 超出 LASER_HEIGHT_NEAR_GROUND_MAX

    def test_is_near_ground_accepts_below_threshold(self):
        from Mission_GPT import is_near_ground, DESCEND_LOCK_HEIGHT_CM
        # 阈值 8cm，低于阈值应返回 True
        assert is_near_ground(DESCEND_LOCK_HEIGHT_CM - 1) is True
        assert is_near_ground(5) is True

    def test_is_near_ground_rejects_at_or_above_threshold(self):
        from Mission_GPT import is_near_ground, DESCEND_LOCK_HEIGHT_CM
        assert is_near_ground(DESCEND_LOCK_HEIGHT_CM) is False     # 等于阈值不算贴地
        assert is_near_ground(DESCEND_LOCK_HEIGHT_CM + 1) is False

    def test_navigate_all_waypoints_done_transitions_to_descend(self, monkeypatch):
        """全部航点完成后 navigate() 应切换到 DESCEND 状态（而非旧版 LAND）。"""
        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
        m.t265_ok = False

        # 设置一个已完成的航点列表，让 target_index 越过末尾
        m.targets = [[0, 0, 0.15]]  # 单个航点，高度 15cm（低于 20cm 安全分界线）
        m.target_index = 1  # 已超出范围
        m.state = "NAVIGATE"

        m.navigate(pos=[0.0, 0.0, 0.15], yaw=0.0)

        assert m.state == "DESCEND", f"预期 DESCEND，实际 {m.state}"

    def test_navigate_high_last_waypoint_stays_in_navigate(self):
        """最后一个航点高度高于安全分界线时，不应直接切 DESCEND，应继续 navigate 下降。"""
        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
        m.t265_ok = False

        m.targets = [[0, 0, 0.50]]  # 高度 50cm > 20cm 安全分界线
        m.target_index = 1
        m.state = "NAVIGATE"

        m.navigate(pos=[0.0, 0.0, 0.50], yaw=0.0)

        assert m.state == "NAVIGATE", f"预期仍为 NAVIGATE，实际 {m.state}"

    def test_descend_enters_land_when_unlock_already_zero(self):
        """DESCEND 首帧检测到 unlock_sta==0（固件已抢先锁桨），应直接转 LAND。"""
        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
        m.t265_ok = True
        m.realsense = FakeRealsense(pos=(0.0, 0.0, 0.08), yaw=0.0)
        m.state = "DESCEND"
        m.re_fc[5] = 0  # 固件已锁

        m.descend(pos=[0.0, 0.0, 0.08])

        assert m.state == "LAND", f"预期 LAND，实际 {m.state}"

    def test_hover_wait_sets_correct_state(self, monkeypatch):
        """hover_wait() 不改变 state，维持当前状态。"""
        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
        m.t265_ok = False
        m.state = "HOVER_WAIT"

        m.hover_wait()

        assert m.state == "HOVER_WAIT", "hover_wait 不应改变 state"

    def test_hover_wait_does_not_crash_without_t265(self):
        """hover_wait() 在 T265 不可用时不应抛异常。"""
        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
        m.t265_ok = False
        m.state = "HOVER_WAIT"

        # 不应抛出任何异常
        m.hover_wait()

