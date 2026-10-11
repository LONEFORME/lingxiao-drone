"""land() 的 land_timeout_gaveup 感知 + raw_imu 日志的单元测试。

从basic/basic_radar移植过来的安全修复(2026-07-17从basic同步)：固件纯超时兜底
(10秒)判定高度仍偏高时会放弃自动锁桨、永久等待人工介入，land()要能感知这个
状态并跳过自己的LAND_CONFIRM_TIMEOUT_S超时——否则Python侧会先关串口退出，
切断固件那边"等人工介入"期间悬停所需的T265速度参考，跟固件设计意图冲突。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/fire_patrol && python -m pytest test_land_timeout_gaveup.py -v
"""
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import mission


class FakeRealsense:
    """伪造T265接口，land()会在等待循环里自己重新采样，不依赖外部传入的旧值。"""
    def __init__(self, pos=(0.12, -0.05, 0.03), yaw=0.01, vel=(0.0, 0.0, 0.0),
                 raw_imu=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
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


def _make_mission_for_land(tmp_path):
    re_fc = [0] * 14
    se_fc = [0] * 11
    router = tmp_path / "router.txt"
    router.write_text("0.0,0.0,1.8\n")
    old_cwd = os.getcwd()
    os.chdir(tmp_path)
    try:
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
    finally:
        os.chdir(old_cwd)
    m.t265_ok = True
    m.realsense = FakeRealsense()
    m.state = "LAND"
    return m


class FakeSerialFcRef:
    """伪造串口对象，只提供 land() 需要读取的激光高度属性。"""
    def __init__(self, laser_height_m, motor_pwm_mask=None):
        self._last_laser_height_cm = laser_height_m
        self.debug_data = {"motor_pwm_mask": motor_pwm_mask} if motor_pwm_mask is not None else {}


class TransientUnlockList(list):
    """模拟re_fc[5](unlock_sta)按预设序列变化，序列用完后保持最后一个值不变。"""
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


class TestLandLogsRawImu:
    def test_land_logs_raw_imu(self, tmp_path):
        m = _make_mission_for_land(tmp_path)
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.realsense = FakeRealsense(raw_imu=(0.1, -0.2, 9.8, 0.01, -0.02, 0.03))

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["raw_imu"] == [0.1, -0.2, 9.8, 0.01, -0.02, 0.03]


class TestLandTimeoutGaveupHandling:
    """固件新增land_timeout_gaveup状态(纯超时兜底判定高度仍偏高、已放弃自动锁桨)，
    land()要能感知并联动调整自己的超时行为——不然Python侧25秒后自己先关串口退出，
    会切断固件那边"等人工介入"期间悬停所需的T265速度参考，跟固件的设计意图冲突。"""

    def test_logs_warning_once_when_gaveup_detected(self, tmp_path, monkeypatch):
        import Mission_GPT as mg
        m = _make_mission_for_land(tmp_path)
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)  # 一直不确认(始终为1)
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
            if call_count["n"] >= 20:
                raise _StopLoop()
        m.set_speed = _counting_set_speed

        try:
            m.land()
        except _StopLoop:
            pass

        gaveup_warnings = [msg for (_lvl, msg) in logged if "已放弃自动锁桨" in msg]
        assert len(gaveup_warnings) == 1  # 只打一次，不重复

    def test_skips_25s_timeout_when_gaveup_true(self, tmp_path, monkeypatch):
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.1)  # 很短的超时，方便测试
        m = _make_mission_for_land(tmp_path)
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

    def test_normal_timeout_still_works_when_gaveup_none(self, tmp_path, monkeypatch):
        """字段为None(老固件/未收到帧2)时，行为不变，仍按LAND_CONFIRM_TIMEOUT_S正常
        超时退出——回归守卫，确保这次改动不破坏旧行为。"""
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.1)
        m = _make_mission_for_land(tmp_path)
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)  # 始终不确认，逼近超时
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        # 不设置 land_timeout_gaveup 键，debug_data.get("land_timeout_gaveup") 返回 None

        logged = []
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        m.land()  # 应该正常在0.1秒超时后自己退出，不需要外部打断

        timeout_msgs = [msg for (_lvl, msg) in logged if "确认超时，强制退出" in msg]
        assert len(timeout_msgs) == 1
