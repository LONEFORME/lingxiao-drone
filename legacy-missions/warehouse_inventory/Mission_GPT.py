"""
任务状态机 — 基本飞行 (无 K230 / 地面站 / 覆盖规划)

状态机:  IDLE → TAKEOFF → NAVIGATE → LAND → END
控制周期: 30ms
安全保护: FC 超时 2s / T265 丢失急停
"""
import threading
import time
import json
import math
import os
import sys
from collections import deque
from dataclasses import replace
from typing import List, Optional
from Lcode.heading_hold import HeadingHoldConfig, HeadingHoldController, wrap_degrees
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import (
    sp_side,
    lock,
    fc_frame_counter,
    fc_last_rx_monotonic,
    fc_last_rx_time,
)
from Lcode.navigation_profile import NavigationProfileConfig
from Lcode.resource_monitor import ResourceMonitor
from t265 import t265_class

# ---------- 常量 ----------
DRY_RUN = os.getenv("DRONE_DRY_RUN", "0") == "1"  # 桌面测试: 不解锁飞控，电机不会转
put_height = 100
VEL_SCALE = 0.7
posthreshold_xy = 0.15
posthreshold_z = 0.20
arrival_confirm_need = 15
arrival_hold_s = 1.5   # 到达判定满足后，在原地强制停留观察的时长（阶跃响应测试用）
arrival_timeout_max = 5.0 + arrival_hold_s
T265_CONFIDENCE_MIN = 2       # 定点所需最低追踪置信度 (0=失败,1=低,2=中,3=高)
T265_CONFIDENCE_WAIT_S = 8.0  # 等待置信度达标的超时时间
FLIGHT_LOG_INTERVAL = 0.05
RAMP_STEP = 1.5
TAKEOFF_CONFIRM_NEED = 10
TAKEOFF_TIMEOUT_S = 15.0
TAKEOFF_CONFIDENCE_ABORT_S = 0.3  # 起飞阶段T265持续丢失超过此时间就安全中止，不进入导航
TAKEOFF_LIFTOFF_CM = 35.0  # 一键起飞只负责盲飞离地这一小段，其余交给navigate()的x/y PID+高度ramp爬升到真正目标高度
                            # 不能设太低：2026-07-06实测15cm时T265/激光近地面定位质量下降，起飞confirm超时+机体水平旋转
LAND_CONFIRM_TIMEOUT_S = 25.0  # 降落触发后最多等待多久确认unlock_sta==0(已上锁)，超时也强制退出，避免卡死
                                # 2026-07-09从10.0改为25.0：1.0m高度门槛测试发现10秒内unlock_sta/motor_pwm_mask
                                # 全程未变但用户确认物理已自动降落成功，怀疑是超时定太短、真实上锁发生在
                                # 断开串口之后——调长验证这个假设，同时避免后续测试的"超时"结果继续有歧义
LAND_UNLOCK_CONFIRM_COUNT = 5  # 降落确认去抖：要连续读到N次unlock_sta==0才真正确认已上锁，不是单次就退出。
RETURN_CRUISE_ENABLED = True  # 返航中间通道点用巡航到达；末点仍保持精确到达。
                                # 2026-07-09真机观察到疑似假阳性——终端打印"已上锁"退出，但用户确认电机实际
                                # 未停转/没有真正降落；飞行日志显示确认发生的那一刻之前unlock_sta全程是1，
                                # 说明原逻辑单次读到0就退出，容易被单帧通信噪声/校验巧合触发误判
LASER_HEIGHT_MAX_M = 10.0  # 激光高度覆盖Z轴前的合理性上限：2026-07-10真机测试(basic_radar)发现降落末尾
                            # 激光传感器偶发返回类似0xFFFFFFFF的错误码，除以100后变成约4.29e7米的垃圾值，
                            # 原逻辑只判断laser_h>0.05、没有上限，会把这个垃圾值当真实高度写进pos[2]。
                            # 10m远超室内飞行实际高度(实测未超过1.4m)，只用来挡掉这种量级的错误码。
ARRIVAL_VEL_THRESH = 0.05  # 到达判定除了位置阈值外，还要求T265速度模长小于此值(m/s)，避免带着残余速度就触发land()盲降
ARRIVAL_VEL_WINDOW = 5  # 到达判定用的速度取最近N帧均值而非单帧瞬时值，平滑T265速度噪声尖峰
                         # (2026-07-07实测: 单帧瞬时速度噪声可达0.07m/s，用瞬时值+连续N次达标会导致到达确认永远凑不齐、超时强制跳过)
ARRIVAL_CONFIRM_RATIO = 0.6  # 到达确认改用滑动窗口比例制而非严格连续帧数：旧逻辑下任意一帧不达标就把
                              # 计数器清零重来，2026-07-08矩形路径测试实测达标帧占比只有30-40%，几乎不可能
                              # 连续凑够arrival_confirm_need帧，导致大多数航点靠超时兜底而非真正确认到达
                              # (2026-07-08复测: 0.8比例下仍有部分航点(占比26-34%)无法确认，下调到0.6)
TAKEOFF_WARN_LED_DURATION_S = 5.0  # 起飞前警示灯常亮时长(秒)，给周围人员留出充分反应时间

# 旧yaw方向开环脉冲诊断工具。正式飞行使用HeadingHoldController；两者互斥。
# 默认关闭；如需再次做方向诊断，必须同时显式关闭航向保持。
YAW_TEST_BURST_ENABLED = os.getenv("DRONE_YAW_TEST_BURST", "0") == "1"
YAW_TEST_BURST_VALUE = int(os.getenv("DRONE_YAW_TEST_BURST_VALUE", "-8"))
YAW_TEST_BURST_DURATION_S = float(os.getenv("DRONE_YAW_TEST_BURST_DURATION_S", "1.5"))

# 航向误差达到软故障阈值后的原地恢复门槛。正常航行仍使用
# 正常航行最多3deg/s；只有停止XY后才允许5deg/s恢复。
HEADING_RECOVERY_MAX_DPS = 5
HEADING_RECOVERY_TIMEOUT_S = 6.0
HEADING_RECOVERY_PROGRESS_WINDOW_S = 2.5
HEADING_RECOVERY_MIN_IMPROVEMENT_DEG = 0.3
HEADING_RECOVERY_STABLE_ENTER_DEG = 5.0
HEADING_RECOVERY_STABLE_EXIT_DEG = 5.5
HEADING_RECOVERY_STABLE_S = 0.5
HEADING_RECOVERY_HARD_ERROR_DEG = 20.0
HEADING_RECOVERY_MAX_SUCCESSES = 3

# 2026-07-22 C面实飞确认：货架低层实际偏转约24°时，飞控融合yaw仅变化约4°；
# T265变化与现场观察一致。因此仓储任务恢复使用T265 yaw闭环，FC yaw仅作旁路诊断。
HEADING_SOURCE = os.getenv("DRONE_HEADING_SOURCE", "t265").strip().lower()
if HEADING_SOURCE not in {"fc", "t265"}:
    raise ValueError("DRONE_HEADING_SOURCE只能是fc/t265")
FC_HEADING_MAX_AGE_S = 0.5
FC_HEADING_INVALID_TICKS = 3
FC_HEADING_MAX_STEP_DEG = 10.0
FC_HEADING_MIN_PREUNLOCK_FRAMES = 5
HEADING_NORMAL_MAX_DPS = 3
# 2026-07-09 非闭环真机脉冲确认：T265 yaw 与飞控命令同号，T265 闭环
# 必须原样发送控制器输出。2026-07-22 的 FC 反馈试飞显示 FC 数值轴与命令轴
# 相反，因此只在 FC 反馈模式反转，不能把该结论扩展到 T265。
HEADING_COMMAND_SIGN_BY_SOURCE = {"t265": 1, "fc": -1}


class HeadingFeedbackError(RuntimeError):
    """The selected heading feedback is unsafe to use for closed-loop flight."""


def arrival_window_confirmed(window, need, ratio):
    """window: 最近若干帧"位置+速度是否同时达标"的布尔值(deque)。
    窗口填满(len>=need)且达标帧占比>=ratio才算确认到达——替代旧的"严格连续N帧"
    逻辑，单帧噪声不会让已经积累的进度清零(见 ARRIVAL_CONFIRM_RATIO 常量注释)。"""
    return len(window) >= need and (sum(window) / len(window)) >= ratio


def laser_height_valid(laser_h):
    """激光高度是否合理，可以用来覆盖pos[2]/land_pos[2]。见 LASER_HEIGHT_MAX_M 注释：
    2026-07-10真机测试(basic_radar)捕获到传感器错误码(约0xFFFFFFFF/100)未被过滤污染日志的真实案例。"""
    return 0.05 < laser_h <= LASER_HEIGHT_MAX_M


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int],
                 realsense_obj: Optional[t265_class] = None,
                 serial_fc_ref=None, route_file: Optional[str] = None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.serial_fc_ref = serial_fc_ref

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False
        self.realsense = realsense_obj

        # XY PID + 独立航向保持外环
        self.x_pid = PID(0, 0)
        self.y_pid = PID(0, 0)
        self.heading_source = HEADING_SOURCE
        self.heading_hold = HeadingHoldController(HeadingHoldConfig.from_env())
        if self.heading_hold.config.enabled and YAW_TEST_BURST_ENABLED:
            raise ValueError("DRONE_HEADING_HOLD 与 DRONE_YAW_TEST_BURST 不能同时启用")
        self._heading_status = self.heading_hold.update(0.0, confidence=0, now=time.time())
        self._last_heading_fault_logged = None
        self._fc_heading_last_frame = None
        self._fc_heading_last_deg = None
        self._fc_heading_invalid_ticks = 0
        self._heading_feedback_age_s = None
        self._heading_feedback_failure_reason = None
        self._heading_t265_reference_deg = None
        self._heading_fc_reference_deg = None
        self._heading_t265_delta_deg = None
        self._heading_fc_delta_deg = None
        self._heading_source_disagreement_deg = None
        self._last_t265_confidence = None
        self._reset_heading_recovery_for_mission()
        self.navigation_profile = NavigationProfileConfig.from_env()

        # 航点
        self.targets = self.load_waypoints(route_file)
        self.target_index = 0
        self._scan_target = None
        self._scan_tick_last_t = None
        self._scan_tick_max_jitter_s = 0.0
        self._navigation_purpose = "normal"
        self._navigation_generation = 0
        self.emergency_stop = False
        self._takeoff_abort_reason = None

        # 到达判断
        self._arrival_window = deque(maxlen=arrival_confirm_need)
        self.arrival_start_time = 0.0
        self.arrival_confirmed_time: Optional[float] = None
        self.last_target_index = -1
        self._vel_window = deque(maxlen=ARRIVAL_VEL_WINDOW)
        self._cruise_arrival_count = 0
        self._active_segment_distance_m = 0.0

        # 高度 ramp
        self._ramp_z_cm = 0.0

        # 飞行数据日志
        self._log_file = None
        self._last_log_time = 0.0
        self._log_lock = threading.Lock()
        self._resource_monitor = ResourceMonitor()

        # yaw方向测试(问题16)
        self._yaw_burst_done = False

    def load_waypoints(self, route_file: Optional[str] = None):
        route_path = route_file or "router.txt"
        try:
            with open(route_path, 'r') as f:
                waypoints = []
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and not line.startswith('x'):
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                x = float(parts[0].strip())
                                y = float(parts[1].strip())
                                z = float(parts[2].strip())
                                waypoints.append([x, y, z])
                            except ValueError:
                                logger.warning(f"无效航点: {line}")
                if waypoints:
                    logger.info(f"加载 {len(waypoints)} 个航点")
                    return waypoints
        except FileNotFoundError:
            logger.warning(f"{route_path} 不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取 {route_path} 失败: {e}，使用默认航点")

        default = [[0.0, 0.0, put_height/100],
                   [0.5, 0.0, put_height/100],
                   [0.5, 0.5, put_height/100],
                   [0.0, 0.5, put_height/100]]
        return default

    # ================= 启动 =================
    def start(self):
        self.heading_hold.reset_for_new_mission()
        self._last_heading_fault_logged = None
        self._fc_heading_last_frame = None
        self._fc_heading_last_deg = None
        self._fc_heading_invalid_ticks = 0
        self._heading_feedback_age_s = None
        self._heading_feedback_failure_reason = None
        self._heading_t265_reference_deg = None
        self._heading_fc_reference_deg = None
        self._heading_t265_delta_deg = None
        self._heading_fc_delta_deg = None
        self._heading_source_disagreement_deg = None
        self._last_t265_confidence = None
        self._reset_heading_recovery_for_mission()
        self.t265_ok = False
        # 按键门禁已在 main.py 中完成；蓝灯表示正在初始化，不是第二次按键门禁。
        self._set_status_led("B")
        if DRY_RUN:
            logger.warning("=" * 40)
            logger.warning("DRY_RUN 模式已启用 — 飞控不会解锁，电机不会转")
            logger.warning("=" * 40)

        if self.realsense and self.realsense.start():
            self.realsense.autoset()
            self.t265_ok = True
            logger.info("T265 OK")

            # 等待追踪置信度稳定，避免T265刚连上、置信度还没起来就进入定点模式
            t_wait_start = time.time()
            confidence = self.realsense.get_tracking_confidence()
            while confidence < T265_CONFIDENCE_MIN and time.time() - t_wait_start < T265_CONFIDENCE_WAIT_S:
                time.sleep(0.2)
                confidence = self.realsense.get_tracking_confidence()

            if confidence < T265_CONFIDENCE_MIN:
                logger.error(
                    f"T265 置信度 {T265_CONFIDENCE_WAIT_S:.0f} 秒内仍偏低"
                    f"(confidence={confidence})，任务取消；仓储盘点禁止降级起飞"
                )
                self.realsense.stop()
                self.t265_ok = False
                self._set_status_led("OFF")
                return
            else:
                logger.info(f"T265 追踪置信度已稳定 (confidence={confidence})")
        else:
            logger.error("T265 FAILED — 仓储盘点禁止仅高度模式起飞，任务取消")
            self._set_status_led("OFF")
            return

        if not self._blink_warning_led():
            logger.error("起飞前红灯警示失败，任务取消；飞控不会解锁")
            if self.realsense and self.t265_ok:
                self.realsense.stop()
            return

        logger.info(
            f"任务启动, {len(self.targets)} 个航点 | 导航={self.navigation_profile.profile}"
            f" | 航向保持={'开启' if self.heading_hold.config.enabled else '关闭'}"
        )

        self.task_running = True
        self.state = "TAKEOFF"

        try:
            path = os.path.dirname(os.path.realpath(sys.argv[0]))
            self._log_file = open(path + "/flight_data.jsonl", "a")
            with self._log_lock:
                self._log_file.write(json.dumps({
                    "event": "task_start",
                    "nav_profile": self.navigation_profile.profile,
                    "heading_hold_enabled": self.heading_hold.config.enabled,
                }) + "\n")
                self._log_file.flush()
        except Exception:
            pass

        self._resource_monitor.start(self._log_file, self._log_lock)

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def _log_flight_loop_exception(self, exc, failed_state):
        if not self._log_file:
            return
        try:
            target = (
                list(self.targets[self.target_index])
                if self.target_index < len(self.targets)
                else None
            )
            with self._log_lock:
                self._log_file.write(json.dumps({
                    "t": round(time.time(), 3),
                    "event": "flight_loop_exception",
                    "state": failed_state,
                    "exception_type": type(exc).__name__,
                    "error": str(exc),
                    "navigation_purpose": self._navigation_purpose,
                    "target_idx": self.target_index,
                    "target": target,
                }) + "\n")
                self._log_file.flush()
        except Exception:
            pass

    def on_flight_loop_exception(self, exc, failed_state):
        yaw_cmd = getattr(self._heading_status, "command_dps", 0)
        z_setpoint = self.se_fc[5] if len(self.se_fc) > 5 else int(self._ramp_z_cm)
        self.set_speed(0, 0, yaw_cmd, z_setpoint)
        if failed_state == "LAND":
            with lock:
                self.se_fc[2] = 0
                self.se_fc[3] = sp_side
                self.se_fc[4] = sp_side
                self.se_fc[6] = sp_side
            self.emergency_stop = True
        else:
            self.state = "LAND"

    def _flight_loop_unrecoverable(self):
        try:
            with lock:
                self.se_fc[2] = 0
                self.se_fc[3] = sp_side
                self.se_fc[4] = sp_side
                self.se_fc[6] = sp_side
                self.se_fc[7] = 101
        except Exception:
            pass
        finally:
            self.task_running = False

    def _dispatch_state_tick(self, pos, yaw):
        if self.state == "TAKEOFF":
            self.takeoff()
        elif self.state == "NAVIGATE":
            self.navigate(pos, yaw)
        elif self.state == "SCAN":
            self.scan_tick(pos, yaw)
        elif self.state == "LAND":
            self.land()
        elif self.state == "END":
            self.stop_all()

    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # FC 串口超时
            if fc_last_rx_time.value > 0 and time.time() - fc_last_rx_time.value > 2.0:
                logger.error("飞控串口超时 2s，紧急降落")
                self.emergency_stop = True
                continue

            # T265 存活
            if self.t265_ok and self.realsense and not self.realsense.is_running():
                logger.error("T265 已停止，紧急降落")
                self.emergency_stop = True
                continue

            # 获取位置
            pos = [0.0, 0.0, 0.0]
            yaw = 0.0
            if self.realsense:
                try:
                    pos = list(self.realsense.get_position())
                    yaw = self.realsense.get_orientation()[2]
                except Exception:
                    logger.error("T265 读取失败")
                    time.sleep(0.03)
                    continue

            # 激光高度覆盖 Z
            if self.serial_fc_ref is not None:
                with lock:
                    laser_h = self.serial_fc_ref._last_laser_height_cm
                if laser_height_valid(laser_h):
                    pos[2] = laser_h

            # 状态机
            failed_state = self.state
            try:
                self._dispatch_state_tick(pos, yaw)
            except Exception as exc:
                logger.exception(
                    f"飞行主循环状态处理异常(state={failed_state})，进入安全处理: {exc}"
                )
                self._log_flight_loop_exception(exc, failed_state)
                try:
                    self.on_flight_loop_exception(exc, failed_state)
                except Exception:
                    logger.critical("飞行异常处理器再次失败，执行裸写锁定保底", exc_info=True)
                    self._flight_loop_unrecoverable()

            time.sleep(0.03)

    # ================= 起飞 =================
    @staticmethod
    def _set_status_led(color):
        """设置初始化状态灯；状态灯故障不改变飞行安全门禁结果。"""
        try:
            from Lcode.gpio_led import set_rgb_led
            return bool(set_rgb_led(color))
        except Exception as exc:
            logger.warning(f"状态灯{color}设置失败: {exc}")
            return False

    def _blink_warning_led(self):
        """起飞前红灯常亮TAKEOFF_WARN_LED_DURATION_S秒提醒周围人员，阻塞调用
        (起飞前的安全等待本来就该是阻塞的，给人反应时间)。GPIO不可用时静默
        失败时返回False，由按钮门禁阻断起飞。"""
        try:
            from Lcode.gpio_led import set_rgb_led
        except Exception as e:
            logger.error(f"起飞警示灯点亮失败: {e}")
            return False
        try:
            if not set_rgb_led('R'):
                return False
            time.sleep(TAKEOFF_WARN_LED_DURATION_S)
            return bool(set_rgb_led('OFF'))
        except Exception as e:
            logger.error(f"起飞警示灯控制失败: {e}")
            try:
                set_rgb_led('OFF')
            except Exception:
                pass
            return False

    def takeoff(self):
        if DRY_RUN:
            logger.warning("takeoff: DRY_RUN 模式，不发送解锁指令，电机不会转")
        else:
            logger.info("takeoff: started")

        # 解锁前最后一次检查T265。红灯5秒期间设备可能掉追踪；仓储路线没有
        # 仅高度模式的安全退化能力，所以此门禁失败时绝不能发送解锁指令。
        if not (self.t265_ok and self.realsense):
            self._abort_takeoff_safely("t265_unavailable")
            return
        try:
            takeoff_confidence = self.realsense.get_tracking_confidence()
            takeoff_yaw = self.realsense.get_orientation()[2]
        except Exception as exc:
            logger.error(f"takeoff: 解锁前读取T265失败: {exc}")
            self._abort_takeoff_safely("t265_preunlock_read_error")
            return
        if takeoff_confidence < T265_CONFIDENCE_MIN:
            self._abort_takeoff_safely(
                f"t265_confidence_{takeoff_confidence}"
            )
            return

        try:
            self._heading_status = self._arm_heading_hold(takeoff_yaw)
        except HeadingFeedbackError as exc:
            self._abort_takeoff_safely(str(exc))
            return
        if self.heading_hold.config.enabled:
            logger.info(
                f"航向保持已锁存起飞方向 "
                f"{self._heading_status.target_deg:+.2f}°"
                f" (source={self.heading_source})"
            )

        target_h_cm = TAKEOFF_LIFTOFF_CM  # 一键起飞只爬升到离地高度，真正目标高度交给 navigate() 闭环爬升

        with lock:
            self.se_fc[5] = int(target_h_cm)  # com_z：一键起飞目标高度，必须在 task_sta 触发前写入，
            self.se_fc[2] = 0 if DRY_RUN else 1  # 否则飞控读到的是 se_fc 初始默认值(120cm)而非本次航点高度

        confirm_count = 0
        t_start = time.time()
        confidence_lost_since = None

        while True:
            elapsed = time.time() - t_start

            yaw = 0.0
            yaw_cmd = 0
            if self.t265_ok and self.realsense:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    confidence = self.realsense.get_tracking_confidence()
                    if confidence < T265_CONFIDENCE_MIN:
                        if confidence_lost_since is None:
                            confidence_lost_since = time.time()
                            logger.warning(
                                f"takeoff: T265置信度丢失(confidence={confidence})，暂停起飞确认"
                            )
                        elif time.time() - confidence_lost_since >= TAKEOFF_CONFIDENCE_ABORT_S:
                            self._abort_takeoff_safely(
                                f"t265_confidence_{confidence}"
                            )
                            return
                    else:
                        confidence_lost_since = None
                    self._heading_status = self._update_heading_hold(yaw, confidence)
                    yaw_cmd = self._heading_status.command_dps
                    with lock:
                        self.se_fc[6] = yaw_cmd + sp_side
                except HeadingFeedbackError as e:
                    logger.error(f"takeoff: 飞控航向反馈失效: {e}")
                    self._abort_takeoff_safely(str(e))
                    return
                except Exception as e:
                    logger.error(f"takeoff: 起飞阶段T265读取失败: {e}")
                    self._abort_takeoff_safely("t265_takeoff_read_error")
                    return
            else:
                with lock:
                    self.se_fc[6] = sp_side

            with lock:
                laser_m = self.serial_fc_ref._last_laser_height_cm if self.serial_fc_ref else 0.0
            laser_cm = laser_m * 100.0

            # 排查起飞离地阶段yaw自稳是否正确（2026-07-06 15cm离地测试观察到水平旋转，
            # 怀疑本函数的vyaw符号跟navigate()相反）：记录T265原始yaw、飞控自己融合的yaw(re_fc[3])、
            # 算出来的修正指令，两路yaw互相交叉验证谁的读数有问题
            with lock:
                fc_yaw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else 0.0
            if self._log_file:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(time.time(), 3),
                            "state": "TAKEOFF",
                            "t265_yaw_deg": round(math.degrees(yaw), 2),
                            "fc_yaw_deg": round(fc_yaw_deg, 2),
                            "yaw_cmd_sent": yaw_cmd,
                            "laser_cm": round(laser_cm, 1),
                            **self._heading_log_fields(),
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass

            if laser_cm > 5.0 and abs(laser_cm - target_h_cm) <= 10.0:
                confirm_count += 1
            else:
                confirm_count = 0

            if confirm_count >= TAKEOFF_CONFIRM_NEED:
                logger.info(f"takeoff: 高度确认 {laser_cm:.0f} cm")
                break

            if elapsed >= TAKEOFF_TIMEOUT_S:
                self._abort_takeoff_safely("liftoff_height_timeout")
                return

            time.sleep(0.03)

        self._ramp_z_cm = target_h_cm
        self.state = "NAVIGATE"

    def _abort_takeoff_safely(self, reason):
        """Stop the takeoff attempt and route it through the normal landing path."""
        self._takeoff_abort_reason = str(reason)
        logger.error(f"takeoff: 未确认安全离地，进入安全降落(reason={reason})")
        with lock:
            self.se_fc[2] = 0
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[5] = 0
            self.se_fc[6] = sp_side
        self._ramp_z_cm = 0.0
        self.state = "LAND"

    # ================= 导航 =================
    def _reset_heading_recovery_for_mission(self):
        self._heading_recovery_active = False
        self._heading_recovery_started_at = None
        self._heading_recovery_start_abs_error = None
        self._heading_recovery_stable_since = None
        self._heading_recovery_hold_z_cm = None
        self._heading_recovery_successes = 0
        self._heading_recovery_failure_reason = None
        self._heading_bypass_active = False

    def on_heading_recovery_started(self, source_state):
        """Extension hook for mission-specific scan cancellation."""

    def heading_recovery_ready_to_resume(self):
        """Extension hook that can delay resume until background work exits."""
        return True

    def on_heading_recovery_completed(self, source_state):
        """Extension hook invoked after the original heading has recovered."""

    def on_heading_recovery_failed(self, reason):
        """Extension hook invoked before switching the flight driver to LAND."""

    def _fail_heading_recovery(self, reason, yaw, hold_z_cm):
        self._heading_recovery_failure_reason = str(reason)
        self._heading_recovery_active = False
        self._heading_recovery_stable_since = None
        try:
            self._heading_status = self._heading_recovery_status(yaw)
        except HeadingFeedbackError:
            # 反馈本身已失效时绝不能再根据另一来源生成恢复转向指令。
            pass
        self.set_speed(0, 0, 0, int(hold_z_cm))
        reason_text = str(reason)
        critical_feedback_failure = (
            reason_text.startswith("heading_recovery_low_confidence_")
            or reason_text == "heading_error_unavailable"
            or reason_text.startswith("fc_heading_")
        )
        if not critical_feedback_failure:
            # 用户明确要求：真实yaw偏转不能中止盘点。恢复失败后只关闭本次任务的
            # 航向外环；从下一个控制tick起继续XY/Z导航，yaw指令保持为0。
            self._heading_bypass_active = True
            self.heading_hold.disarm("heading_fault_bypassed")
            self._heading_status = replace(
                self._heading_status,
                command_dps=0,
                armed=False,
                degraded_reason="heading_fault_bypassed",
                fault_reason=None,
            )
            logger.warning(f"航向恢复失败，关闭航向外环并继续任务: {reason_text}")
            return {
                "active": False,
                "failed": False,
                "completed": False,
                "bypassed": True,
                "reason": reason_text,
                "yaw_cmd": 0,
                "z_setpoint_cm": int(hold_z_cm),
            }

        logger.error(f"航向反馈失效，转入降落: {reason_text}")
        self.on_heading_recovery_failed(reason_text)
        self.state = "LAND"
        return {
            "active": False,
            "failed": True,
            "reason": str(reason),
            "yaw_cmd": 0,
            "z_setpoint_cm": int(hold_z_cm),
        }

    def _heading_recovery_tick(
        self, yaw, pos_z_m, status, source_state, confidence
    ):
        now = time.time()
        if confidence < T265_CONFIDENCE_MIN:
            return self._fail_heading_recovery(
                f"heading_recovery_low_confidence_{confidence}",
                yaw,
                self._heading_recovery_hold_z_cm or self._ramp_z_cm,
            )
        error_deg = status.error_deg
        if error_deg is None:
            return self._fail_heading_recovery(
                "heading_error_unavailable", yaw, self._ramp_z_cm
            )
        abs_error = abs(error_deg)

        if not self._heading_recovery_active:
            if self._heading_recovery_successes >= HEADING_RECOVERY_MAX_SUCCESSES:
                return self._fail_heading_recovery(
                    "heading_recovery_repeat_limit", yaw, self._ramp_z_cm
                )
            self._heading_recovery_active = True
            self._heading_recovery_started_at = now
            self._heading_recovery_start_abs_error = abs_error
            self._heading_recovery_stable_since = None
            self._heading_recovery_hold_z_cm = max(0, int(round(float(pos_z_m) * 100)))
            self._heading_recovery_failure_reason = None
            logger.warning(
                f"航向恢复开始: error={error_deg:+.2f}deg, "
                f"hold_z={self._heading_recovery_hold_z_cm}cm"
            )
            self.on_heading_recovery_started(source_state)

        hold_z_cm = self._heading_recovery_hold_z_cm
        elapsed = now - self._heading_recovery_started_at
        if abs_error >= HEADING_RECOVERY_HARD_ERROR_DEG:
            return self._fail_heading_recovery(
                f"heading_error_{error_deg:+.2f}deg_exceeds_recovery_limit",
                yaw,
                hold_z_cm,
            )
        if elapsed >= HEADING_RECOVERY_TIMEOUT_S:
            return self._fail_heading_recovery(
                "heading_recovery_timeout", yaw, hold_z_cm
            )
        if elapsed >= HEADING_RECOVERY_PROGRESS_WINDOW_S:
            improvement = self._heading_recovery_start_abs_error - abs_error
            if improvement < HEADING_RECOVERY_MIN_IMPROVEMENT_DEG:
                return self._fail_heading_recovery(
                    f"heading_recovery_no_progress_{improvement:+.2f}deg",
                    yaw,
                    hold_z_cm,
                )

        if self._heading_recovery_stable_since is None:
            if abs_error <= HEADING_RECOVERY_STABLE_ENTER_DEG:
                self._heading_recovery_stable_since = now
        elif abs_error > HEADING_RECOVERY_STABLE_EXIT_DEG:
            self._heading_recovery_stable_since = None
        elif now - self._heading_recovery_stable_since >= HEADING_RECOVERY_STABLE_S:
            if self.heading_recovery_ready_to_resume():
                self._heading_status = self._clear_heading_fault(yaw)
                self._heading_recovery_active = False
                self._heading_recovery_successes += 1
                self._heading_recovery_stable_since = None
                logger.info(
                    f"航向恢复完成: error={error_deg:+.2f}deg, "
                    f"successes={self._heading_recovery_successes}"
                )
                self.on_heading_recovery_completed(source_state)
                self.set_speed(0, 0, 0, int(hold_z_cm))
                return {
                    "active": False,
                    "failed": False,
                    "completed": True,
                    "reason": None,
                    "yaw_cmd": 0,
                    "z_setpoint_cm": int(hold_z_cm),
                }

        self._heading_status = self._heading_recovery_status(yaw)
        yaw_cmd = self._heading_status.command_dps
        self.set_speed(0, 0, yaw_cmd, int(hold_z_cm))
        return {
            "active": True,
            "failed": False,
            "completed": False,
            "reason": None,
            "yaw_cmd": yaw_cmd,
            "z_setpoint_cm": int(hold_z_cm),
        }

    def position_control_tick(self, target, pos, yaw):
        """Run one reusable XY PID, Z ramp, and heading-hold control tick."""
        confidence = (
            self.realsense.get_tracking_confidence()
            if self.t265_ok and self.realsense
            else 0
        )
        source_state = self.state
        try:
            self._heading_status = self._update_heading_hold(
                yaw, confidence, allow_fault_relock=False
            )
        except HeadingFeedbackError as exc:
            reason = str(exc)
            self._heading_feedback_failure_reason = reason
            logger.error(f"航向反馈失效，当前tick清零并转入降落: {reason}")
            hold_z_cm = max(0, int(round(float(pos[2]) * 100)))
            self.set_speed(0, 0, 0, hold_z_cm)
            self.on_heading_recovery_failed(reason)
            self.state = "LAND"
            return {
                "confidence": confidence,
                "vx": 0,
                "vy": 0,
                "yaw_cmd": 0,
                "z_setpoint_cm": hold_z_cm,
                "heading_recovery_active": False,
                "heading_recovery_failed": True,
                "heading_recovery_completed": False,
                "heading_recovery_reason": reason,
            }
        yaw_cmd = self._heading_status.command_dps

        if (
            self.heading_source == "fc"
            and self._heading_feedback_failure_reason is not None
        ):
            # 允许最多两个控制 tick 的串口调度毛刺，但毛刺期间也不能继续 XY/航点推进。
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            return {
                "confidence": confidence,
                "vx": 0,
                "vy": 0,
                "yaw_cmd": 0,
                "z_setpoint_cm": int(self._ramp_z_cm),
                "heading_recovery_active": True,
                "heading_recovery_failed": False,
                "heading_recovery_completed": False,
                "heading_recovery_reason": self._heading_feedback_failure_reason,
            }

        if self._heading_status.fault_reason or self._heading_recovery_active:
            try:
                recovery = self._heading_recovery_tick(
                    yaw, pos[2], self._heading_status, source_state, confidence
                )
            except HeadingFeedbackError as exc:
                recovery = self._fail_heading_recovery(
                    str(exc), yaw, max(0, int(round(float(pos[2]) * 100)))
                )
                return {
                    "confidence": confidence,
                    "vx": 0,
                    "vy": 0,
                    "yaw_cmd": recovery["yaw_cmd"],
                    "z_setpoint_cm": recovery["z_setpoint_cm"],
                    "heading_recovery_active": recovery["active"],
                    "heading_recovery_failed": recovery["failed"],
                    "heading_recovery_completed": recovery.get("completed", False),
                    "heading_recovery_reason": recovery["reason"],
                }
            return {
                "confidence": confidence,
                "vx": 0,
                "vy": 0,
                "yaw_cmd": recovery["yaw_cmd"],
                "z_setpoint_cm": recovery["z_setpoint_cm"],
                "heading_recovery_active": recovery["active"],
                "heading_recovery_failed": recovery["failed"],
                "heading_recovery_completed": recovery.get("completed", False),
                "heading_recovery_reason": recovery["reason"],
            }

        if confidence == 0 and self.t265_ok:
            self.set_speed(0, 0, yaw_cmd, int(self._ramp_z_cm))
            return None

        if self.t265_ok and self.realsense:
            self.x_pid.set_target(target[0])
            self.y_pid.set_target(target[1])
            vx = int(self.limit(self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE, 40))
            vy = int(self.limit(self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE, 40))
        else:
            vx, vy = 0, 0

        self._step_ramp_z(int(target[2] * 100))
        self.set_speed(vx, vy, yaw_cmd, int(self._ramp_z_cm))
        return {
            "confidence": confidence,
            "vx": vx,
            "vy": vy,
            "yaw_cmd": yaw_cmd,
            "z_setpoint_cm": int(self._ramp_z_cm),
            "heading_recovery_active": False,
            "heading_recovery_failed": False,
            "heading_recovery_completed": False,
            "heading_recovery_reason": None,
        }

    def begin_scan_hold(self, target):
        """Latch an explicit hold target and enter SCAN control."""
        if len(target) != 3:
            raise ValueError("scan target must contain x, y, z")
        self._scan_target = tuple(float(value) for value in target)
        self.state = "SCAN"

    def end_scan_hold(self):
        self._scan_target = None

    def on_scan_tick(self, pos, yaw, control):
        """Extension hook invoked after a successful SCAN control tick."""

    def on_scan_tracking_lost(self, pos, yaw):
        """Default SCAN safety action when position control is unavailable."""
        self.state = "LAND"

    def _record_scan_tick_jitter(self):
        now = time.monotonic()
        if self._scan_tick_last_t is not None:
            interval = now - self._scan_tick_last_t
            self._scan_tick_max_jitter_s = max(
                self._scan_tick_max_jitter_s,
                max(0.0, interval - 0.03),
            )
        self._scan_tick_last_t = now

    def _log_control_tick(self, state, target, pos, yaw, control):
        """Log SCAN and heading-recovery ticks with navigation-equivalent fields."""
        now = time.time()
        if not self._log_file or now - self._last_log_time < FLIGHT_LOG_INTERVAL:
            return
        try:
            tv = (
                self.realsense.get_velocity()
                if self.t265_ok and self.realsense
                else (0.0, 0.0, 0.0)
            )
            with lock:
                of1_dx = self.re_fc[9] if len(self.re_fc) > 9 else 0
                of1_dy = self.re_fc[10] if len(self.re_fc) > 10 else 0
                roll_deg = self.re_fc[1] / 100.0 if len(self.re_fc) > 1 else 0.0
                pitch_deg = self.re_fc[2] / 100.0 if len(self.re_fc) > 2 else 0.0
                fc_yaw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else 0.0
                of_quality = self.re_fc[11] if len(self.re_fc) > 11 else 0
                of_link_sta = self.re_fc[12] if len(self.re_fc) > 12 else 0
                of_work_sta = self.re_fc[13] if len(self.re_fc) > 13 else 0
            entry = {
                "t": round(now, 3),
                "state": str(state),
                "target_idx": self.target_index,
                "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                "vx": control["vx"],
                "vy": control["vy"],
                "yaw_cmd_sent": control["yaw_cmd"],
                "t265_yaw_deg": round(math.degrees(yaw), 2),
                "fc_yaw_deg": round(fc_yaw_deg, 2),
                "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                "of1_vel_cms": [of1_dx, of1_dy],
                "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                "height_setpoint_cm": control["z_setpoint_cm"],
                "of_status": [of_quality, of_link_sta, of_work_sta],
                "nav_profile": self.navigation_profile.profile,
                "waypoint_mode": "scan" if state == "SCAN" else self._waypoint_mode(),
                "arrival_distance_m": round(
                    math.hypot(pos[0] - target[0], pos[1] - target[1]), 4
                ),
                **self._heading_log_fields(),
            }
            with self._log_lock:
                self._log_file.write(json.dumps(entry) + "\n")
                self._log_file.flush()
        except Exception:
            pass
        self._last_log_time = now

    def scan_tick(self, pos, yaw):
        self._record_scan_tick_jitter()
        target = self._scan_target
        if target is None:
            self.on_scan_tracking_lost(pos, yaw)
            return
        try:
            control = self.position_control_tick(target, pos, yaw)
            if control is None:
                self.on_scan_tracking_lost(pos, yaw)
                return
            self._log_control_tick("SCAN", target, pos, yaw, control)
            if (
                control["heading_recovery_active"]
                or control["heading_recovery_failed"]
                or control["heading_recovery_completed"]
            ):
                return
            self.on_scan_tick(pos, yaw, control)
        except Exception as exc:
            logger.exception(f"SCAN控制tick异常，转入安全降落: {exc}")
            self.state = "LAND"

    def navigate(self, pos, yaw):
        if self.target_index >= len(self.targets):
            logger.info("全部航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]

        waypoint_mode = self._waypoint_mode()
        arrival_distance = math.hypot(pos[0] - target[0], pos[1] - target[1])

        if self.target_index != self.last_target_index:
            self._reset_arrival_tracking(pos)

        control = self.position_control_tick(target, pos, yaw)
        yaw_cmd = self._heading_status.command_dps
        if control is None:
            confidence = 0
            logger.warning("T265 追踪丢失，悬停等待")
            # 定位丢失期间暂停航点超时和到达确认。否则恢复追踪后的第一帧可能
            # 带着失联前的旧计时立即跳点，或沿用过期的确认窗口。
            self.arrival_start_time = time.time()
            self._arrival_window.clear()
            self._vel_window.clear()
            self.arrival_confirmed_time = None
            self._cruise_arrival_count = 0
            # 2026-07-09从basic_radar/补同步(2026-07-08已在那边修复)：这里原本直接return
            # 会跳过日志写入，导致T265追踪丢失期间完全没有数据记录。
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(now, 3),
                            "state": self.state,
                            "target_idx": self.target_index,
                            "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                            "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                            "vx": 0, "vy": 0, "yaw_cmd_sent": yaw_cmd,
                            "t265_yaw_deg": round(math.degrees(yaw), 2),
                            "height_setpoint_cm": round(self._ramp_z_cm, 1),
                            "t265_confidence_lost": True,
                            "nav_profile": self.navigation_profile.profile,
                            "waypoint_mode": waypoint_mode,
                            "arrival_distance_m": round(arrival_distance, 4),
                            **self._heading_log_fields(),
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
            return

        if (
            control["heading_recovery_active"]
            or control["heading_recovery_failed"]
            or control["heading_recovery_completed"]
        ):
            # 恢复期间不允许航点超时、到达窗口或PID积分继续累积。
            self.arrival_start_time = time.time()
            self._arrival_window.clear()
            self._vel_window.clear()
            self.arrival_confirmed_time = None
            self._cruise_arrival_count = 0
            self.x_pid.reset()
            self.y_pid.reset()
            self._log_control_tick("NAVIGATE", target, pos, yaw, control)
            return

        confidence = control["confidence"]
        vx = control["vx"]
        vy = control["vy"]
        yaw_cmd = control["yaw_cmd"]

        # T265 速度（到达检测的速度门槛 + 后面日志/终端输出共用，避免重复取值）
        if self.t265_ok and self.realsense:
            tv = self.realsense.get_velocity()
        else:
            tv = (0.0, 0.0, 0.0)

        # 到达检测
        if self.t265_ok and self.realsense:
            xy_thresh = 0.10 if confidence >= 3 else (posthreshold_xy if confidence == 2 else 0.30)
            dx = abs(pos[0] - target[0])
            dy = abs(pos[1] - target[1])
            dz = abs(pos[2] - target[2])
            # 速度用最近N帧均值而非瞬时值，平滑T265速度噪声尖峰(见 ARRIVAL_VEL_WINDOW 注释)
            self._vel_window.append((tv[0], tv[1]))
            avg_vx = sum(v[0] for v in self._vel_window) / len(self._vel_window)
            avg_vy = sum(v[1] for v in self._vel_window) / len(self._vel_window)
            speed = math.hypot(avg_vx, avg_vy)

            if dx > 0.3:
                self.x_pid.reset()
            if dy > 0.3:
                self.y_pid.reset()

            if waypoint_mode == "precision":
                frame_ok = (
                    dx < xy_thresh
                    and dy < xy_thresh
                    and dz < posthreshold_z
                    and speed < ARRIVAL_VEL_THRESH
                )
                self._arrival_window.append(frame_ok)
                if arrival_window_confirmed(
                    self._arrival_window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO
                ):
                    if self.arrival_confirmed_time is None:
                        self.arrival_confirmed_time = time.time()
                        logger.info(
                            f"到达航点 {self.target_index}，停留 {arrival_hold_s:.0f}s 观察"
                        )
                    elif time.time() - self.arrival_confirmed_time >= arrival_hold_s:
                        logger.info(f"航点 {self.target_index} 停留完成")
                        self._advance_waypoint("precision_arrival", pos, target, arrival_distance)
                        return
                else:
                    self.arrival_confirmed_time = None
            else:
                cruise_position_ok = arrival_distance <= self.navigation_profile.cruise_radius_m
                if (
                    self.navigation_profile.cruise_require_z
                    or self._navigation_purpose == "return"
                ):
                    cruise_position_ok = cruise_position_ok and dz < posthreshold_z
                if cruise_position_ok:
                    self._cruise_arrival_count += 1
                else:
                    self._cruise_arrival_count = 0
                if (
                    self._cruise_arrival_count
                    >= self.navigation_profile.cruise_confirm_cycles
                ):
                    logger.info(f"航点 {self.target_index} 掠过(巡航航点，不停留)")
                    self._advance_waypoint("cruise_arrival", pos, target, arrival_distance)
                    return

            timeout_s = self._waypoint_timeout_s(waypoint_mode)
            if time.time() - self.arrival_start_time >= timeout_s:
                if self._navigation_purpose == "return":
                    is_intermediate = self.target_index < len(self.targets) - 1
                    near_target = (
                        confidence >= T265_CONFIDENCE_MIN
                        and arrival_distance <= self.navigation_profile.cruise_radius_m
                        and dz < posthreshold_z
                    )
                    if RETURN_CRUISE_ENABLED and is_intermediate and near_target:
                        logger.warning(
                            f"返航中间航点 {self.target_index} 超时但已接近，继续下一航点"
                        )
                        self._advance_waypoint(
                            "return_timeout_near", pos, target, arrival_distance
                        )
                    else:
                        logger.warning(
                            f"返航航点 {self.target_index} 超时且未满足安全推进条件，"
                            "切换当前位置受控降落"
                        )
                        self.state = "LAND"
                    return
                logger.warning(f"航点 {self.target_index} 超时，强制跳过")
                self._advance_waypoint("timeout", pos, target, arrival_distance)
                return

        # 光流融合速度（帧1 of1_dx/dy，用于跟 T265 速度交叉对比）
        # + roll/pitch（帧1 已回传，用于排查高度控制异常是否跟倾角同步，见 CLAUDE.md 已知问题6）
        # + 光流质量/状态（排查异常是否由光流信号本身变差导致）
        with lock:
            of1_dx = self.re_fc[9] if len(self.re_fc) > 9 else 0
            of1_dy = self.re_fc[10] if len(self.re_fc) > 10 else 0
            roll_deg = self.re_fc[1] / 100.0 if len(self.re_fc) > 1 else 0.0
            pitch_deg = self.re_fc[2] / 100.0 if len(self.re_fc) > 2 else 0.0
            fc_yaw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else 0.0
            of_quality = self.re_fc[11] if len(self.re_fc) > 11 else 0
            of_link_sta = self.re_fc[12] if len(self.re_fc) > 12 else 0
            of_work_sta = self.re_fc[13] if len(self.re_fc) > 13 else 0

        # 日志
        now = time.time()
        if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                with self._log_lock:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": vx, "vy": vy, "yaw_cmd_sent": yaw_cmd,
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "fc_yaw_deg": round(fc_yaw_deg, 2),
                        "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                        "of1_vel_cms": [of1_dx, of1_dy],
                        "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "of_status": [of_quality, of_link_sta, of_work_sta],
                        "nav_profile": self.navigation_profile.profile,
                        "waypoint_mode": waypoint_mode,
                        "arrival_distance_m": round(arrival_distance, 4),
                        **self._heading_log_fields(),
                    }) + "\n")
                    self._log_file.flush()
            except Exception:
                pass
            self._last_log_time = now

        # 终端输出
        if self.t265_ok and self.realsense:
            t265_str = f"| t265v=({tv[0]:+.2f},{tv[1]:+.2f}) | of1=({of1_dx:+d},{of1_dy:+d})"
        else:
            t265_str = ""
        t265_str += f" | att=({roll_deg:+.1f},{pitch_deg:+.1f})"
        print(
            f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"| tgt=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) "
            f"| v=({vx:>3},{vy:>3}) "
            f"| send=({self.se_fc[3]:>3},{self.se_fc[4]:>3},{self.se_fc[5]:>3})"
            f" | mode={waypoint_mode[:4]} yawerr={self._format_heading_error():>5}"
            f" cmd={yaw_cmd:+d}"
            f"{t265_str}",
            end="", flush=True
        )

    # ================= 到达处理 =================
    def _waypoint_mode(self, target_index=None):
        index = self.target_index if target_index is None else int(target_index)
        if (
            RETURN_CRUISE_ENABLED
            and self._navigation_purpose == "return"
            and index < len(self.targets) - 1
        ):
            return "cruise"
        return self.navigation_profile.waypoint_mode(index, len(self.targets))

    def replace_navigation_targets(self, new_targets, current_pos, *, purpose="normal"):
        """Atomically install a new route and reset route-specific control state."""
        normalized = [tuple(float(value) for value in target) for target in new_targets]
        if not normalized:
            raise ValueError("navigation targets cannot be empty")
        if any(len(target) != 3 for target in normalized):
            raise ValueError("each navigation target must contain x, y, z")

        self.targets = normalized
        self.target_index = 0
        self.last_target_index = -1
        self._arrival_window.clear()
        self._vel_window.clear()
        self.arrival_confirmed_time = None
        self.arrival_start_time = time.time()
        self._cruise_arrival_count = 0
        self._active_segment_distance_m = math.hypot(
            current_pos[0] - normalized[0][0],
            current_pos[1] - normalized[0][1],
        )
        self.x_pid.reset()
        self.y_pid.reset()
        self._navigation_purpose = str(purpose)
        self._navigation_generation += 1
        return self._navigation_generation

    def _advance_waypoint(self, reason, pos, target, arrival_distance):
        """统一推进航点并原子重置到达状态，避免同tick连跳和日志目标错位。"""
        completed_index = self.target_index
        if YAW_TEST_BURST_ENABLED and self.target_index == 0 and not self._yaw_burst_done:
            self._yaw_burst_done = True
            self._do_yaw_test_burst()
        self._log_waypoint_event(
            reason, completed_index, pos, target, arrival_distance
        )
        self.target_index += 1
        if self.target_index < len(self.targets):
            # 直接用当前切点位置初始化下一航段；下一tick无需再次重置，超时从
            # 真正切换目标的时刻开始计算。
            self._reset_arrival_tracking(pos)
        else:
            self.last_target_index = self.target_index
            self._arrival_window.clear()
            self._vel_window.clear()
            self.arrival_confirmed_time = None
            self.arrival_start_time = time.time()
            self._cruise_arrival_count = 0
            self._active_segment_distance_m = 0.0

    def _reset_arrival_tracking(self, pos):
        self.last_target_index = self.target_index
        self._arrival_window.clear()
        self._vel_window.clear()
        self.arrival_confirmed_time = None
        self.arrival_start_time = time.time()
        self._cruise_arrival_count = 0
        target = self.targets[self.target_index]
        self._active_segment_distance_m = math.hypot(
            pos[0] - target[0], pos[1] - target[1]
        )

    def _waypoint_timeout_s(self, waypoint_mode):
        if waypoint_mode == "precision":
            return arrival_timeout_max
        return self.navigation_profile.cruise_timeout_s(
            self._active_segment_distance_m
        )

    def _log_waypoint_event(self, reason, target_index, pos, target, distance):
        if not self._log_file:
            return
        try:
            with self._log_lock:
                self._log_file.write(json.dumps({
                    "event": "waypoint_advance",
                    "t": round(time.time(), 3),
                    "reason": reason,
                    "target_idx": target_index,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "arrival_distance_m": round(distance, 4),
                    "nav_profile": self.navigation_profile.profile,
                    "waypoint_mode": self._waypoint_mode(target_index),
                    "navigation_purpose": self._navigation_purpose,
                }) + "\n")
                self._log_file.flush()
        except Exception:
            pass

    def _do_yaw_test_burst(self):
        """非闭环yaw方向验证：直接发固定vyaw一小段时间后归零，不经过yaw_pid。
        阻塞调用线程(navigate()所在的主循环线程)，但发送线程独立运行在另一个
        线程，se_fc当前值会持续以100Hz/50Hz发出，不受阻塞影响，是安全的。"""
        if not (self.t265_ok and self.realsense):
            logger.warning("[YAW测试] T265不可用，跳过")
            return
        yaw0 = math.degrees(self.realsense.get_orientation()[2])
        logger.warning(
            f"[YAW测试] 脉冲前yaw={yaw0:.1f}° 发送固定vyaw={YAW_TEST_BURST_VALUE}"
            f"持续{YAW_TEST_BURST_DURATION_S:.1f}秒(非闭环，不经过yaw_pid)"
        )
        t_start = time.time()
        with lock:
            self.se_fc[6] = YAW_TEST_BURST_VALUE + sp_side
        while time.time() - t_start < YAW_TEST_BURST_DURATION_S:
            time.sleep(0.05)
        with lock:
            self.se_fc[6] = 0 + sp_side
        yaw1 = math.degrees(self.realsense.get_orientation()[2])
        logger.warning(
            f"[YAW测试] 脉冲后yaw={yaw1:.1f}° 变化={yaw1 - yaw0:+.1f}°"
            f"（vyaw为负，若变化为负=方向符合固件'逆时针为正'约定；若变化为正=方向相反，疑似正反馈根因）"
        )

    def _log_land_event(self, event, **fields):
        if not self._log_file:
            return
        try:
            with self._log_lock:
                self._log_file.write(json.dumps({
                    "t": round(time.time(), 3),
                    "event": event,
                    "state": "LAND",
                    **fields,
                }) + "\n")
                self._log_file.flush()
        except Exception:
            pass

    # ================= 降落 =================
    def land(self):
        logger.info("降落")
        land_started_monotonic = time.monotonic()
        self._log_land_event(
            "land_start",
            navigation_purpose=self._navigation_purpose,
            target_idx=self.target_index,
            target=(
                list(self.targets[self.target_index])
                if self.target_index < len(self.targets)
                else None
            ),
            ramp_z_cm=round(self._ramp_z_cm, 1),
            task_command=0,
        )
        with lock:
            self.se_fc[2] = 0

        # Keep the T265 heading outer loop active until the flight controller
        # confirms that the motors are locked.  A zero yaw-rate command is not
        # an angle hold: during descent, thrust asymmetry or ground effect can
        # still rotate the aircraft.  Disarm only after the landing loop exits.

        # 不能一触发就关串口退出：凌霄IMU定点悬停依赖Pi持续喂T265速度参考(CMD 0x33)，
        # 串口一关这个参考直接断流，而OneKey_Land()的物理下降通常要持续数秒。
        # 这里继续跑主循环(保持串口/T265速度帧不断)，轮询真实解锁状态(unlock_sta)，
        # 确认真的上锁了(或超时兜底)才真正进入END关闭退出。
        #
        # 2026-07-08修复：这个循环原本只轮询unlock_sta，没有主动清零速度指令——
        # se_fc[3]/[4]/[6]会停留在navigate()最后一次set_speed()的值上，被发送线程原样
        # 重复发送，且没有PID再持续修正，真机测试观察到一键降落无响应时飞机会明显
        # 偏离原位置。这里持续调用set_speed(0,0,0,ramp)清零水平速度、保持高度，
        # 避免过期指令导致失控漂移。
        # 2026-07-08修复：land()原本从触发到确认/超时全程不写任何飞行日志，导致
        # 降落物理下降过程完全没有位置数据(真机测试想验证"降落时有没有额外偏移"
        # 但发现日志是空的)。这里跟takeoff()一样，自己在循环里直接采样T265(不依赖
        # loop()调用时传入的旧值，那个值在整个等待期间不会更新)。
        t_start = time.time()
        unlock_confirm_count = 0
        gaveup_logged = False
        while True:
            yaw_cmd = 0
            if self.t265_ok and self.realsense:
                try:
                    land_pos = list(self.realsense.get_position())
                    land_yaw = self.realsense.get_orientation()[2]
                    land_tv = self.realsense.get_velocity()
                    land_raw_imu = list(self.realsense.get_raw_imu())
                except Exception:
                    land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)
                    land_raw_imu = [0.0] * 6
                else:
                    # Older test doubles and alternate T265 wrappers may not
                    # expose confidence; preserve the landing telemetry while
                    # treating that case as usable for the bounded controller.
                    confidence_reader = getattr(
                        self.realsense, "get_tracking_confidence", None
                    )
                    confidence = confidence_reader() if confidence_reader else 3
                    try:
                        self._heading_status = self._update_heading_hold(
                            land_yaw, confidence
                        )
                        yaw_cmd = self._heading_status.command_dps
                    except Exception as exc:
                        logger.warning(f"降落阶段航向保持更新失败，当前tick输出0: {exc}")
            else:
                land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)
                land_raw_imu = [0.0] * 6

            # Keep XY stopped and the descent setpoint unchanged, while
            # applying only the bounded heading correction from T265.
            self.set_speed(0, 0, yaw_cmd, int(self._ramp_z_cm))

            # 激光高度覆盖Z：跟 loop()/takeoff() 一样，T265自身Z轴未标定不是真实高度，
            # 这里如果继续用原始T265 Z，降落阶段记录的"高度"会是假数据，没法验证物理降落过程。
            with lock:
                laser_h = self.serial_fc_ref._last_laser_height_cm if self.serial_fc_ref else 0.0
            laser_height_is_valid = laser_height_valid(laser_h)
            if laser_height_is_valid:
                land_pos[2] = laser_h

            with lock:
                unlock_sta = self.re_fc[5] if len(self.re_fc) > 5 else 0

            # 电机PWM非零位掩码(帧2新增字段)：诊断unlock_sta是否假阳性
            # (问题7 2026-07-08：unlock_sta读到0但用户确认电机实际未停转)
            motor_pwm_mask = None
            motor_pwm_mask_t = None
            if self.serial_fc_ref is not None:
                with lock:
                    motor_pwm_mask = self.serial_fc_ref.debug_data.get("motor_pwm_mask")
                    motor_pwm_mask_t = self.serial_fc_ref.debug_data.get("motor_pwm_mask_t")

            # 2026-07-12新增：固件纯超时兜底(10秒)判定高度仍偏高时会放弃自动锁桨，
            # 转为永久等待人工接管(问题7/9严重安全隐患修复)。land()要能感知这个状态，
            # 否则Python自己的LAND_CONFIRM_TIMEOUT_S超时会先关串口退出，切断固件
            # 悬停所需的T265速度参考，跟固件"等人工介入"的设计意图冲突。
            land_timeout_gaveup = None
            if self.serial_fc_ref is not None:
                with lock:
                    land_timeout_gaveup = self.serial_fc_ref.debug_data.get("land_timeout_gaveup")
            if land_timeout_gaveup and not gaveup_logged:
                logger.warning("降落纯超时兜底判定高度仍偏高，已放弃自动锁桨，需要人工介入")
                self._log_land_event(
                    "land_wait_manual",
                    reason="firmware_timeout_gaveup",
                    land_elapsed_s=round(time.monotonic() - land_started_monotonic, 3),
                    laser_height_m=laser_h if laser_height_is_valid else None,
                    unlock_sta=unlock_sta,
                    motor_pwm_mask=motor_pwm_mask,
                )
                gaveup_logged = True

            motor_pwm_age_s = None
            if motor_pwm_mask_t is not None:
                age = time.time() - motor_pwm_mask_t
                if age >= 0:
                    motor_pwm_age_s = round(age, 3)
            motor_pwm_ok = motor_pwm_mask is None or motor_pwm_mask == 0
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(now, 3),
                            "state": self.state,
                            "pos": [round(land_pos[0], 4), round(land_pos[1], 4), round(land_pos[2], 4)],
                            "t265_yaw_deg": round(math.degrees(land_yaw), 2),
                            "t265_vel": [round(land_tv[0], 4), round(land_tv[1], 4)],
                            "raw_imu": [round(v, 4) for v in land_raw_imu],
                            "unlock_sta": unlock_sta,
                            "motor_pwm_mask": motor_pwm_mask,
                            "motor_pwm_mask_t": motor_pwm_mask_t,
                            "motor_pwm_age_s": motor_pwm_age_s,
                            "motor_pwm_ok": motor_pwm_ok,
                            "unlock_confirm_count": unlock_confirm_count,
                            "land_elapsed_s": round(
                                time.monotonic() - land_started_monotonic, 3
                            ),
                            "laser_height_m": laser_h if laser_height_is_valid else None,
                            "laser_height_valid": laser_height_is_valid,
                            "land_timeout_gaveup": land_timeout_gaveup,
                            "task_command": self.se_fc[2],
                            "z_command_cm": self.se_fc[5],
                            "yaw_cmd_sent": yaw_cmd,
                            **self._heading_log_fields(),
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now

            # 2026-07-10修复：只看unlock_sta的去抖仍会假阳性(问题7)——矩形路径基线测试
            # (basic_radar)复现了unlock_sta连续读到0、去抖满足，但motor_pwm_mask全程
            # 非零(电机仍在出PWM)的矛盾场景，用户确认那次是人工接管才降落的。这里要求
            # unlock_sta==0同时motor_pwm_mask==0才计入确认；motor_pwm_mask为None(诊断
            # 数据不可用)时不阻塞，退化成只看unlock_sta。
            motor_pwm_ok = motor_pwm_mask is None or motor_pwm_mask == 0
            if unlock_sta == 0 and motor_pwm_ok:
                unlock_confirm_count += 1
                if unlock_confirm_count >= LAND_UNLOCK_CONFIRM_COUNT:
                    logger.info("降落确认：已上锁")
                    self._log_land_event(
                        "land_exit",
                        reason="confirmed",
                        land_elapsed_s=round(
                            time.monotonic() - land_started_monotonic, 3
                        ),
                        pos=[round(land_pos[0], 4), round(land_pos[1], 4), round(land_pos[2], 4)],
                        raw_imu=[round(v, 4) for v in land_raw_imu],
                        heading_hold_enabled=self.heading_hold.config.enabled,
                        heading_hold_armed=self.heading_hold.armed,
                        yaw_cmd_sent=yaw_cmd,
                        motor_pwm_mask_t=motor_pwm_mask_t,
                        laser_height_m=laser_h if laser_height_is_valid else None,
                        unlock_sta=unlock_sta,
                        motor_pwm_mask=motor_pwm_mask,
                        unlock_confirm_count=unlock_confirm_count,
                    )
                    break
            else:
                unlock_confirm_count = 0
            if not gaveup_logged and time.time() - t_start >= LAND_CONFIRM_TIMEOUT_S:
                logger.warning("降落确认超时，强制退出")
                self._log_land_event(
                    "land_exit",
                    reason="python_timeout",
                    land_elapsed_s=round(
                        time.monotonic() - land_started_monotonic, 3
                    ),
                    laser_height_m=laser_h if laser_height_is_valid else None,
                    unlock_sta=unlock_sta,
                    motor_pwm_mask=motor_pwm_mask,
                    unlock_confirm_count=unlock_confirm_count,
                )
                break
            time.sleep(0.03)

        self.heading_hold.disarm("land")
        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")
        # 上锁指令必须最先发出、前面不能有任何阻塞调用——stop_all()由emergency_stop
        # 路径(飞控串口超时/T265丢失)触发，se_fc是独立50Hz发送线程读取的共享状态，
        # 越早写入，飞控收到断电/上锁指令就越早。见docs/known_issues.md #22
        # (电机停转可靠性是全项目最高优先级安全隐患)。
        with lock:
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[6] = sp_side
            self.se_fc[7] = 101
        self.heading_hold.disarm("stop_all")
        self._resource_monitor.stop()
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
        if self.realsense:
            self.realsense.stop()
        self.task_running = False

    # ================= 控制接口 =================
    def set_speed(self, x, y, yaw, z):
        with lock:
            self.se_fc[3] = x + sp_side
            self.se_fc[4] = y + sp_side
            self.se_fc[5] = z
            self.se_fc[6] = yaw + sp_side

    def _read_fc_heading(self, *, preunlock=False):
        """Read one atomic FC-yaw snapshot and enforce freshness/jump bounds."""
        with lock:
            raw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else float("nan")
            received_at = fc_last_rx_monotonic.value
            frame_counter = int(fc_frame_counter.value)

        now = time.monotonic()
        age_s = now - received_at if received_at > 0 else float("inf")
        self._heading_feedback_age_s = age_s if math.isfinite(age_s) else None

        if not math.isfinite(raw_deg):
            self._heading_feedback_failure_reason = "fc_heading_non_finite"
            raise HeadingFeedbackError("fc_heading_non_finite")
        current_deg = wrap_degrees(raw_deg)

        if preunlock and frame_counter < FC_HEADING_MIN_PREUNLOCK_FRAMES:
            self._heading_feedback_failure_reason = (
                f"fc_heading_not_ready_{frame_counter}_frames"
            )
            raise HeadingFeedbackError(self._heading_feedback_failure_reason)
        if received_at <= 0:
            self._heading_feedback_failure_reason = "fc_heading_not_ready"
            raise HeadingFeedbackError("fc_heading_not_ready")

        if age_s > FC_HEADING_MAX_AGE_S:
            if preunlock or self._fc_heading_last_deg is None:
                self._heading_feedback_failure_reason = (
                    f"fc_heading_stale_{age_s:.3f}s"
                )
                raise HeadingFeedbackError(self._heading_feedback_failure_reason)
            self._fc_heading_invalid_ticks += 1
            self._heading_feedback_failure_reason = (
                f"fc_heading_stale_{age_s:.3f}s"
            )
            if self._fc_heading_invalid_ticks >= FC_HEADING_INVALID_TICKS:
                raise HeadingFeedbackError(self._heading_feedback_failure_reason)
            return self._fc_heading_last_deg, False

        if self._fc_heading_last_frame != frame_counter:
            if self._fc_heading_last_deg is not None:
                step_deg = wrap_degrees(current_deg - self._fc_heading_last_deg)
                if abs(step_deg) > FC_HEADING_MAX_STEP_DEG:
                    self._heading_feedback_failure_reason = (
                        f"fc_heading_jump_{step_deg:+.2f}deg"
                    )
                    raise HeadingFeedbackError(self._heading_feedback_failure_reason)
            self._fc_heading_last_frame = frame_counter
            self._fc_heading_last_deg = current_deg

        self._fc_heading_invalid_ticks = 0
        self._heading_feedback_failure_reason = None
        return current_deg, True

    def _update_heading_diagnostics(self, t265_yaw, fc_yaw_deg=None):
        t265_deg = wrap_degrees(math.degrees(t265_yaw))
        if self._heading_t265_reference_deg is not None:
            self._heading_t265_delta_deg = wrap_degrees(
                t265_deg - self._heading_t265_reference_deg
            )
        if fc_yaw_deg is not None and self._heading_fc_reference_deg is not None:
            self._heading_fc_delta_deg = wrap_degrees(
                fc_yaw_deg - self._heading_fc_reference_deg
            )
        if (
            self._heading_t265_delta_deg is not None
            and self._heading_fc_delta_deg is not None
        ):
            self._heading_source_disagreement_deg = wrap_degrees(
                self._heading_t265_delta_deg - self._heading_fc_delta_deg
            )

    def _select_heading_feedback(self, t265_yaw, t265_confidence, *, preunlock=False):
        self._last_t265_confidence = int(t265_confidence)
        if self.heading_source == "t265":
            self._update_heading_diagnostics(t265_yaw)
            return t265_yaw, int(t265_confidence)

        fc_yaw_deg, fresh = self._read_fc_heading(preunlock=preunlock)
        self._update_heading_diagnostics(t265_yaw, fc_yaw_deg)
        return math.radians(fc_yaw_deg), 3 if fresh else 0

    def _arm_heading_hold(self, t265_yaw):
        t265_deg = wrap_degrees(math.degrees(t265_yaw))
        if self.heading_source == "fc":
            selected_yaw, _ = self._select_heading_feedback(
                t265_yaw, T265_CONFIDENCE_MIN, preunlock=True
            )
            self._heading_fc_reference_deg = wrap_degrees(math.degrees(selected_yaw))
        else:
            selected_yaw = t265_yaw
            self._heading_feedback_age_s = None
        self._heading_t265_reference_deg = t265_deg
        self._update_heading_diagnostics(
            t265_yaw,
            self._heading_fc_reference_deg if self.heading_source == "fc" else None,
        )
        return self.heading_hold.arm(selected_yaw, time.time())

    def _heading_recovery_status(self, t265_yaw):
        selected_yaw, heading_confidence = self._select_heading_feedback(
            t265_yaw, self._last_t265_confidence or T265_CONFIDENCE_MIN
        )
        if self.heading_source == "fc" and heading_confidence < 2:
            raise HeadingFeedbackError(
                self._heading_feedback_failure_reason or "fc_heading_stale"
            )
        return self._apply_heading_command_polarity(
            self.heading_hold.recovery_status(
                selected_yaw, max_rate_dps=HEADING_RECOVERY_MAX_DPS
            )
        )

    def _apply_heading_command_polarity(self, status):
        """Map the selected feedback axis onto the flight-command axis."""
        if status.command_dps:
            return replace(
                status,
                command_dps=(
                    HEADING_COMMAND_SIGN_BY_SOURCE[self.heading_source]
                    * status.command_dps
                ),
            )
        return status

    def _clear_heading_fault(self, t265_yaw):
        selected_yaw, heading_confidence = self._select_heading_feedback(
            t265_yaw, self._last_t265_confidence or T265_CONFIDENCE_MIN
        )
        if self.heading_source == "fc" and heading_confidence < 2:
            raise HeadingFeedbackError(
                self._heading_feedback_failure_reason or "fc_heading_stale"
            )
        return self.heading_hold.clear_fault_preserving_target(selected_yaw)

    def _update_heading_hold(self, yaw, confidence, *, allow_fault_relock=True):
        selected_yaw, heading_confidence = self._select_heading_feedback(
            yaw, confidence
        )
        if self._heading_bypass_active:
            current_deg = wrap_degrees(math.degrees(selected_yaw))
            target_deg = self.heading_hold.target_deg
            error_deg = (
                wrap_degrees(target_deg - current_deg)
                if target_deg is not None
                else None
            )
            return replace(
                self._heading_status,
                command_dps=0,
                armed=False,
                current_deg=current_deg,
                error_deg=error_deg,
                degraded_reason="heading_fault_bypassed",
                fault_reason=None,
            )
        status = self.heading_hold.update(
            selected_yaw,
            heading_confidence,
            time.time(),
            allow_fault_relock=allow_fault_relock,
        )
        if abs(status.command_dps) > HEADING_NORMAL_MAX_DPS:
            status = replace(
                status,
                command_dps=(
                    HEADING_NORMAL_MAX_DPS
                    if status.command_dps > 0
                    else -HEADING_NORMAL_MAX_DPS
                ),
            )
        status = self._apply_heading_command_polarity(status)
        if status.fault_reason and status.fault_reason != self._last_heading_fault_logged:
            logger.error(f"航向保持触发保护: {status.fault_reason}")
            self._last_heading_fault_logged = status.fault_reason
        return status

    def _heading_log_fields(self):
        status = self._heading_status
        return {
            "heading_hold_enabled": status.enabled,
            "heading_hold_armed": status.armed,
            "heading_source": self.heading_source,
            "heading_feedback_age_s": (
                round(self._heading_feedback_age_s, 3)
                if self._heading_feedback_age_s is not None
                else None
            ),
            "heading_feedback_failure_reason": self._heading_feedback_failure_reason,
            "heading_t265_delta_deg": (
                round(self._heading_t265_delta_deg, 2)
                if self._heading_t265_delta_deg is not None
                else None
            ),
            "heading_fc_delta_deg": (
                round(self._heading_fc_delta_deg, 2)
                if self._heading_fc_delta_deg is not None
                else None
            ),
            "heading_source_disagreement_deg": (
                round(self._heading_source_disagreement_deg, 2)
                if self._heading_source_disagreement_deg is not None
                else None
            ),
            "t265_tracking_confidence": self._last_t265_confidence,
            "heading_target_deg": (
                round(status.target_deg, 2) if status.target_deg is not None else None
            ),
            "heading_current_deg": (
                round(status.current_deg, 2) if status.current_deg is not None else None
            ),
            "heading_error_deg": (
                round(status.error_deg, 2) if status.error_deg is not None else None
            ),
            "heading_degraded_reason": status.degraded_reason,
            "heading_fault_reason": status.fault_reason,
            "heading_recovery_active": self._heading_recovery_active,
            "heading_recovery_elapsed_s": (
                round(time.time() - self._heading_recovery_started_at, 3)
                if self._heading_recovery_active
                and self._heading_recovery_started_at is not None
                else None
            ),
            "heading_recovery_hold_z_cm": self._heading_recovery_hold_z_cm,
            "heading_recovery_successes": self._heading_recovery_successes,
            "heading_recovery_failure_reason": self._heading_recovery_failure_reason,
            "heading_bypass_active": self._heading_bypass_active,
        }

    def _format_heading_error(self):
        error_deg = self._heading_status.error_deg
        return "--" if error_deg is None else f"{error_deg:+.1f}"

    # ================= 工具 =================
    def limit(self, v, max_v=0.3):
        return max(min(v, max_v), -max_v)

    def _step_ramp_z(self, target_z_cm: float):
        if self._ramp_z_cm < target_z_cm - RAMP_STEP:
            self._ramp_z_cm += RAMP_STEP
        elif self._ramp_z_cm > target_z_cm + RAMP_STEP:
            self._ramp_z_cm -= RAMP_STEP
        else:
            self._ramp_z_cm = target_z_cm

    # ================= 急停 =================
    def emergency(self):
        logger.warning("紧急停止触发！")
        with lock:
            self.se_fc[6] = sp_side
        self.heading_hold.disarm("emergency")
        self.emergency_stop = True
