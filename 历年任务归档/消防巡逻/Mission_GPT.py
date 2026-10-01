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
from typing import List, Optional
from Lcode.heading_hold import HeadingHoldConfig, HeadingHoldController
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock, fc_last_rx_time
from Lcode.resource_monitor import ResourceMonitor
from t265 import t265_class

# ---------- 常量 ----------
DRY_RUN = os.getenv("DRONE_DRY_RUN", "0") == "1"  # 桌面测试: 不解锁飞控，电机不会转
put_height = 100
VEL_SCALE = 0.7
posthreshold_xy = 0.15
posthreshold_z = 0.20
# 2026-07-16用户反馈：巡航航点(弓字形覆盖巡逻的中间点)不需要像最终降落准备航点那样
# 精确停留，只在最后两个航点(回原点+分段降到0.2m，
# 为紧接着的land()做准备)保留严格精度。CRUISE_*给巡航航点用，非巡航航点仍用
# 原有posthreshold_xy/z(经confidence分级)的严格判据。
# 2026-07-18日志回放确认：旧的0.4m分轴方形阈值会让0.8m换道航段在还差
# 约0.38~0.39m时就切换，下一条4m直线因此从两条规划线中间开始。提速应来自
# "不停留/不等速度归零"，不是来自未到航点就提前切换。历史三次完整路线
# 的27个中间航点均能连续进入15cm圆形半径至少0.3s。
CRUISE_WAYPOINT_RADIUS_M = 0.15
CRUISE_ARRIVAL_CONFIRM_CYCLES = 3  # 30ms控制周期下约90ms，只过滤单帧毛刺，不停留
PRECISION_TAIL_WAYPOINTS = 2  # 航点列表最后N个视为"精确降落准备"航点，不放宽
PRECISION_HEAD_WAYPOINTS = 1  # 2026-07-17新增：航点列表最前N个也视为精确航点，不能掠过。
                                # 起飞后第一个航点(原点,目标高度)本意是"先原地爬升到巡航
                                # 高度，再开始水平移动"，但"掠过式"逻辑只看xy不看z——起点
                                # 本来xy就已经达标(还没开始移动)，会在高度还只有起飞离地
                                # 高度(TAKEOFF_LIFTOFF_CM)时就被判定"到达"并立刻推进到下一个
                                # 航点，导致爬升和水平移动同时发生，而不是先爬升再移动。
                                # 2026-07-17真机测试实测到这个现象(边爬升边往-y飞)。
arrival_confirm_need = 15
arrival_hold_s = 1.5   # 到达判定满足后，在原地强制停留观察的时长（阶跃响应测试用）
# 2026-07-16真机测试实测：basic继承来的"5.0+arrival_hold_s"=6.5s是按basic那边
# 几十厘米量级的小方框测试航线定的，fire_patrol的6x5格心网格航点间距最大到
# 4.0m(单条车道全长)。真机数据显示恢复PATROL续飞后10个航点全部精确卡在6.5秒
# 超时强制跳过，实际每段只飞了1~2.5米就被打断(数据换算有效前进速度约
# 0.28~0.40m/s，6.5秒完全不够走完最长4.0m的航段)。改成按最长航段距离
# (4.0m ÷ 保守速度0.25m/s ≈ 16s) + 到达确认/PID收敛余量，取25s。
arrival_timeout_max = 25.0
T265_CONFIDENCE_MIN = 2       # 定点所需最低追踪置信度 (0=失败,1=低,2=中,3=高)
T265_CONFIDENCE_WAIT_S = 8.0  # 等待置信度达标的超时时间
FLIGHT_LOG_INTERVAL = 0.05
RAMP_STEP = 1.5
TAKEOFF_CONFIRM_NEED = 10
TAKEOFF_TIMEOUT_S = 15.0
TAKEOFF_LIFTOFF_CM = 35.0  # 一键起飞只负责盲飞离地这一小段，其余交给navigate()的x/y PID+高度ramp爬升到真正目标高度
                            # 不能设太低：2026-07-06实测15cm时T265/激光近地面定位质量下降，起飞confirm超时+机体水平旋转
LAND_CONFIRM_TIMEOUT_S = 25.0  # 降落触发后最多等待多久确认unlock_sta==0(已上锁)，超时也强制退出，避免卡死
                                # 2026-07-09从10.0改为25.0：1.0m高度门槛测试发现10秒内unlock_sta/motor_pwm_mask
                                # 全程未变但用户确认物理已自动降落成功，怀疑是超时定太短、真实上锁发生在
                                # 断开串口之后——调长验证这个假设，同时避免后续测试的"超时"结果继续有歧义
LAND_UNLOCK_CONFIRM_COUNT = 5  # 降落确认去抖：要连续读到N次unlock_sta==0才真正确认已上锁，不是单次就退出。
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

# ---------- fire_patrol 新增常量 ----------
FIRE_VISION_STALE_S = 0.5       # 火情检测结果超过此值未更新，视为摄像头/线程故障，不阻断飞行
FIRE_DETECT_MIN_HEIGHT_M = 1.0  # 低于此高度不触发火情检测。2026-07-17真机测试发现：
                                  # 起飞/降落阶段下视画面里出现的小红点，是无人机自己
                                  # 下视激光测距的反射光点，不是真实火源/干扰物——
                                  # 之前误以为是"灯罩"本身。近地时这个激光点在HSV/形状
                                  # 上跟真实火源难以区分，加高度门槛避免起飞/降落阶段
                                  # 被自己的激光反光误触发。
PATROL_MISS_SNAPSHOT_INTERVAL_S = 5.0  # 2026-07-16新增：PATROL态检测不到火源时，
                                          # 每隔这么久也存一张画面(reason="patrol_no_detect")，
                                          # 用来跟触发时存的清晰图做对比，验证"运动模糊导致
                                          # 巡航中检测不到"这个假设——之前只有触发时才存图，
                                          # 没有"检测失败时长什么样"的对照样本
APPROACH_DEADBAND_PX = 20       # 像素偏移小于此值不修正，防止中心附近来回抖
APPROACH_GAIN = 0.15            # APPROACH独立的小增益，明显小于navigate()跨格移动用的PID增益
APPROACH_MAX_STEP_CMPS = 8      # APPROACH阶段单次修正的速度上限(cm/s)，远小于navigate()的40，避免大幅晃动
APPROACH_TIMEOUT_S = 10.0       # 视觉伺服对准超时兜底，超时不强求完全居中，直接进入CONFIRM_WARN
APPROACH_CENTERED_DIST_M = 0.3  # 像素偏移换算的水平距离小于此值才算"对准"，对应赛题
                                  # 发挥部分(1)"接近火源水平距离<=5dm"，留量到3dm量级
HOVER_DROP_ALTITUDE_CM = 10.0 * 10  # 悬停抛投高度=10dm=100cm
HOVER_DROP_DURATION_S = 3.0     # 赛题写死的固定悬停时长，与arrival_hold_s(navigate()到达确认用)
                                  # 完全独立，不能混用——见设计文档"HOVER_DROP"一节
HOVER_HOLD_MAX_STEP_CMPS = APPROACH_MAX_STEP_CMPS  # HOVER_DROP水平位置闭环锁定的输出限幅，
                                  # 复用APPROACH的保守值(8cm/s)而非navigate()跨格移动的40——
                                  # 抛投前悬停应该只做小幅漂移修正，不该有大动作
PIXEL_TO_METER_AT_CRUISE = 0.0015  # 像素偏移->水平距离粗略换算系数，现场需按实际摄像头
                                     # FOV/高度标定，这里给占位初始值
TAKEOFF_WARN_LED_DURATION_S = 2.0  # 起飞前警示灯常亮时长(秒)，提醒周围人员即将解锁/起飞

# 2026-07-09新增：yaw方向验证专用。该测试已确认凌霄IMU执行方向与T265测量
# 方向一致；2026-07-18进一步定位到旧navigate()把PID输出额外取负，才是正反馈
# 的直接代码原因。脉冲工具保留作诊断，但与正式HeadingHoldController互斥。
# 默认关闭，不会影响正常飞行；显式设DRONE_YAW_TEST_BURST=1才会触发。
YAW_TEST_BURST_ENABLED = os.getenv("DRONE_YAW_TEST_BURST", "0") == "1"
YAW_TEST_BURST_VALUE = int(os.getenv("DRONE_YAW_TEST_BURST_VALUE", "-8"))
YAW_TEST_BURST_DURATION_S = float(os.getenv("DRONE_YAW_TEST_BURST_DURATION_S", "1.5"))


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
                 serial_fc_ref=None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.serial_fc_ref = serial_fc_ref

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False
        self.realsense = realsense_obj

        # PID
        self.x_pid = PID(0, 0)
        self.y_pid = PID(0, 0)
        self.heading_hold = HeadingHoldController(HeadingHoldConfig.from_env())
        if self.heading_hold.config.enabled and YAW_TEST_BURST_ENABLED:
            raise ValueError("DRONE_HEADING_HOLD 与 DRONE_YAW_TEST_BURST 不能同时启用")
        self._heading_status = self.heading_hold.update(0.0, confidence=0, now=time.time())
        self._last_heading_fault_logged = None

        # 航点
        self.targets = self.load_waypoints()
        self.target_index = 0
        self.emergency_stop = False

        # fire_patrol: 火情检测/接近/悬停抛投状态
        self.nav_mode = "PATROL"  # PATROL / APPROACH / CONFIRM_WARN / HOVER_DROP
        self.fire_triggered = False  # 全程只响应一次火情检测（见设计文档）
        self.saved_target_index_before_fire = None
        self._approach_start_time = None
        self.fire_vision = None  # main.py 注入 FireVision 实例
        self.serial_ground = None  # main.py 注入 Serial_ground 实例
        self._hover_drop_start_time = None
        self._hover_hold_pos = None  # HOVER_DROP闭环锁定的水平目标点，见_do_hover_drop()
        self._recover_hold_pos = None  # RECOVER_HEIGHT闭环锁定的水平目标点，见_do_recover_height()
        self._recover_target_z_cm = None  # RECOVER_HEIGHT要爬升回的目标高度(cm)

        # 到达判断
        self._arrival_window = deque(maxlen=arrival_confirm_need)
        self.arrival_start_time = 0.0
        self.arrival_confirmed_time: Optional[float] = None
        self.last_target_index = -1
        self._vel_window = deque(maxlen=ARRIVAL_VEL_WINDOW)
        self._cruise_arrival_count = 0

        # 高度 ramp
        self._ramp_z_cm = 0.0

        # 飞行数据日志
        self._log_file = None
        self._last_log_time = 0.0
        self._last_detect_log_time = 0.0  # 独立节流时间戳，避免跟主日志块共用_last_log_time
                                            # 导致两个日志块互相抢节流窗口、采样率减半
        self._last_miss_snapshot_time = 0.0  # PATROL"检测不到"存图节流时间戳
        self._log_lock = threading.Lock()
        self._resource_monitor = ResourceMonitor()

        # 旧yaw方向开环脉冲测试(问题16)，与正式航向保持互斥
        self._yaw_burst_done = False

    def load_waypoints(self):
        try:
            with open('router.txt', 'r', encoding="utf-8") as f:
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
            logger.warning("router.txt 不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取 router.txt 失败: {e}，使用默认航点")

        default = [[0.0, 0.0, put_height/100],
                   [0.5, 0.0, put_height/100],
                   [0.5, 0.5, put_height/100],
                   [0.0, 0.5, put_height/100]]
        return default

    # ================= 启动 =================
    def start(self):
        self.heading_hold.reset_for_new_mission()
        self._last_heading_fault_logged = None
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
                logger.error(f"T265 置信度 {T265_CONFIDENCE_WAIT_S:.0f} 秒内仍偏低(confidence={confidence})，定点可能不稳定")
                confirm = input(f"T265置信度过低(confidence={confidence})，输入 YES 强制起飞，其他任意键取消任务: ")
                if confirm.strip() != "YES":
                    logger.error("任务已取消（T265置信度未确认）")
                    return
                logger.warning("已人工确认，强制以低置信度T265数据起飞")
            else:
                logger.info(f"T265 追踪置信度已稳定 (confidence={confidence})")
        else:
            logger.error("T265 FAILED — 无水平位置反馈，仅高度模式起飞有失控风险")
            confirm = input("T265 未连接，输入 YES 强制以仅高度模式起飞，其他任意键取消任务: ")
            if confirm.strip() != "YES":
                logger.error("任务已取消（T265 未确认）")
                return
            logger.warning("已人工确认，强制以仅高度模式起飞")

        logger.info(f"任务启动, {len(self.targets)} 个航点")

        self.task_running = True
        self.state = "TAKEOFF"

        try:
            path = os.path.dirname(os.path.realpath(sys.argv[0]))
            self._log_file = open(path + "/flight_data.jsonl", "a")
            with self._log_lock:
                self._log_file.write(json.dumps({"event": "task_start"}) + "\n")
                self._log_file.flush()
        except Exception:
            pass

        self._resource_monitor.start(self._log_file, self._log_lock)

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
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
            if self.state == "TAKEOFF":
                self.takeoff()
            elif self.state == "NAVIGATE":
                self.navigate(pos, yaw)
            elif self.state == "LAND":
                self.land()
            elif self.state == "END":
                self.stop_all()

            time.sleep(0.03)

    # ================= 起飞 =================
    def _blink_warning_led(self):
        """起飞前红灯常亮TAKEOFF_WARN_LED_DURATION_S秒提醒周围人员，阻塞调用
        (起飞前的安全等待本来就该是阻塞的，给人反应时间)。GPIO不可用时静默
        跳过，不阻断起飞流程。"""
        try:
            from Lcode.gpio_led import set_rgb_led
        except Exception as e:
            logger.error(f"起飞警示灯点亮失败: {e}")
            return
        set_rgb_led('R')
        time.sleep(TAKEOFF_WARN_LED_DURATION_S)
        set_rgb_led('OFF')

    def takeoff(self):
        if DRY_RUN:
            logger.warning("takeoff: DRY_RUN 模式，不发送解锁指令，电机不会转")
        else:
            logger.info("takeoff: started")

        # 起飞前红灯常亮示警，必须在task_sta(解锁指令)写入se_fc之前完成
        self._blink_warning_led()

        # 航向目标必须在解锁前锁存为“起飞时机头方向”，不能固定追T265初始化时的0°。
        # T265不可用/置信度不足时不阻断基本飞行，航向环保持未arm并输出0°/s。
        if self.t265_ok and self.realsense:
            try:
                takeoff_confidence = self.realsense.get_tracking_confidence()
                takeoff_yaw = self.realsense.get_orientation()[2]
                if takeoff_confidence >= T265_CONFIDENCE_MIN:
                    self._heading_status = self.heading_hold.arm(takeoff_yaw, time.time())
                else:
                    logger.warning(f"航向保持未启用：起飞前T265置信度不足({takeoff_confidence})")
            except Exception as e:
                logger.warning(f"航向保持未启用：起飞前读取T265 yaw失败({e})")

        target_h_cm = TAKEOFF_LIFTOFF_CM  # 一键起飞只爬升到离地高度，真正目标高度交给 navigate() 闭环爬升

        with lock:
            self.se_fc[5] = int(target_h_cm)  # com_z：一键起飞目标高度，必须在 task_sta 触发前写入，
            self.se_fc[2] = 0 if DRY_RUN else 1  # 否则飞控读到的是 se_fc 初始默认值(120cm)而非本次航点高度

        confirm_count = 0
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start

            yaw = 0.0
            yaw_cmd = 0
            if self.t265_ok and self.realsense:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    confidence = self.realsense.get_tracking_confidence()
                    self._heading_status = self._update_heading_hold(yaw, confidence)
                    yaw_cmd = self._heading_status.command_dps
                    with lock:
                        self.se_fc[6] = yaw_cmd + sp_side
                except Exception as e:
                    logger.warning(f"起飞阶段航向保持读取失败，当前tick输出0: {e}")
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
                logger.warning("takeoff: 超时，强制切换")
                break

            time.sleep(0.03)

        self._ramp_z_cm = target_h_cm
        self.state = "NAVIGATE"

    # ================= 导航 =================
    def navigate(self, pos, yaw):
        confidence = self.realsense.get_tracking_confidence() if (self.t265_ok and self.realsense) else 0
        self._heading_status = self._update_heading_hold(yaw, confidence)
        yaw_cmd = self._heading_status.command_dps

        if self.nav_mode == "APPROACH":
            self._do_approach(pos, yaw_cmd)
            return
        if self.nav_mode == "CONFIRM_WARN":
            self._do_confirm_warn()
            return
        if self.nav_mode == "HOVER_DROP":
            self._do_hover_drop(pos, yaw_cmd)
            return
        if self.nav_mode == "RECOVER_HEIGHT":
            self._do_recover_height(pos, yaw_cmd)
            return

        # PATROL态：持续检查火情检测结果，触发APPROACH
        if self.nav_mode == "PATROL" and self.fire_vision is not None:
            latest = self.fire_vision.latest()
            now = time.time()
            # 2026-07-16补充日志：之前完全没记录PATROL态逐帧的检测结果，没法诊断
            # "火源明明在巡逻范围内为什么没检测到/为什么在别的时机才检测到"这类问题。
            # 节流写法跟navigate()其它日志块一致(FLIGHT_LOG_INTERVAL)。
            if self._log_file and now - self._last_detect_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(now, 3),
                            "state": "PATROL_DETECT",
                            "target_idx": self.target_index,
                            "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                            "dx_px": latest.get("dx_px"),
                            "dy_px": latest.get("dy_px"),
                            "detect_t": latest.get("t"),
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
                self._last_detect_log_time = now

            if latest.get("dx_px") is None:
                # 检测不到时定期存一张画面，跟触发时存的清晰图做对比，验证运动模糊假设
                if now - self._last_miss_snapshot_time >= PATROL_MISS_SNAPSHOT_INTERVAL_S:
                    try:
                        self.fire_vision.save_snapshot(reason="patrol_no_detect")
                    except Exception as e:
                        logger.error(f"patrol_no_detect存图失败: {e}")
                    self._last_miss_snapshot_time = now
            elif now - latest.get("t", 0) < FIRE_VISION_STALE_S and pos[2] >= FIRE_DETECT_MIN_HEIGHT_M:
                if self.maybe_trigger_approach((latest["dx_px"], latest["dy_px"])):
                    return

        if self.target_index >= len(self.targets):
            logger.info("全部航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]
        target_z = int(target[2] * 100)

        if confidence == 0 and self.t265_ok:
            logger.warning("T265 追踪丢失，悬停等待")
            self.set_speed(0, 0, yaw_cmd, int(self._ramp_z_cm))
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
                            **self._heading_log_fields(),
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
            return

        if self.t265_ok and self.realsense:
            self.x_pid.set_target(target[0])
            self.y_pid.set_target(target[1])
            vx = self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE
            vy = self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE
            vx = int(self.limit(vx, 40))
            vy = int(self.limit(vy, 40))
        else:
            vx, vy = 0, 0

        self._step_ramp_z(target_z)
        # HeadingHoldController已经按target-current生成正确符号，必须原样发送，
        # 禁止再像旧代码那样额外取负把负反馈变成正反馈。
        self.set_speed(vx, vy, yaw_cmd, int(self._ramp_z_cm))

        # T265 速度（到达检测的速度门槛 + 后面日志/终端输出共用，避免重复取值）
        if self.t265_ok and self.realsense:
            tv = self.realsense.get_velocity()
        else:
            tv = (0.0, 0.0, 0.0)

        # 到达检测
        if self.t265_ok and self.realsense:
            is_precision_waypoint = (
                self.target_index < PRECISION_HEAD_WAYPOINTS
                or self.target_index >= len(self.targets) - PRECISION_TAIL_WAYPOINTS
            )
            if is_precision_waypoint:
                xy_thresh = 0.10 if confidence >= 3 else (posthreshold_xy if confidence == 2 else 0.30)
                z_thresh = posthreshold_z
            dx = abs(pos[0] - target[0])
            dy = abs(pos[1] - target[1])
            dz = abs(pos[2] - target[2])
            # 速度用最近N帧均值而非瞬时值，平滑T265速度噪声尖峰(见 ARRIVAL_VEL_WINDOW 注释)
            self._vel_window.append((tv[0], tv[1]))
            avg_vx = sum(v[0] for v in self._vel_window) / len(self._vel_window)
            avg_vy = sum(v[1] for v in self._vel_window) / len(self._vel_window)
            speed = math.hypot(avg_vx, avg_vy)

            if self.target_index != self.last_target_index:
                self.last_target_index = self.target_index
                self._arrival_window.clear()
                self.arrival_confirmed_time = None
                self.arrival_start_time = time.time()
                self._cruise_arrival_count = 0

            if dx > 0.3:
                self.x_pid.reset()
            if dy > 0.3:
                self.y_pid.reset()

            if is_precision_waypoint:
                # 精确航点(最前PRECISION_HEAD_WAYPOINTS个，起飞后原地爬升到巡航高度；
                # 最后PRECISION_TAIL_WAYPOINTS个，为land()做准备)：维持原有滑动窗口
                # 确认+停留观察的严格流程，不能掠过。
                frame_ok = dx < xy_thresh and dy < xy_thresh and dz < z_thresh and speed < ARRIVAL_VEL_THRESH
                self._arrival_window.append(frame_ok)

                if arrival_window_confirmed(self._arrival_window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO):
                    if self.arrival_confirmed_time is None:
                        self.arrival_confirmed_time = time.time()
                        logger.info(f"到达航点 {self.target_index}，停留 {arrival_hold_s:.0f}s 观察")
                    elif time.time() - self.arrival_confirmed_time >= arrival_hold_s:
                        logger.info(f"航点 {self.target_index} 停留完成")
                        self._on_arrival(target)
                else:
                    self.arrival_confirmed_time = None
            else:
                # 2026-07-16用户反馈：巡航航点(弓字形中间点)不需要精确停留确认，
                # 改成"掠过式"——连续3个控制周期进入航点的15cm圆形半径就立刻切下一个
                # 目标点，不等速度归零、不停留观察；短确认只过滤单帧定位毛刺。不检查
                # z/速度门槛：高度由_step_ramp_z单独渐进逼近，
                # 拿它来卡住水平推进反而会让HOVER_DROP之后高度未恢复的已知问题
                # (见2026-07-16讨论)连带拖慢巡航，没有必要绑在一起。
                cruise_distance = math.hypot(dx, dy)
                if cruise_distance <= CRUISE_WAYPOINT_RADIUS_M:
                    self._cruise_arrival_count += 1
                else:
                    self._cruise_arrival_count = 0
                if self._cruise_arrival_count >= CRUISE_ARRIVAL_CONFIRM_CYCLES:
                    logger.info(f"航点 {self.target_index} 掠过(巡航航点，不停留)")
                    self._on_arrival(target)

            if time.time() - self.arrival_start_time >= arrival_timeout_max:
                logger.warning(f"航点 {self.target_index} 超时，强制跳过")
                self.target_index += 1

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
            f" | yawerr={self._format_heading_error():>5} cmd={yaw_cmd:+d}"
            f"{t265_str}",
            end="", flush=True
        )

    # ================= 到达处理 =================
    def _on_arrival(self, target):
        if YAW_TEST_BURST_ENABLED and self.target_index == 0 and not self._yaw_burst_done:
            self._yaw_burst_done = True
            self._do_yaw_test_burst()
        if self.target_index == len(self.targets) - 2:
            pass  # 到达倒数第二个航点 (原 rgb_led 逻辑已移除)
        self.target_index += 1
        # 2026-07-16修复：巡航"掠过式"逻辑在navigate()同一次调用里就地推进target_index，
        # 不像原来的"多帧确认+停留"那样要等到下一次tick顶部的
        # `if self.target_index != self.last_target_index`才重置arrival_start_time——
        # 如果不在这里立刻重置，紧接着本次调用末尾的超时检查会拿着旧的
        # arrival_start_time(可能已经过期)去判定刚刚推进到的新target_index，
        # 一次调用内连续误判超时、连跳两个航点。在这里直接同步重置，不留这个窗口。
        self.last_target_index = self.target_index
        self.arrival_start_time = time.time()
        self._arrival_window.clear()
        self.arrival_confirmed_time = None
        self._cruise_arrival_count = 0

    # ================= fire_patrol: 火情检测触发 =================
    def maybe_trigger_approach(self, detection):
        """detection为None或已经触发过(fire_triggered=True)时不触发。
        触发时保存当前target_index用于HOVER_DROP完成后续飞。"""
        if detection is None or self.fire_triggered:
            return False
        self.saved_target_index_before_fire = self.target_index
        self.fire_triggered = True
        self.nav_mode = "APPROACH"
        self._approach_start_time = time.time()
        logger.info(f"检测到火情，悬停进入APPROACH（保存航点索引{self.target_index}）")
        # 2026-07-16新增：触发时存一张现场画面，方便事后调试分析(比如误检测复盘、
        # 确认当时看到的到底是不是真火源)。存图失败不应该影响触发流程本身。
        if self.fire_vision is not None:
            try:
                self.fire_vision.save_snapshot(reason="fire_triggered")
            except Exception as e:
                logger.error(f"火情触发存图失败: {e}")
        return True

    def is_approach_centered(self, dx_px, dy_px):
        """像素偏移换算成水平距离，小于APPROACH_CENTERED_DIST_M才算对准正下方。"""
        dist_m = math.hypot(dx_px, dy_px) * PIXEL_TO_METER_AT_CRUISE
        return dist_m < APPROACH_CENTERED_DIST_M

    def approach_timed_out(self):
        if self._approach_start_time is None:
            return False
        return time.time() - self._approach_start_time >= APPROACH_TIMEOUT_S

    def finish_hover_drop_and_resume(self):
        """HOVER_DROP完成后不直接恢复PATROL，先进入RECOVER_HEIGHT原地爬升回
        巡航高度，见设计文档"续飞逻辑"。

        2026-07-17用户反馈：HOVER_DROP降到10dm抛投后，之前直接恢复PATROL会
        导致"一边巡航一边爬升"——恢复的目标航点通常离触发点有一段距离，如果
        是掠过式(非精确)航点，_on_arrival()只看xy不看z就会判定到达并继续
        推进，爬升还没完成水平就已经在移动，跟问题31(起飞爬升顺序)是同一类
        问题。改成两阶段：这里先进入RECOVER_HEIGHT，在当前水平位置原地爬升
        回目标航点的高度，爬满了才真正恢复PATROL(见_do_recover_height())。

        到达检测状态（arrival_start_time/_arrival_window/arrival_confirmed_time）
        的重置延后到RECOVER_HEIGHT完成、真正恢复PATROL的那一刻才做——如果在这里
        就重置，RECOVER_HEIGHT爬升期间流逝的时间会被navigate()的超时判断计入
        "航点等待时长"，可能导致爬升还没完成就被"超时强制跳过"。
        target_index/last_target_index仍然在这里恢复，因为_do_recover_height()
        需要用target_index查询要爬升回的目标高度。"""
        self.target_index = self.saved_target_index_before_fire
        self.last_target_index = self.target_index
        self.nav_mode = "RECOVER_HEIGHT"
        self._approach_start_time = None
        # 清空HOVER_DROP闭环锁定期间累积的PID积分项，避免带着旧误差历史进入
        # RECOVER_HEIGHT的原地爬升(_do_recover_height()会在第一次调用时重新
        # set_target，但reset()清掉积分项更干净，不留隐性偏置)
        self.x_pid.reset()
        self.y_pid.reset()
        self._hover_hold_pos = None
        self._recover_hold_pos = None

        # 2026-07-16真机测试发现：警示LED点亮后(_do_confirm_warn里的warn_led())
        # 从没有任何地方关掉，导致降落时灯还亮着——set_rgb_led()是状态型接口，
        # 点亮后不会自动熄灭，调用方必须自己在合适时机关掉。LED的作用是示警
        # "正在处理这次火情"，续飞后不该继续亮着占用状态，这里关掉。
        try:
            from Lcode.gpio_led import set_rgb_led
            set_rgb_led('OFF')
        except Exception as e:
            logger.error(f"set_rgb_led('OFF') 调用失败: {e}")

    def _do_approach(self, pos, yaw_cmd=0):
        """悬停对准正下方：独立小增益+死区+符号预验证的伺服修正，超时兜底进CONFIRM_WARN。
        见设计文档"APPROACH（视觉伺服对准正下方）"一节。"""
        if self.fire_vision is None:
            self.nav_mode = "CONFIRM_WARN"
            return
        latest = self.fire_vision.latest()
        dx_px, dy_px = latest.get("dx_px"), latest.get("dy_px")
        # 2026-07-17新增：PATROL分支触发APPROACH前会检查latest()新鲜度
        # (FIRE_VISION_STALE_S)，但APPROACH阶段本身的伺服修正之前没有做同样的
        # 检查——如果视觉线程卡死，latest()可能一直返回同一份冻结的旧坐标，
        # _do_approach()会持续伺服到这个不再代表真实目标位置的点上。跟dx_px为
        # None时一样按"没有检测"处理(悬停等待，不主动修正)。
        if dx_px is not None and time.time() - latest.get("t", 0) >= FIRE_VISION_STALE_S:
            dx_px, dy_px = None, None
        vx, vy = 0, 0
        centered = False

        if dx_px is not None and dy_px is not None:
            if self.is_approach_centered(dx_px, dy_px):
                logger.info("APPROACH: 已对准正下方")
                centered = True
                self.nav_mode = "CONFIRM_WARN"
            else:
                # 死区：像素偏移小于阈值不修正
                # 符号约定(2026-07-16物理确认下视摄像头安装朝向后推导，非猜测)：
                #   画面上边=+y、右边=+x(无镜像安装)
                #   dx_px>0(目标在画面右侧) → 目标物理上在+x方向 → 需要+x速度靠近 → vx与dx_px同号
                #   dy_px>0(目标在画面下方) → 画面"下"对应-y(上边才是+y) → 目标物理上在-y方向
                #     → 需要-y速度靠近 → vy与dy_px反号，不能直接用dy_px*增益(那样会正反馈发散，
                #     跟这个项目里yaw方向出过的同类问题一样)
                vx = 0 if abs(dx_px) < APPROACH_DEADBAND_PX else self.limit(
                    dx_px * APPROACH_GAIN, APPROACH_MAX_STEP_CMPS)
                vy = 0 if abs(dy_px) < APPROACH_DEADBAND_PX else self.limit(
                    -dy_px * APPROACH_GAIN, APPROACH_MAX_STEP_CMPS)
                self.set_speed(int(vx), int(vy), yaw_cmd, int(self._ramp_z_cm))
        else:
            self.set_speed(0, 0, yaw_cmd, int(self._ramp_z_cm))

        # 2026-07-16补充日志：之前APPROACH阶段完全没有逐帧记录，没法诊断"伺服
        # 是不是越修越偏"这类问题，只能靠对准/超时两条最终结果日志猜。
        now = time.time()
        if self._log_file and now - self._last_detect_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                with self._log_lock:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": "APPROACH",
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "dx_px": dx_px,
                        "dy_px": dy_px,
                        "vx": int(vx),
                        "vy": int(vy),
                        "centered": centered,
                        "yaw_cmd_sent": yaw_cmd,
                        **self._heading_log_fields(),
                    }) + "\n")
                    self._log_file.flush()
            except Exception:
                pass
            self._last_detect_log_time = now

        if not centered and self.approach_timed_out():
            logger.warning("APPROACH: 对准超时，按当前位置继续")
            self.nav_mode = "CONFIRM_WARN"

    def _do_confirm_warn(self):
        """点亮警示LED后立即进入HOVER_DROP。"""
        from Lcode.actuators import warn_led
        try:
            warn_led()
        except Exception as e:
            logger.error(f"warn_led() 调用失败: {e}")
        self.nav_mode = "HOVER_DROP"
        self._hover_drop_start_time = None
        self._hover_hold_pos = None  # 触发_do_hover_drop()第一次调用时锁定当前位置

    def _do_hover_drop(self, pos, yaw_cmd=0):
        """降到10dm悬停HOVER_DROP_DURATION_S后抛投+广播坐标，见设计文档"HOVER_DROP"一节。
        水平位置用x_pid/y_pid闭环锁定在进入本状态那一刻的坐标，不能像之前那样只发
        vx=vy=0的开环零速度指令——3秒悬停期间任何风扰/残余速度都会导致漂移，
        影响抛投精度和最终广播坐标的准确性(用户2026-07-16反馈：悬停期间需要闭环
        控制保证精度，不能开环)。"""
        if self._hover_hold_pos is None:
            self._hover_hold_pos = (pos[0], pos[1])
            self.x_pid.set_target(pos[0])
            self.y_pid.set_target(pos[1])
            self.x_pid.reset()
            self.y_pid.reset()
            logger.info(f"HOVER_DROP: 锁定水平位置 ({pos[0]:.2f}, {pos[1]:.2f})")

        self._step_ramp_z(HOVER_DROP_ALTITUDE_CM)

        vx = self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE
        vy = self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE
        vx = int(self.limit(vx, HOVER_HOLD_MAX_STEP_CMPS))
        vy = int(self.limit(vy, HOVER_HOLD_MAX_STEP_CMPS))
        self.set_speed(vx, vy, yaw_cmd, int(self._ramp_z_cm))

        # setpoint_ok: 内部软件ramp是否已收敛到目标值（跟原逻辑一样）。
        # measured_ok: pos[2](loop()里已被激光高度覆盖成米制真实测量值，见laser_height_valid
        # 分支)是否也接近目标高度。只信setpoint会在真实高度还没跟上（比如飞控/凌霄IMU响应滞后、
        # 或悬停中被风扰导致实际高度偏离）时提前开始计时，抛投时实际高度不在10dm。
        # 阈值10cm：比posthreshold_z(0.20m=20cm，navigate()到达检测用的量级)更严格，因为
        # HOVER_DROP要求精确到10dm这个赛题写死的高度用于抛投，容忍度不宜比普通到达检测更松；
        # 但也不需要严格到cm级——激光高度本身有噪声，10cm对应赛题100cm目标的10%相对误差，
        # 参考takeoff()里laser_cm的10cm确认容差保持一致。
        setpoint_ok = abs(self._ramp_z_cm - HOVER_DROP_ALTITUDE_CM) <= 2.0
        measured_ok = abs(pos[2] * 100 - HOVER_DROP_ALTITUDE_CM) <= 10.0

        # 2026-07-17新增：这个状态之前从来没写过飞行日志——2026-07-17真机测试
        # 复现了HOVER_DROP卡住不动(疑似问题26"高度不恢复")，但因为没有日志，
        # 完全没法诊断当时高度是冻结/漂移还是别的什么状态，只能确认"卡住了"这个
        # 事实。补上跟navigate()其它分支一样的节流日志，下次复现时至少能拿到
        # 高度曲线数据。
        now = time.time()
        if self._log_file and now - self._last_detect_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                with self._log_lock:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": "HOVER_DROP",
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "hover_hold_pos": list(self._hover_hold_pos) if self._hover_hold_pos else None,
                        "vx": vx, "vy": vy,
                        "yaw_cmd_sent": yaw_cmd,
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "setpoint_ok": setpoint_ok,
                        "measured_ok": measured_ok,
                        "hover_drop_start_time": self._hover_drop_start_time,
                        **self._heading_log_fields(),
                    }) + "\n")
                    self._log_file.flush()
            except Exception:
                pass
            self._last_detect_log_time = now

        if not (setpoint_ok and measured_ok):
            return  # 还没真正降到位，不开始计时

        if self._hover_drop_start_time is None:
            self._hover_drop_start_time = time.time()
            logger.info(f"HOVER_DROP: 到达{HOVER_DROP_ALTITUDE_CM:.0f}cm，悬停{HOVER_DROP_DURATION_S:.0f}s")
            return

        if time.time() - self._hover_drop_start_time < HOVER_DROP_DURATION_S:
            return

        from Lcode.actuators import drop_bag
        try:
            drop_bag()
        except Exception as e:
            logger.error(f"drop_bag() 调用失败: {e}")
        if self.serial_ground is not None:
            try:
                self.serial_ground.send_fire(int(pos[0] * 100), int(pos[1] * 100))
            except Exception as e:
                logger.error(f"serial_ground.send_fire() 调用失败: {e}")
        logger.info("HOVER_DROP: 抛投+坐标广播完成，恢复PATROL续飞")
        self.finish_hover_drop_and_resume()

    def _do_recover_height(self, pos, yaw_cmd=0):
        """HOVER_DROP抛投完成后，先在当前水平位置原地爬升回巡航高度，爬满了
        才真正恢复PATROL——不要一边巡航一边爬升(2026-07-17用户反馈，跟问题31
        起飞爬升顺序是同一类问题)。水平位置用x_pid/y_pid闭环锁定，模式跟
        _do_hover_drop()一样。"""
        if self._recover_hold_pos is None:
            self._recover_hold_pos = (pos[0], pos[1])
            self.x_pid.set_target(pos[0])
            self.y_pid.set_target(pos[1])
            self.x_pid.reset()
            self.y_pid.reset()
            target = self.targets[self.target_index] if self.target_index < len(self.targets) else None
            self._recover_target_z_cm = target[2] * 100 if target is not None else self._ramp_z_cm
            logger.info(f"RECOVER_HEIGHT: 锁定水平位置 ({pos[0]:.2f}, {pos[1]:.2f})，"
                        f"原地爬升回{self._recover_target_z_cm:.0f}cm")

        self._step_ramp_z(self._recover_target_z_cm)

        vx = self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE
        vy = self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE
        vx = int(self.limit(vx, HOVER_HOLD_MAX_STEP_CMPS))
        vy = int(self.limit(vy, HOVER_HOLD_MAX_STEP_CMPS))
        self.set_speed(vx, vy, yaw_cmd, int(self._ramp_z_cm))

        setpoint_ok = abs(self._ramp_z_cm - self._recover_target_z_cm) <= 2.0
        measured_ok = abs(pos[2] * 100 - self._recover_target_z_cm) <= 10.0

        now = time.time()
        if self._log_file and now - self._last_detect_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                with self._log_lock:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": "RECOVER_HEIGHT",
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "recover_hold_pos": list(self._recover_hold_pos) if self._recover_hold_pos else None,
                        "vx": vx, "vy": vy,
                        "yaw_cmd_sent": yaw_cmd,
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "recover_target_z_cm": self._recover_target_z_cm,
                        "setpoint_ok": setpoint_ok,
                        "measured_ok": measured_ok,
                        **self._heading_log_fields(),
                    }) + "\n")
                    self._log_file.flush()
            except Exception:
                pass
            self._last_detect_log_time = now

        if not (setpoint_ok and measured_ok):
            return  # 还没爬满，继续原地锁定悬停

        logger.info(f"RECOVER_HEIGHT: 已恢复到{self._recover_target_z_cm:.0f}cm，继续PATROL")
        self.nav_mode = "PATROL"
        self._recover_hold_pos = None
        # 到这里才重置到达检测状态，避免RECOVER_HEIGHT期间流逝的时间被计入
        # 航点等待时长，导致爬升刚完成就被判定超时强制跳过(见
        # finish_hover_drop_and_resume()的说明)
        self._arrival_window.clear()
        self.arrival_confirmed_time = None
        self.arrival_start_time = time.time()
        self.last_target_index = self.target_index
        self._cruise_arrival_count = 0
        self.x_pid.reset()
        self.y_pid.reset()

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

    # ================= 降落 =================
    def land(self):
        logger.info("降落")
        self.heading_hold.disarm("land")
        with lock:
            self.se_fc[2] = 0

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
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))

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
                land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)
                land_raw_imu = [0.0] * 6

            # 激光高度覆盖Z：跟 loop()/takeoff() 一样，T265自身Z轴未标定不是真实高度，
            # 这里如果继续用原始T265 Z，降落阶段记录的"高度"会是假数据，没法验证物理降落过程。
            with lock:
                laser_h = self.serial_fc_ref._last_laser_height_cm if self.serial_fc_ref else 0.0
            if laser_height_valid(laser_h):
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
                gaveup_logged = True

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
                    break
            else:
                unlock_confirm_count = 0
            if not gaveup_logged and time.time() - t_start >= LAND_CONFIRM_TIMEOUT_S:
                logger.warning("降落确认超时，强制退出")
                break
            time.sleep(0.03)

        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")
        self.heading_hold.disarm("stop_all")
        # 上锁指令必须最先发出、前面不能有任何阻塞调用——stop_all()由emergency_stop
        # 路径(飞控串口超时/T265丢失)触发，se_fc是独立50Hz发送线程读取的共享状态，
        # 越早写入，飞控收到断电/上锁指令就越早。见docs/known_issues.md #22
        # (电机停转可靠性是全项目最高优先级安全隐患)。
        with lock:
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[6] = sp_side
            self.se_fc[7] = 101
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

    def _update_heading_hold(self, yaw, confidence):
        status = self.heading_hold.update(yaw, confidence, time.time())
        if status.fault_reason and status.fault_reason != self._last_heading_fault_logged:
            logger.error(f"航向保持已锁存关闭: {status.fault_reason}")
            self._last_heading_fault_logged = status.fault_reason
        return status

    def _heading_log_fields(self):
        status = self._heading_status
        return {
            "heading_hold_enabled": status.enabled,
            "heading_hold_armed": status.armed,
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
        self.heading_hold.disarm("emergency")
        # 立即清掉发送线程可能仍在重复发送的旧yaw指令，不等下一次loop()进入stop_all()。
        with lock:
            self.se_fc[6] = sp_side
        self.emergency_stop = True
