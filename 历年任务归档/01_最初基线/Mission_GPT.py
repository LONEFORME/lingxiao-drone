import threading
import time
import json
import math
import os
import sys
from collections import deque
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock, fc_last_rx_time
from Lcode.k230_client import K230Client
from Lcode.coverage_planner import CoveragePlanner
from t265 import t265_class
from Lcode.rgb_led import rgb_led

put_height = 100
fly_height = 100
VEL_SCALE = 0.7  # XY/Yaw 速度缩放系数 (1.0=原速, 0.7=七成)
posthreshold_xy = 0.15  # XY 到达阈值（米），置信度高时使用，低置信度时动态增大
posthreshold_z = 0.20   # Z 到达阈值（米）
arrival_confirm_need = 15  # XY 连续确认到达次数 (~450ms)
arrival_timeout_max = 5.0  # 单航点超时（秒）
FLIGHT_LOG_INTERVAL = 1.0  # 飞行数据记录间隔（秒）
RAMP_STEP = 1.5        # cm per frame at 30 ms cycle ≈ 50 cm/s climb/descent rate
TAKEOFF_CONFIRM_NEED = 10     # consecutive frames within ±10 cm of target
TAKEOFF_TIMEOUT_S    = 15.0   # force transition to NAVIGATE after this
TAKEOFF_LIFTOFF_CM = 35.0  # 一键起飞只负责盲飞离地这一小段，其余交给navigate()的x/y PID+高度ramp爬升到真正目标高度
                            # 不能设太低：2026-07-06实测15cm时T265/激光近地面定位质量下降，起飞confirm超时+机体水平旋转
T265_CONFIDENCE_MIN = 2       # 定点所需最低追踪置信度 (0=失败,1=低,2=中,3=高)
T265_CONFIDENCE_WAIT_S = 8.0  # 等待置信度达标的超时时间
LAND_CONFIRM_TIMEOUT_S = 25.0  # 降落触发后最多等待多久确认unlock_sta==0(已上锁)，超时也强制退出，避免卡死
                                # 2026-07-09从10.0改为25.0，同步basic/的修复：真机数据显示完整降落
                                # 序列常需约11秒(平滑下降+贴地维持+收尾)，10秒窗口本来就卡在临界点
LAND_UNLOCK_CONFIRM_COUNT = 5  # 降落确认去抖：要连续读到N次unlock_sta==0才真正确认已上锁，不是单次就退出。
                                # 2026-07-09真机观察到疑似假阳性——终端打印"已上锁"退出，但用户确认电机实际
                                # 未停转/没有真正降落；飞行日志显示确认发生的那一刻之前unlock_sta全程是1，
                                # 说明原逻辑单次读到0就退出，容易被单帧通信噪声/校验巧合触发误判
ARRIVAL_VEL_THRESH = 0.05  # 到达判定除了位置阈值外，还要求T265速度模长小于此值(m/s)，避免带着残余速度就触发land()盲降
ARRIVAL_VEL_WINDOW = 5  # 到达判定用的速度取最近N帧均值而非单帧瞬时值，平滑T265速度噪声尖峰
                         # (2026-07-07实测: 单帧瞬时速度噪声可达0.07m/s，用瞬时值+连续N次达标会导致到达确认永远凑不齐、超时强制跳过)
ARRIVAL_CONFIRM_RATIO = 0.6  # 到达确认改用滑动窗口比例制而非严格连续帧数：旧逻辑下任意一帧不达标就把
                              # 计数器清零重来，2026-07-08矩形路径测试实测达标帧占比只有30-40%，几乎不可能
                              # 连续凑够arrival_confirm_need帧，导致大多数航点靠超时兜底而非真正确认到达
                              # (2026-07-08复测: 0.8比例下仍有部分航点(占比26-34%)无法确认，下调到0.6)


def arrival_window_confirmed(window, need, ratio):
    """window: 最近若干帧"位置+速度是否同时达标"的布尔值(deque)。
    窗口填满(len>=need)且达标帧占比>=ratio才算确认到达——替代旧的"严格连续N帧"
    逻辑，单帧噪声不会让已经积累的进度清零(见 ARRIVAL_CONFIRM_RATIO 常量注释)。"""
    return len(window) >= need and (sum(window) / len(window)) >= ratio


LASER_HEIGHT_MAX_M = 10.0  # 激光高度覆盖Z轴前的合理性上限：2026-07-10真机测试(basic_radar)发现降落末尾
                            # 激光传感器偶发返回类似0xFFFFFFFF的错误码，除以100后变成约4.29e7米的垃圾值，
                            # 原逻辑只判断laser_h>0.05、没有上限，会把这个垃圾值当真实高度写进pos[2]。
                            # 10m远超室内飞行实际高度(实测未超过1.4m)，只用来挡掉这种量级的错误码。


def laser_height_valid(laser_h):
    """激光高度是否合理，可以用来覆盖pos[2]/land_pos[2]。见 LASER_HEIGHT_MAX_M 注释：
    2026-07-10真机测试(basic_radar)捕获到传感器错误码(约0xFFFFFFFF/100)未被过滤污染日志的真实案例。"""
    return 0.05 < laser_h <= LASER_HEIGHT_MAX_M


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int], re_dmz: List[int], se_dmz: List[int],
                 realsense_obj=None, k230_client=None, serial_fc_ref=None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.re_dmz = re_dmz
        self.se_dmz = se_dmz
        self.k230 = k230_client
        self.serial_fc_ref = serial_fc_ref  # 用于查询激光高度

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False
        self.realsense = realsense_obj

        # PID控制器（常驻）
        self.x_pid = PID(0, 0)
        self.y_pid = PID(0, 0)
        self.yaw_pid = PID(1, 0)

        # 当前目标
        self.current_target = None

        # 航点
        self.targets = self.load_waypoints()
        
        self.target_index = 0
        self.emergency_stop = False

        # 到达判断状态（进入航点时自动重置）
        self._arrival_window = deque(maxlen=arrival_confirm_need)
        self.arrival_start_time = 0.0
        self.last_target_index = -1
        self._vel_window = deque(maxlen=ARRIVAL_VEL_WINDOW)

        # Height ramp state (cm); steps toward target each navigate() frame
        self._ramp_z_cm = 0.0

        # 飞行数据日志
        self._log_file = None
        self._last_log_time = 0.0

    def load_waypoints(self):
        """从router.txt文件加载航点"""
        try:
            with open('router.txt', 'r') as f:      
                waypoints = []
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and not line.startswith('x'):
                        # 格式: x,y,z  (逗号分隔)
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                x = float(parts[0].strip())
                                y = float(parts[1].strip())
                                z = float(parts[2].strip()) 
                                waypoints.append([x, y, z])  
                            except ValueError:
                                logger.warning(f"无效的航点数据: {line}")
                if waypoints:
                    logger.info(f"从router.txt加载了{len(waypoints)}个航点")
                    return waypoints
                else:
                    logger.warning("router.txt中没有有效的航点数据，使用默认航点")
        except FileNotFoundError:
            logger.warning("router.txt文件不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取router.txt时出错: {str(e)}，使用默认航点")

        # 默认航点（第一个航点为起飞点：原地爬升到目标高度）
        default_waypoints = [[0.0, 0.0,put_height/100], [0.5, 0.0,put_height/100], [0.5, 0.5,put_height/100], [0.0, 0.5,put_height/100]]
        logger.info(f"使用默认航点: {default_waypoints}")
        return default_waypoints

    def _gs_data_valid(self):
        """校验地面站数据：3个有效禁飞区坐标 (A1~A9, B1~B7)"""
        if len(self.re_dmz) < 3:
            return False
        for a_str, b_str in self.re_dmz:
            try:
                a = int(a_str[1:])
                b = int(b_str[1:])
                if not (1 <= a <= 9 and 1 <= b <= 7):
                    return False
            except (ValueError, IndexError):
                return False
        return True

    # ================= 启动 =================
    def start(self):

        if self.realsense.start():
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

        # === 动态路径生成（优先用地面站禁飞区，失败则保留 load_waypoints 结果） ===
        gs_ok = False
        logger.info("等待地面站禁飞区数据...")
        for _ in range(50):  # 5s 超时
            with lock:
                gs_ok = self._gs_data_valid()
            if gs_ok:
                break
            time.sleep(0.1)

        if gs_ok:
            with lock:
                forbidden = list(self.re_dmz)
            logger.info(f"地面站禁飞区: {forbidden}")
            planner = CoveragePlanner(forbidden)
            name, full_path, steps = planner.plan()
            self.targets = []
            for xy in full_path:
                rx, ry = planner.xy_to_real(xy)
                self.targets.append([rx, ry, fly_height / 100])
            self.target_index = 0
            self.last_target_index = -1
            logger.info(f"动态路径: 策略={name}, {len(self.targets)}航点")
        else:
            logger.warning("地面站数据不可用，使用 router.txt 静态路径")

        self.task_running = True
        self.state = "TAKEOFF"
        # 打开飞行数据日志
        try:
            path = os.path.dirname(os.path.realpath(sys.argv[0]))
            log_file = open(path + "/flight_data.jsonl", "a")
            log_file.write(json.dumps({"event": "task_start"}) + "\n")
            log_file.flush()
            self._log_file = log_file
        except Exception:
            pass

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # 串口超时检测：超过2秒无飞控回传数据则急停
            if fc_last_rx_time.value > 0 and time.time() - fc_last_rx_time.value > 2.0:
                logger.error("飞控串口超时无回传，触发紧急降落")
                self.emergency_stop = True
                continue

            # T265存活检测：数据采集线程已停止则急停
            if self.t265_ok and not self.realsense.is_running():
                logger.error("T265数据采集已停止，触发紧急降落")
                self.emergency_stop = True
                continue

            # 获取定位
            try:
                pos = self.realsense.get_position()
                yaw = self.realsense.get_orientation()[2]
            except Exception:
                logger.error("T265 ERROR")
                continue

            # P1a: 用飞控回传的激光测距高度覆盖 Z 轴（替代 T265 伪 Z 数据）
            if self.serial_fc_ref is not None:
                # 注意: _last_laser_height_cm 需要在 listen_fc 的 lock 中写入，保持一致
                with lock:
                    laser_h = self.serial_fc_ref._last_laser_height_cm
                if laser_height_valid(laser_h):  # 有效值 > 5cm 且 <= LASER_HEIGHT_MAX_M
                    pos[2] = laser_h

            # 状态机调度
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
    def takeoff(self):
        logger.info("takeoff: started")

        target_h_cm = TAKEOFF_LIFTOFF_CM  # 一键起飞只爬升到离地高度，真正目标高度交给 navigate() 闭环爬升

        with lock:
            self.se_fc[5] = int(target_h_cm)  # com_z：一键起飞目标高度，必须在 task_sta 触发前写入，
            self.se_fc[2] = 1  # trigger FC: unlock + mode switch  否则飞控读到的是 se_fc 初始默认值(120cm)而非本次航点高度

        confirm_count = 0
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start

            # Yaw stabilization during climb
            if self.t265_ok:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    # 2026-07-09临时回退问题16的角度修复：basic/真机验证发现yaw修正回路一旦
                    # 真正输出非零指令会导致yaw持续发散(疑似固件/协议层符号约定与Python假设
                    # 不一致，形成正反馈)，触发一次~90°失控需人工介入。喂弧度让PID输出重新
                    # 恒近似为0，回到已知安全状态，直到符号问题查清。
                    vyaw = int(self.limit(self.yaw_pid.get_pid(yaw) * VEL_SCALE, 30))
                    with lock:
                        self.se_fc[6] = vyaw + sp_side
                except Exception:
                    pass

            # Height confirmation (note: _last_laser_height_cm is in metres)
            with lock:
                laser_m = self.serial_fc_ref._last_laser_height_cm \
                          if self.serial_fc_ref else 0.0
            laser_cm = laser_m * 100.0

            if laser_cm > 5.0 and abs(laser_cm - target_h_cm) <= 10.0:
                confirm_count += 1
            else:
                confirm_count = 0

            if confirm_count >= TAKEOFF_CONFIRM_NEED:
                logger.info(f"takeoff: height confirmed {laser_cm:.0f} cm")
                break

            if elapsed >= TAKEOFF_TIMEOUT_S:
                logger.warning("takeoff: timeout, proceeding anyway")
                break

            time.sleep(0.03)

        # Seed ramp at first waypoint height so navigate() starts smooth
        self._ramp_z_cm = target_h_cm
        self.state = "NAVIGATE"
    
    def navigate(self, pos, yaw):
        if self.target_index >= len(self.targets):
            logger.info("所有航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]

        # Z轴: 直接传航点高度（米→厘米），FC自主控高
        target_z = int(target[2] * 100)

        # 每帧更新地面站: idx=实时进度, cls/cnt=哨兵值(FF/0)
        with lock:
            self.se_dmz[1] = self.target_index & 0xFF
            if self.se_dmz[2] != 0xFF or self.se_dmz[3] != 0:
                self.se_dmz[2] = 0xFF
                self.se_dmz[3] = 0

        # XY/Yaw: PID计算速度（检测期间也运行，维持悬停）
        confidence = self.realsense.get_tracking_confidence() if self.t265_ok else 0

        # P1c: 置信度==0 → 悬停不飞
        if confidence == 0:
            logger.warning("T265追踪完全丢失，悬停等待恢复")
            # Hold current ramp height during hover; avoid a Z jump on lost tracking
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            # 2026-07-09从basic_radar/补同步(2026-07-08已在那边修复)：这里原本直接return
            # 会跳过日志写入，导致T265追踪丢失期间完全没有数据记录。
            now = time.time()
            if now - self._last_log_time >= FLIGHT_LOG_INTERVAL and self._log_file is not None:
                try:
                    record = {
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": 0, "vy": 0, "vyaw": 0,
                        "t265_confidence_lost": True,
                    }
                    self._log_file.write(json.dumps(record) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
            return
        self.x_pid.set_target(target[0])
        self.y_pid.set_target(target[1])
        self.yaw_pid.set_target(0)
        vx = self.x_pid.get_pid(pos[0])
        vy = self.y_pid.get_pid(pos[1])
        # 2026-07-09临时回退问题16的角度修复，理由同上(navigate()同一个符号问题)
        vyaw = self.yaw_pid.get_pid(yaw)

        vx *= 100 * VEL_SCALE
        vy *= 100 * VEL_SCALE
        vx = int(self.limit(vx, 40))
        vy = int(self.limit(vy, 40))
        vyaw = int(self.limit(vyaw * VEL_SCALE, 30))
        # Smooth height: step the ramp toward target_z, send ramped value (no jumps)
        self._step_ramp_z(target_z)
        self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))

        # T265 速度（到达检测的速度门槛 + 后面日志/终端输出共用，避免重复取值）
        if self.t265_ok:
            t265v = self.realsense.get_velocity()
        else:
            t265v = (0.0, 0.0, 0.0)

        # === 到达判断 ===
        # P0b: 动态阈值 - 按追踪置信度自适应调整XY阈值
        if confidence >= 3:
            xy_thresh = 0.10   # 高置信度: 严格阈值
        elif confidence == 2:
            xy_thresh = posthreshold_xy  # 中置信度: 默认
        else:
            xy_thresh = 0.30   # 低置信度(1): 宽松阈值，避免噪声误判
        dx = abs(pos[0] - target[0])
        dy = abs(pos[1] - target[1])
        dz = abs(pos[2] - target[2])
        # 速度用最近N帧均值而非瞬时值，平滑T265速度噪声尖峰(见 ARRIVAL_VEL_WINDOW 注释)
        self._vel_window.append((t265v[0], t265v[1]))
        avg_vx = sum(v[0] for v in self._vel_window) / len(self._vel_window)
        avg_vy = sum(v[1] for v in self._vel_window) / len(self._vel_window)
        speed = math.hypot(avg_vx, avg_vy)

        if self.target_index != self.last_target_index:
            self.last_target_index = self.target_index
            self._arrival_window.clear()
            self.arrival_start_time = time.time()

        if dx > 0.3:
            self.x_pid.reset()
        if dy > 0.3:
            self.y_pid.reset()

        xy_ok = dx < xy_thresh and dy < xy_thresh
        z_ok = dz < posthreshold_z
        vel_ok = speed < ARRIVAL_VEL_THRESH

        self._arrival_window.append(xy_ok and z_ok and vel_ok)
        if arrival_window_confirmed(self._arrival_window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO):
            logger.info(f"到达航点 {self.target_index}")
            self._on_arrival(target)

        if time.time() - self.arrival_start_time >= arrival_timeout_max:
            logger.warning(f"航点 {self.target_index} 超时，强制跳过")
            self.target_index += 1

        # 光流融合速度（帧1 of1_dx/dy，用于跟 T265 速度交叉对比）
        with lock:
            of1_dx = self.re_fc[9] if len(self.re_fc) > 9 else 0
            of1_dy = self.re_fc[10] if len(self.re_fc) > 10 else 0

        # === 飞行数据日志（每秒记录一次） ===
        now = time.time()
        if now - self._last_log_time >= FLIGHT_LOG_INTERVAL and self._log_file is not None:
            try:
                record = {
                    "t": round(now, 3),
                    "state": self.state,
                    "target_idx": self.target_index,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy, "vyaw": vyaw,
                    "t265_vel": [round(t265v[0], 4), round(t265v[1], 4)],
                    "of1_vel_cms": [of1_dx, of1_dy],
                }
                self._log_file.write(json.dumps(record) + "\n")
                self._log_file.flush()
            except Exception:
                pass
            self._last_log_time = now

        if self.t265_ok:
            t265_str = f"| t265v=({t265v[0]:+.2f},{t265v[1]:+.2f}) | of1=({of1_dx:+d},{of1_dy:+d})"
        else:
            t265_str = ""

        print(
            f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"| tgt=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) "
            f"| v=({vx:>3},{vy:>3}) "
            f"| send=({self.se_fc[3]:>3},{self.se_fc[4]:>3},{self.se_fc[5]:>3})"
            f"{t265_str}",
            end="",
            flush=True
        )

    # ================= 到达处理 =================
    def _on_arrival(self, target):
        """到达航点后直接前往下一航点（检测已关闭）"""
        if self.target_index == len(self.targets) - 2:
            rgb_led('R', 1)
        self.target_index += 1

    # ================= 降落 =================
    def land(self):
        logger.info("降落")

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
        while True:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))

            if self.t265_ok and self.realsense:
                try:
                    land_pos = list(self.realsense.get_position())
                    land_yaw = self.realsense.get_orientation()[2]
                    land_tv = self.realsense.get_velocity()
                except Exception:
                    land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)
            else:
                land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)

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

            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "pos": [round(land_pos[0], 4), round(land_pos[1], 4), round(land_pos[2], 4)],
                        "t265_yaw_deg": round(math.degrees(land_yaw), 2),
                        "t265_vel": [round(land_tv[0], 4), round(land_tv[1], 4)],
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
            if time.time() - t_start >= LAND_CONFIRM_TIMEOUT_S:
                logger.warning("降落确认超时，强制退出")
                break
            time.sleep(0.03)

        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")

        try:
            self._log_file.close()
        except Exception:
            pass

        with lock:
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[6] = sp_side
            self.se_fc[7] = 101

        self.realsense.stop()
        if self.k230:
            self.k230.close()
        self.task_running = False

    # ================= 控制接口 =================
    def set_speed(self, x, y, yaw, z):
        with lock:
            self.se_fc[3] = x + sp_side
            self.se_fc[4] = y + sp_side
            self.se_fc[5] = z
            self.se_fc[6] = yaw + sp_side
        
    # ================= 工具 =================
    def limit(self, v, max_v=0.3):
        return max(min(v, max_v), -max_v)

    def _step_ramp_z(self, target_z_cm: float):
        """Step the ramped Z command one frame toward target_z_cm (no jumps)."""
        if self._ramp_z_cm < target_z_cm - RAMP_STEP:
            self._ramp_z_cm += RAMP_STEP
        elif self._ramp_z_cm > target_z_cm + RAMP_STEP:
            self._ramp_z_cm -= RAMP_STEP
        else:
            self._ramp_z_cm = target_z_cm

    # ================= 急停 =================
    def emergency(self):
        logger.warning("紧急停止触发！")
        self.emergency_stop = True
