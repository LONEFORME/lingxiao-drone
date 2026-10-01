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
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock, fc_last_rx_time
from Lcode.Lradar import PoleTracker
from t265 import t265_class

# ---------- 常量 ----------
DRY_RUN = os.getenv("DRONE_DRY_RUN", "0") == "1"  # 桌面测试: 不解锁飞控，电机不会转
put_height = 100
VEL_SCALE = 0.7
posthreshold_xy = 0.15
posthreshold_z = 0.20
arrival_confirm_need = 15
arrival_hold_s = 0.3   # 2026-07-10尝试降到1.0(-33%)想提速，真机测试反而更慢(22.62s vs 19.69s，
                        # 2/4航点超时)——因为当时arrival_timeout_max=5.0+arrival_hold_s是耦合公式，
                        # 缩短停留时间同时也缩短了超时上限，滑动窗口攒不够确认帧数就被更早截断，
                        # 超时本身比真正确认更耗时。2026-07-12把两者解耦后先压缩到0.7秒真机验证有效
                        # (5/6航点确认，平均4.67秒/段，比历史基线快12-20%)，沿用同一条杠杆继续压到0.3。
                        # 到达判定满足后，在原地强制停留观察的时长（阶跃响应测试用）
arrival_timeout_max = 6.5  # 2026-07-12改成独立常量(不再是 5.0+arrival_hold_s 的耦合公式)，
                            # 锁定改动前的有效值，这样调整arrival_hold_s不会连带影响超时上限
T265_CONFIDENCE_MIN = 2       # 定点所需最低追踪置信度 (0=失败,1=低,2=中,3=高)
T265_CONFIDENCE_WAIT_S = 8.0  # 等待置信度达标的超时时间
FLIGHT_LOG_INTERVAL = 0.05
RAMP_STEP = 1.5
TAKEOFF_CONFIRM_NEED = 10
TAKEOFF_TIMEOUT_S = 15.0
TAKEOFF_LIFTOFF_CM = 35.0  # 一键起飞只负责盲飞离地这一小段，其余交给navigate()的x/y PID+高度ramp爬升到真正目标高度
                            # 不能设太低：2026-07-06实测15cm时T265/激光近地面定位质量下降，起飞confirm超时+机体水平旋转
LAND_CONFIRM_TIMEOUT_S = 25.0  # 降落触发后最多等待多久确认unlock_sta==0(已上锁)，超时也强制退出，避免卡死
                                # 2026-07-09从10.0改为25.0，同步basic/的修复：真机数据显示完整降落
                                # 序列常需约11秒(平滑下降+贴地维持+收尾)，10秒窗口本来就卡在临界点
LAND_UNLOCK_CONFIRM_COUNT = 5  # 降落确认去抖：要连续读到N次unlock_sta==0才真正确认已上锁，不是单次就退出。
                                # 2026-07-09真机观察到疑似假阳性——终端打印"已上锁"退出，但用户确认电机实际
                                # 未停转/没有真正降落；飞行日志显示确认发生的那一刻之前unlock_sta全程是1，
                                # 说明原逻辑单次读到0就退出，容易被单帧通信噪声/校验巧合触发误判
LASER_HEIGHT_MAX_M = 10.0  # 激光高度覆盖Z轴前的合理性上限：2026-07-10真机测试发现降落末尾激光
                            # 传感器偶发返回类似0xFFFFFFFF的错误码，除以100后变成约4.29e7米的垃圾值，
                            # 原逻辑只判断laser_h>0.05、没有上限，会把这个垃圾值当真实高度写进pos[2]。
                            # 10m远超室内飞行实际高度(实测未超过1.4m)，只用来挡掉这种量级的错误码。
ARRIVAL_VEL_THRESH = 0.05  # 到达判定除了位置阈值外，还要求T265速度模长小于此值(m/s)，避免带着残余速度就触发land()盲降
ARRIVAL_VEL_WINDOW = 5  # 到达判定用的速度取最近N帧均值而非单帧瞬时值，平滑T265速度噪声尖峰
                         # (2026-07-07实测: 单帧瞬时速度噪声可达0.07m/s，用瞬时值+连续N次达标会导致到达确认永远凑不齐、超时强制跳过)
ARRIVAL_CONFIRM_RATIO = 0.6  # 到达确认改用滑动窗口比例制而非严格连续帧数：旧逻辑下任意一帧不达标就把
                              # 计数器清零重来，2026-07-08矩形路径测试实测达标帧占比只有30-40%，几乎不可能
                              # 连续凑够arrival_confirm_need帧，导致大多数航点靠超时兜底而非真正确认到达
                              # (2026-07-08复测: 0.8比例下仍有部分航点(占比26-34%)无法确认，下调到0.6)
POLE_POLL_INTERVAL_S = 0.5   # PoleTracker轮询间隔，跟07-07真机测试/回放验证用的节奏一致
POLE_DANGER_DIST_M = 0.75    # 确认的杆子距飞机当前位置小于此值就悬停。2026-07-09从0.6上调到0.9后，
                              # 2026-07-10真机测试悬停过早触发(在起飞点附近就报0.90m)，用户反馈判定阈值
                              # 偏保守，下调到0.6/0.9的中间值0.75；该距离是T265中心点到杆子的距离，未减去
                              # 机身物理半径，真机观察0.6m触发时桨叶到杆子实际间隙只有约20cm(旧数据，
                              # 0.75m档尚未实测验证间隙)
POLE_RESUME_DIST_M = 0.9     # 已经在悬停时，距离要超过这个值(比POLE_DANGER_DIST_M更远)才恢复导航——
                              # 滞回区间(保持0.15m不变)，避免距离刚好卡在POLE_DANGER_DIST_M附近抖动时
                              # 悬停状态反复
POLE_HOVER_TIMEOUT_S = 15.0   # 2026-07-09新增：悬停位置修正生效后，杆子如果一直不移开会一直悬停
                              # 下去(此前版本靠位置漂移bug"意外"带出安全区域才结束悬停，修复漂移后
                              # 暴露出这个问题)。超时后没有绕行能力，直接原地触发降落，不冒险恢复
                              # 导航飞向原目标(可能正对着障碍物)。绕行是后续单独设计的功能，这里只是
                              # 兜底超时。
                              # 触发/取消(2026-07-08真机0.1m步进接近测试观察到这个问题)
POLE_YAW_SIGN = 1            # 未标定！CLAUDE.md已知问题13——真机/台架标定前只是假设值，
                              # 标定结果可能是+1也可能是-1，标定前这个避障功能的世界坐标可能是错的

YAW_TEST_KP = float(os.getenv("DRONE_YAW_TEST_KP", "0"))
# 问题16：2026-07-09用原始Kp=1.5闭环触发过近90°失控事故，此后yaw_pid长期保持
# 喂弧度的安全回退状态(恒输出≈0，等于没有yaw修正)。2026-07-12递进式真机复测
# (单独会话+小步长+人工全程待命)确认稳定性边界在[0.45,0.5]之间：Kp=0.3/0.4/0.45
# 均收敛，Kp=0.5确认无界发散。同一天曾短暂把Kp=0.4设为默认值正式启用，但事后
# 对比历史"完全不修正"基线(2026-07-08记录，凌霄IMU纯姿态自稳，yaw漂移峰值
# 6.12°、会自己回归)发现：今天"开启修正"的样本(峰值6.13°~10.38°)并不比这个
# 不修正基线更好，甚至部分样本更差——修正是否真的有效果缺乏证据支持，而下行
# 风险(Kp=0.5发散、Kp=0.45方差大)是明确的。**已改回默认禁用**，喂弧度、恒
# 输出≈0。>0时才切换成喂角度+此增益闭环，用于问题16后续需要时的真机复测——
# 如果要重新考虑投入使用，必须先设计出能证明"修正比不修正好"的对照测试，
# 不能只看峰值幅度是否在合理范围内。


def nearest_confirmed_pole_dist(confirmed_poles, x, y):
    """confirmed_poles: PoleTracker.confirmed_poles()的返回值(list of {'x','y','hits'})。
    返回离(x,y)最近的确认杆子的距离(m)；没有杆子返回None。"""
    if not confirmed_poles:
        return None
    return min(math.hypot(p["x"] - x, p["y"] - y) for p in confirmed_poles)


def arrival_window_confirmed(window, need, ratio):
    """window: 最近若干帧"位置+速度是否同时达标"的布尔值(deque)。
    窗口填满(len>=need)且达标帧占比>=ratio才算确认到达——替代旧的"严格连续N帧"
    逻辑，单帧噪声不会让已经积累的进度清零(见 ARRIVAL_CONFIRM_RATIO 常量注释)。"""
    return len(window) >= need and (sum(window) / len(window)) >= ratio


def laser_height_valid(laser_h):
    """激光高度是否合理，可以用来覆盖pos[2]/land_pos[2]。见 LASER_HEIGHT_MAX_M 注释：
    2026-07-10真机测试捕获到传感器错误码(约0xFFFFFFFF/100)未被过滤污染日志的真实案例。"""
    return 0.05 < laser_h <= LASER_HEIGHT_MAX_M


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int],
                 realsense_obj: Optional[t265_class] = None,
                 serial_fc_ref=None, radar_obj=None):
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
        self._yaw_test_enabled = YAW_TEST_KP > 0
        if self._yaw_test_enabled:
            self.yaw_pid = PID(1, 0, p=YAW_TEST_KP, i=0, d=0.05)
            logger.warning(f"问题16排查：yaw修正回路以Kp={YAW_TEST_KP}闭环启用(喂角度)，"
                            f"非默认状态，需人工全程监控")
        else:
            self.yaw_pid = PID(1, 0)

        # 航点
        self.targets = self.load_waypoints()
        self.target_index = 0
        self.emergency_stop = False

        # 到达判断
        self._arrival_window = deque(maxlen=arrival_confirm_need)
        self.arrival_start_time = 0.0
        self.arrival_confirmed_time: Optional[float] = None
        self.last_target_index = -1
        self._vel_window = deque(maxlen=ARRIVAL_VEL_WINDOW)

        # 高度 ramp
        self._ramp_z_cm = 0.0

        # 飞行数据日志
        self._log_file = None
        self._last_log_time = 0.0

        # 雷达避障(可选)
        self.radar = radar_obj
        self.pole_tracker = PoleTracker(yaw_sign=POLE_YAW_SIGN) if radar_obj is not None else None
        self._last_pole_poll_time = 0.0
        self._pole_hovering = False  # 只在悬停状态切换时打日志，不是每帧刷屏
        self._hover_hold_pos = None  # 悬停期间用x_pid/y_pid锁定的位置(进入悬停那一刻的pos)
        self._hover_start_time = None  # 悬停开始时间，用于POLE_HOVER_TIMEOUT_S超时判断

    def load_waypoints(self):
        try:
            with open('router.txt', 'r') as f:
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
            self._log_file.write(json.dumps({"event": "task_start"}) + "\n")
            self._log_file.flush()
        except Exception:
            pass

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
    def takeoff(self):
        if DRY_RUN:
            logger.warning("takeoff: DRY_RUN 模式，不发送解锁指令，电机不会转")
        else:
            logger.info("takeoff: started")

        target_h_cm = TAKEOFF_LIFTOFF_CM  # 一键起飞只爬升到离地高度，真正目标高度交给 navigate() 闭环爬升

        with lock:
            self.se_fc[5] = int(target_h_cm)  # com_z：一键起飞目标高度，必须在 task_sta 触发前写入，
            self.se_fc[2] = 0 if DRY_RUN else 1  # 否则飞控读到的是 se_fc 初始默认值(120cm)而非本次航点高度

        confirm_count = 0
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start

            yaw = 0.0
            vyaw = 0
            if self.t265_ok and self.realsense:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    # 默认喂弧度(问题16已知安全回退状态，恒输出≈0)；YAW_TEST_KP>0时才喂角度，
                    # 用于问题16的低增益递进式复测，见 YAW_TEST_KP 常量注释
                    yaw_input = math.degrees(yaw) if self._yaw_test_enabled else yaw
                    vyaw = int(self.limit(self.yaw_pid.get_pid(yaw_input) * VEL_SCALE, 30))
                    with lock:
                        self.se_fc[6] = vyaw + sp_side
                except Exception:
                    pass

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
                    self._log_file.write(json.dumps({
                        "t": round(time.time(), 3),
                        "state": "TAKEOFF",
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "fc_yaw_deg": round(fc_yaw_deg, 2),
                        "vyaw": vyaw,
                        "laser_cm": round(laser_cm, 1),
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
        if self.target_index >= len(self.targets):
            logger.info("全部航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]
        target_z = int(target[2] * 100)

        # 雷达避障：检测到确认的杆子且距离过近就悬停，不绕行。
        # 触发(POLE_DANGER_DIST_M)和恢复(POLE_RESUME_DIST_M)用两个不同阈值(滞回)，
        # 避免距离刚好卡在阈值附近抖动时悬停状态反复触发/取消。
        pole_hover = False
        pole_dist = None
        confirmed_poles_list = []  # 2026-07-09新增：记录全部确认杆子(不只是最近的一个)，
                                    # 用于验证多障碍物场景下是否真的同时跟踪了多个目标
        if self.pole_tracker is not None:
            now = time.time()
            if now - self._last_pole_poll_time >= POLE_POLL_INTERVAL_S:
                self._last_pole_poll_time = now
                self.pole_tracker.update(self.radar, pos[0], pos[1], yaw)
            confirmed = self.pole_tracker.confirmed_poles()
            confirmed_poles_list = [
                {"x": round(p["x"], 3), "y": round(p["y"], 3), "hits": p["hits"],
                 "dist": round(math.hypot(p["x"] - pos[0], p["y"] - pos[1]), 3)}
                for p in confirmed
            ]
            pole_dist = nearest_confirmed_pole_dist(confirmed, pos[0], pos[1])
            if self._pole_hovering:
                pole_hover = pole_dist is not None and pole_dist < POLE_RESUME_DIST_M
            else:
                pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M

        if pole_hover:
            if not self._pole_hovering:
                logger.warning(f"检测到杆子距离{pole_dist:.2f}m，悬停等待")
                self._pole_hovering = True
                # 2026-07-09修复：进入悬停时记住当前位置，用x_pid/y_pid持续锁定，
                # 而不是常量发送(0,0,0)速度——后者只是"目标速度为0"，不修正实际位置
                # 漂移，真机测试发现~23秒悬停里飞机在没有任何指令的情况下真实漂移
                # 了约0.5m(t265_vel持续非零、位置连续平滑漂移，不是数据毛刺)。
                self._hover_hold_pos = (pos[0], pos[1])
                self._hover_start_time = time.time()
                self.x_pid.set_target(self._hover_hold_pos[0])
                self.y_pid.set_target(self._hover_hold_pos[1])
            elif time.time() - self._hover_start_time >= POLE_HOVER_TIMEOUT_S:
                # 悬停超时：没有绕行能力，不冒险恢复导航飞向原目标(可能正对着障碍物)，
                # 直接原地触发降落。
                logger.warning(f"悬停超过{POLE_HOVER_TIMEOUT_S:.0f}秒仍未恢复导航，原地触发降落")
                self._pole_hovering = False
                self._hover_hold_pos = None
                self._hover_start_time = None
                self.state = "LAND"
                return
            vx = int(self.limit(self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE, 40))
            vy = int(self.limit(self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE, 40))
            self.set_speed(vx, vy, 0, int(self._ramp_z_cm))
            # 2026-07-08修复：这里原本直接return，会跳过下面的飞行日志写入，
            # 导致悬停期间完全没有位置数据被记录(真机测试发现日志时间戳有秒级空白)。
            # 这里单独写一份简化日志(不含光流/姿态遥测，避免为了几个字段重复读锁)。
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                tv = self.realsense.get_velocity() if (self.t265_ok and self.realsense) else (0.0, 0.0, 0.0)
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": vx, "vy": vy, "vyaw": 0,
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "pole_hover": self._pole_hovering,
                        "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                        "hover_hold_pos": list(self._hover_hold_pos),
                        "confirmed_poles": confirmed_poles_list,
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
            print(f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                  f"| 悬停中(杆子距离{pole_dist:.2f}m) v=({vx},{vy})", end="", flush=True)
            return
        elif self._pole_hovering:
            logger.info("杆子确认已消失，恢复导航")
            self._pole_hovering = False
            self._hover_hold_pos = None
            self._hover_start_time = None

        confidence = self.realsense.get_tracking_confidence() if (self.t265_ok and self.realsense) else 0

        if confidence == 0 and self.t265_ok:
            logger.warning("T265 追踪丢失，悬停等待")
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            # 2026-07-08修复：同pole_hover分支一样，这里原本直接return会跳过日志写入，
            # 导致T265追踪丢失期间也是日志空白(此处pos/yaw本身可能是丢失前的最后已知值，
            # 只是留个"发生过丢失"的记录，不代表这段时间位置真的没变)。
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": 0, "vy": 0, "vyaw": 0,
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "t265_confidence_lost": True,
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
            print(f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) | T265追踪丢失，悬停等待", end="", flush=True)
            return

        if self.t265_ok and self.realsense:
            self.x_pid.set_target(target[0])
            self.y_pid.set_target(target[1])
            self.yaw_pid.set_target(0)
            vx = self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE
            vy = self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE
            # 默认喂弧度(问题16已知安全回退状态)；YAW_TEST_KP>0时喂角度，理由同上(takeoff())
            yaw_input = math.degrees(yaw) if self._yaw_test_enabled else yaw
            vyaw = self.yaw_pid.get_pid(yaw_input) * VEL_SCALE
            vx = int(self.limit(vx, 40))
            vy = int(self.limit(vy, 40))
            vyaw = int(self.limit(vyaw, 30))
        else:
            vx, vy, vyaw = 0, 0, 0

        self._step_ramp_z(target_z)
        self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))

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

            if self.target_index != self.last_target_index:
                self.last_target_index = self.target_index
                self._arrival_window.clear()
                self.arrival_confirmed_time = None
                self.arrival_start_time = time.time()

            if dx > 0.3:
                self.x_pid.reset()
            if dy > 0.3:
                self.y_pid.reset()

            frame_ok = dx < xy_thresh and dy < xy_thresh and dz < posthreshold_z and speed < ARRIVAL_VEL_THRESH
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
                self._log_file.write(json.dumps({
                    "t": round(now, 3),
                    "state": self.state,
                    "target_idx": self.target_index,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy, "vyaw": vyaw,
                    "t265_yaw_deg": round(math.degrees(yaw), 2),
                    "fc_yaw_deg": round(fc_yaw_deg, 2),
                    "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                    "of1_vel_cms": [of1_dx, of1_dy],
                    "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                    "height_setpoint_cm": round(self._ramp_z_cm, 1),
                    "of_status": [of_quality, of_link_sta, of_work_sta],
                    "pole_hover": self._pole_hovering,
                    "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                    "confirmed_poles": confirmed_poles_list,
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
        pole_str = f" | pole_dist={pole_dist:.2f}m" if pole_dist is not None else ""
        print(
            f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"| tgt=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) "
            f"| v=({vx:>3},{vy:>3}) "
            f"| send=({self.se_fc[3]:>3},{self.se_fc[4]:>3},{self.se_fc[5]:>3})"
            f"{t265_str}{pole_str}",
            end="", flush=True
        )

    # ================= 到达处理 =================
    def _on_arrival(self, target):
        if self.target_index == len(self.targets) - 2:
            pass  # 到达倒数第二个航点 (原 rgb_led 逻辑已移除)
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
            # 复现了unlock_sta连续读到0、去抖满足，但motor_pwm_mask全程非零(电机仍在出PWM)
            # 的矛盾场景，用户确认那次是人工接管才降落的。这里要求unlock_sta==0同时
            # motor_pwm_mask==0才计入确认；motor_pwm_mask为None(诊断数据不可用，比如
            # 还没收到过帧2)时不阻塞，退化成只看unlock_sta。
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
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
        with lock:
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[6] = sp_side
            self.se_fc[7] = 101
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
        self.emergency_stop = True
