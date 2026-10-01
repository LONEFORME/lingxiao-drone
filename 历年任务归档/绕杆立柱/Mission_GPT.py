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
from Lcode.circle_planner import generate_circle_waypoints, compute_detour_waypoint
from Lcode.pole_vision import azimuth_from_dx
from Lcode.resource_monitor import ResourceMonitor
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
POLE_CIRCLE_RADIUS_M = 0.7   # 环绕半径(中心到杆塔中心)。2026-07-13从0.5上调：赛题"50cm"
                              # 指飞行器与杆塔的实际净空，不是中心到中心距离；`POLE_DANGER_DIST_M`
                              # 注释记录的真机数据(0.6m中心距时桨叶到杆子实际间隙仅约20cm)反推出
                              # 机身等效半径约0.4m，若中心距仍用0.5m，实际净空只剩约8cm，
                              # 对连续转弯场景(未经真机验证的超调风险)太薄。0.7m是安全和测试
                              # 空间大小之间的折中值，不是净空恰好50cm的精确解。
POLE_CIRCLE_N_POINTS = 6     # 环绕航点数(60°一个)。弦长(0.7m半径下约0.7m)未超出已验证安全范围
                              # (已知问题15唯一站得住的结论是"扰动随步长单调增大"，无硬上限，
                              # 且问题21大范围大步长测试精度反而更好)
POLE_WORLD_MATCH_EPS_M = 0.2 # "同一根杆子"世界坐标匹配容差(用于_already_circled判断"是不是
                              # 已环绕过的那根")，跟PoleTracker.world_eps_m默认值一致
POLE_CIRCLE_EXCLUDE_MARGIN_M = 0.4  # 环绕中排除"自己正在绕的目标"时，用比POLE_WORLD_MATCH_EPS_M
                              # 更宽松的容差(环绕半径+这个余量)：环绕时飞机离杆子很近(0.5~0.9m)，
                              # yaw漂移(问题16回路默认关闭无修正)换算到近距离的世界坐标误差
                              # 可能超过0.2m，太窄的容差会让排除逻辑"认不出"自己正在绕的目标，
                              # 中途误触发悬停。赛题杆塔间距最小150cm，只要排除半径明显小于
                              # 1.5m，就不会误伤阶段2场景下真正的第二根杆子。
TOTAL_POLES = int(os.getenv("DRONE_POLE_TOTAL", "1"))  # 阶段1=1(默认)；阶段2设DRONE_POLE_TOTAL=2
POLE_DETOUR_SAFETY_RADIUS_M = POLE_DANGER_DIST_M  # 绕行安全半径，跟悬停避让触发阈值一致(2026-07-13)
POLE_DETOUR_MARGIN_M = 0.15  # 绕行航点比安全半径再往外推一点，避免刚好卡在边界抖动

POLE_COLOR_DIRECTION = {"red": "cw", "green": "ccw"}  # 赛题固定映射，见2026-07-14设计文档

# ---------- 阶段2：前置摄像头颜色识别 + 视觉伺服接近 ----------
POLE_VISION_STALE_S = 0.5     # 视觉检测结果超过此值未更新，APPROACHING视为目标丢失
POLE_TRIGGER_CONFIRM_S = 0.3  # PATROL→APPROACHING触发条件(视觉确认+颜色未去重，
                                # 2026-07-14现场测试后改为纯视觉触发，不再要求雷达同时
                                # 确认——雷达有效探测距离(~1.2m)远小于摄像头，"两者同时"
                                # 等于卡在雷达的短距离，起不到摄像头远处先看到的作用)
                                # 需要持续满足这么久才真正触发，同一个计时器也兼做颜色
                                # 锁定防抖，避免单帧误判/边界抖动被当真
APPROACH_CENTERED_DX_PX = 100  # 2026-07-14新增：APPROACHING阶段雷达坐标还没冻结时
                                # (纯视觉控制期)，杆子在画面中|dx_px|小于这个值才算"居中"，
                                # 居中前只修正y不前进(避免斜着冲向杆子)，居中后才用固定慢速
                                # 前进；1920宽画面下100px约对应2.9°方位角
                                # 2026-07-15复用：同一阈值也用于_update_trigger_candidate的
                                # PATROL→APPROACHING触发门槛(杆子必须居中才计入/继续累积
                                # POLE_TRIGGER_CONFIRM_S确认计时)，避免以后单独为"接近控制律
                                # 居中感"调这个值(比如嫌居中修正太严格想放宽)时，没意识到
                                # 同时改变了触发门槛的松紧——两处如果要分开调，需要拆成两个
                                # 独立常量。
POLE_VISION_Y_SIGN = 1        # 2026-07-15真机测试：-1(此前值)在当天3次真机测试(含2次任务级
                                # "成功")里全部观测到vy和dx_px同号(17/17、4/4、以及失败那次的
                                # 全部样本)——代入Lpid.py的标准PID公式，target=0时
                                # error=0-azimuth*(-1)=+azimuth，vy理论上必然跟azimuth(即dx_px)
                                # 同号，正是正反馈发散的数学特征，不是噪声。前两次"成功"只是雷达
                                # 接管快(3秒/<1秒)、发散没来得及积累，第三次雷达接管慢(~8秒)才让
                                # 发散充分暴露(dx_px从-39单调涨到945)。已用roll_pitch/t265_yaw_deg
                                # 排除姿态抖动和yaw漂移是根因，APPROACH_Y_KI=0排除积分饱和，
                                # 判断是符号本身该翻回+1。
                                # **2026-07-15当天第4次真机测试已验证**：翻回+1后vy/dx_px符号
                                # 反转为189条纯视觉阶段样本里188条相反(99.5%，此前是100%同号)，
                                # dx_px在10.5秒纯视觉窗口内有界震荡(80~180px)，没有再复现发散到
                                # ±950饱和值的情况——控制律确认从正反馈变成负反馈，不是又靠雷达
                                # 快速接管侥幸掩盖。震荡没有完全收敛到0，可能是Kp=30增益偏大导致
                                # 有点过冲/振铃，是调优空间不是安全问题，本次不处理。
                                # 2026-07-14曾用+1观测到发散(y到-2.83)后改成-1，但那次判断是
                                # "雷达坐标异常"这个下游症状，没有逐样本核对vy/dx_px符号关系，
                                # 可能当时的+1测试还有其他因素混杂——这次的+1已用更严谨的逐样本
                                # 符号核对方法独立验证过，不是简单类推回07-14的结论。
APPROACH_Y_KP = 30.0           # 视觉伺服y轴PID增益，误差量是方位角(弧度，量级约±0.7rad)，
                                # 不是x_pid/y_pid那种米制误差，不能沿用0.82那组增益。
                                # 初始值未经真机标定，见2026-07-14设计文档测试计划步骤1
APPROACH_Y_KI = 0.0
APPROACH_Y_KD = 2.0
APPROACH_Y_VEL_MAX = 35        # 视觉伺服y轴输出限幅(cm/s量级，跟x_pid/y_pid的40上限同量级)
APPROACH_X_SPEED_FAR = 25      # 雷达距离 > APPROACH_X_SWITCH_DIST_M 时的接近速度
APPROACH_X_SPEED_NEAR = 12     # 雷达距离 <= APPROACH_X_SWITCH_DIST_M 时降速微调
APPROACH_X_SWITCH_DIST_M = 0.8
APPROACH_CIRCLE_TRIGGER_DIST_M = POLE_CIRCLE_RADIUS_M  # APPROACHING→CIRCLING触发距离，
                                # 直接复用环绕半径(2026-07-14讨论决定，不额外加余量常量)
CAMERA_DEVICE = os.getenv("DRONE_CAMERA_DEVICE", "/dev/front_cam")  # 2026-07-14重建udev固定命名，
                                # 见/etc/udev/rules.d/99-drone-serial.rules

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
                 serial_fc_ref=None, radar_obj=None, pole_vision_obj=None):
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
        self.targets = self.load_waypoints('patrol_router.txt')
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
        self._log_lock = threading.Lock()
        self._resource_monitor = ResourceMonitor()

        # 雷达避障(可选)
        self.radar = radar_obj
        self.pole_tracker = PoleTracker(yaw_sign=POLE_YAW_SIGN) if radar_obj is not None else None
        self._last_pole_poll_time = 0.0
        self._pole_hovering = False  # 只在悬停状态切换时打日志，不是每帧刷屏
        self._hover_hold_pos = None  # 悬停期间用x_pid/y_pid锁定的位置(进入悬停那一刻的pos)
        self._hover_start_time = None  # 悬停开始时间，用于POLE_HOVER_TIMEOUT_S超时判断

        # 环绕状态机(阶段1单杆/阶段2双杆共用)
        self.nav_mode = "PATROL"  # PATROL / APPROACHING / CIRCLING / RETREAT / TO_LANDING
        self.circled_poles = []   # 已完成环绕的杆塔 [(x,y,color), ...]，坐标供绕行检查用
        self.circled_colors = set()  # 已环绕颜色集合，去重判断依据(替代坐标容差)，见2026-07-14设计文档
        self._approach_pole_center = None  # 当前正在接近/环绕的杆塔世界坐标(冻结快照)。
                                          # 2026-07-14 Task 5从_circle_pole_center改名：
                                          # APPROACHING阶段(Task 6+引入)也要用同一个字段，
                                          # 不再是CIRCLING专属
        self._approach_color = None        # 当前APPROACHING/CIRCLING锁定颜色
        self._approach_lost_since = None   # APPROACHING视觉结果开始陈旧的时间点(见POLE_VISION_STALE_S)
        self._trigger_candidate = None       # PATROL态正在累计确认时长的(x,y,color)候选目标
        self._trigger_candidate_since = None
        self._patrol_saved_targets = None
        self._patrol_saved_index = 0
        self._cruise_z = self.targets[0][2] if self.targets else put_height / 100
        self.pole_total = TOTAL_POLES
        self._detour_checked_index = -1  # 上次检查过绕行需求的target_index，避免同一个
                                          # 目标每tick重复计算/重复插入绕行航点

        # 视觉子系统(可选)
        self.pole_vision = pole_vision_obj
        self.approach_y_pid = PID(0, 0, p=APPROACH_Y_KP, i=APPROACH_Y_KI, d=APPROACH_Y_KD)

    def load_waypoints(self, path='patrol_router.txt'):
        try:
            with open(path, 'r', encoding='utf-8') as f:
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
                    logger.info(f"从{path}加载 {len(waypoints)} 个航点")
                    return waypoints
        except FileNotFoundError:
            logger.warning(f"{path} 不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取 {path} 失败: {e}，使用默认航点")

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
                    with self._log_lock:
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
        if self.nav_mode == "APPROACHING":
            self._approaching_step(pos, yaw)
            return

        # 全部航点耗尽：按当前nav_mode分支处理(环绕完成/到达降落点/巡航耗尽兜底)
        if self.target_index >= len(self.targets):
            if self.nav_mode == "CIRCLING":
                self._on_circle_complete(pos)
            elif self.nav_mode == "RETREAT":
                self._on_retreat_complete()
            elif self.nav_mode == "TO_LANDING":
                logger.info("到达降落点")
                self.state = "LAND"
            else:
                logger.info("全部航点完成")
                self.state = "LAND"
            return

        target = self.targets[self.target_index]
        target_z = int(target[2] * 100)

        # PATROL态：视觉持续确认(颜色+未去重+居中)POLE_TRIGGER_CONFIRM_S即可
        # 触发APPROACHING(2026-07-14现场测试后改为纯视觉触发，随后又补上居中
        # 约束，避免巡逻转弯时杆子从画面边缘一闪而过误触发；详见
        # _update_trigger_candidate的注释)。特意放在下面"if self.pole_tracker
        # is not None:"判断之外——雷达不再是触发条件，即使没有雷达(pole_tracker
        # 为None，比如桌面/无雷达测试场景)，视觉单独触发也必须照常生效。
        if self.nav_mode == "PATROL":
            if self._update_trigger_candidate():
                return

        # 雷达避障：检测到确认的杆子且距离过近就悬停，不绕行(除非正在主动环绕它)。
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

            # 绕行：每次切换到一个新目标点时检查一次，如果直飞会穿进某根已知杆塔
            # (排除当前正在环绕的目标，但**包含**已经环绕完成的杆塔——2026-07-13
            # 真机测试发现巡航恢复/降落路径可能会绕回已环绕过的杆塔附近，那根杆塔
            # 已经从悬停避让里永久排除了，如果直飞路径又正好穿过它，没有绕行机制的
            # 话就是纯粹的碰撞风险)的安全区，插入一个绕开的中间航点。
            # 2026-07-14全量review修复：CIRCLING阶段绕行检测整体关闭，跟悬停避让
            # 关闭是同一套设计哲学——环绕路径本身是精心规划好的轨迹，途中被绕行
            # 逻辑打断的风险比不检测更高；这里也没有排除已环绕过的杆塔，阶段1
            # 单杆场景从未触发过这条路径，阶段2才会暴露。
            if self.nav_mode != "CIRCLING" and self._maybe_insert_detour(confirmed, pos, target):
                target = self.targets[self.target_index]
                target_z = int(target[2] * 100)

            # 悬停避让距离判断：CIRCLING阶段整体关闭(2026-07-14决定，不只排除当前
            # 目标)。赛题最小杆塔间距150cm、环绕半径0.7m下，环绕到最靠近另一根杆
            # 的点最坏情况只有80cm，仅比0.75m悬停阈值高5cm，这个理论安全边际被
            # 已知定位噪声(confirmed_poles()漂移0.4~1m量级)完全吞掉，环绕过程中
            # 途悬停打断本身(近距离转弯突然停住)比不检测风险更高。PATROL/
            # APPROACHING/TO_LANDING/RETREAT阶段不受影响，仍用现有悬停避让+
            # _exclude_circle_target排除机制。
            if self.nav_mode != "CIRCLING":
                hover_check_poles = self._exclude_circle_target(confirmed)
                pole_dist = nearest_confirmed_pole_dist(hover_check_poles, pos[0], pos[1])
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
                    with self._log_lock:
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
                    with self._log_lock:
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
                with self._log_lock:
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

    # ================= 环绕状态机辅助方法 =================
    def _already_circled(self, x, y):
        # 2026-07-13真机测试发现：confirmed_poles()每次都从滑动窗口重新平均计算杆塔
        # 世界坐标，窗口滚动+新读数噪声会让同一根静止杆塔的"当前估计位置"逐渐漂移。
        # 如果用POLE_WORLD_MATCH_EPS_M(0.2m)这种给"同一物体"设计的窄容差判断"是否
        # 已环绕过"，漂移超过0.2m就会被误判成"新杆塔"，导致环绕完成后被自己刚绕完
        # 的目标重新触发悬停(时间上发生在_exclude_circle_target修复之后，是同一类
        # 问题的另一个入口)。改用跟环绕排除一致的宽容差(环绕半径+余量)。
        tolerance = POLE_CIRCLE_RADIUS_M + POLE_CIRCLE_EXCLUDE_MARGIN_M
        return any(math.hypot(x - cx, y - cy) <= tolerance
                   for cx, cy, _color in self.circled_poles)

    def _color_already_circled(self, color):
        """颜色去重判断——替代坐标容差成为"是否已环绕过"的依据(2026-07-14设计
        文档"去重机制"一节)。已被`_update_trigger_candidate`接入，作为
        PATROL→APPROACHING触发条件之一(Task 7)。`_already_circled`(坐标容差)
        继续保留给悬停避让/绕行排除使用，两者服务不同目的，见设计文档。
        """
        return color in self.circled_colors

    def _find_new_pole(self, confirmed):
        for p in confirmed:
            if not self._already_circled(p["x"], p["y"]):
                return p
        return None

    def _exclude_active_circle_target_only(self, confirmed):
        """只排除当前正在主动环绕的目标(宽容差，见POLE_CIRCLE_EXCLUDE_MARGIN_M注释)，
        不排除已环绕完成的杆塔——供绕行检测使用：已环绕过的杆塔已经从悬停避让永久
        排除了，如果直飞路径又正好穿过它，绕行是唯一还能防碰撞的机制，不能连带排除。"""
        if self._approach_pole_center is None:
            return confirmed
        cx, cy = self._approach_pole_center
        exclude_radius = POLE_CIRCLE_RADIUS_M + POLE_CIRCLE_EXCLUDE_MARGIN_M
        return [p for p in confirmed
                if math.hypot(p["x"] - cx, p["y"] - cy) > exclude_radius]

    def _exclude_circle_target(self, confirmed):
        """从悬停避让候选里排除：(1)当前正在主动环绕的目标(宽容差，见常量注释)；
        (2)所有已经环绕完成的杆塔——已经安全绕过的杆塔不再需要触发悬停避让，
        否则环绕刚完成时飞机必然还在杆塔附近，会被自己刚绕完的目标重新悬停住。"""
        result = self._exclude_active_circle_target_only(confirmed)
        return [p for p in result if not self._already_circled(p["x"], p["y"])]

    def _maybe_insert_detour(self, confirmed, pos, target):
        """每次切换到一个新目标点时检查一次(用_detour_checked_index去重，不是
        每tick都算)：如果直飞会穿进某根已知杆塔(排除当前正在环绕的目标，但包含
        已环绕完成的杆塔)的安全区，插入一个绕开的中间航点。返回True表示插入了
        航点(调用方需要重新读取target)，False表示不需要绕行。"""
        if self.target_index == self._detour_checked_index:
            return False
        self._detour_checked_index = self.target_index

        detour_poles = self._exclude_active_circle_target_only(confirmed)
        for p in detour_poles:
            detour = compute_detour_waypoint(
                p["x"], p["y"], pos[0], pos[1], target[0], target[1],
                POLE_DETOUR_SAFETY_RADIUS_M, margin=POLE_DETOUR_MARGIN_M,
            )
            if detour is not None:
                self.targets.insert(self.target_index, [detour[0], detour[1], target[2]])
                logger.warning(
                    f"航段(({pos[0]:.2f},{pos[1]:.2f})->({target[0]:.2f},{target[1]:.2f}))"
                    f"穿过杆塔({p['x']:.2f},{p['y']:.2f})安全区，插入绕行航点"
                    f"({detour[0]:.2f},{detour[1]:.2f})"
                )
                return True
        return False

    def _update_trigger_candidate(self):
        """PATROL→APPROACHING触发(2026-07-14现场测试后改为纯视觉触发)：雷达
        有效探测距离(~1.2m)远小于摄像头，要求雷达+视觉同时确认等于要求飞机先
        物理飞到很近才能触发，起不到摄像头"远处先看到"的作用。改为只看视觉
        持续确认(颜色+未去重)POLE_TRIGGER_CONFIRM_S即可触发APPROACHING；雷达
        的职责后移到_approaching_step内部，负责飞机靠近后确认世界坐标、切换
        到距离阶梯控制，以及给"其余未处理杆塔"的悬停避让托底。
        额外要求杆子在画面中居中(|dx_px| < APPROACH_CENTERED_DX_PX，同一个
        阈值也复用APPROACHING内部控制律的"居中判定"，见该常量定义处的注释)：
        纯视觉触发只看颜色时，巡逻转弯途中杆子从画面边缘一闪而过也会被计入
        确认窗口，容易打断本不该打断的巡航路线；要求先居中(大致在画面正前
        方)才开始/继续累积POLE_TRIGGER_CONFIRM_S计时，确保触发的都是飞机
        朝向已经基本对着杆子的情形。计时器语义也变了：不再是"颜色第一次
        出现"就开始计时，而是"变成居中"那一刻才开始；确认窗口内一旦偏出
        居中范围，计时立即作废重置，不能沿用之前攒的时长。
        返回True表示本次调用触发了APPROACHING(调用方应该直接return，跳过
        本tick剩余的悬停避让/日志逻辑，语义上跟原来_start_circling后直接return
        一致)。"""
        vision = self.pole_vision.latest() if self.pole_vision is not None else None
        vision_color = vision["color"] if vision else None
        vision_fresh = (vision is not None and vision["t"] > 0
                         and time.time() - vision["t"] < POLE_VISION_STALE_S)

        candidate = None
        if (vision_fresh and vision_color is not None
                and not self._color_already_circled(vision_color)
                and vision["dx_px"] is not None
                and abs(vision["dx_px"]) < APPROACH_CENTERED_DX_PX):
            candidate = vision_color

        if candidate is None:
            self._trigger_candidate = None
            self._trigger_candidate_since = None
            return False

        if self._trigger_candidate != candidate:
            self._trigger_candidate = candidate
            self._trigger_candidate_since = time.time()
            return False

        if time.time() - self._trigger_candidate_since >= POLE_TRIGGER_CONFIRM_S:
            self._start_approaching(candidate)
            self._trigger_candidate = None
            self._trigger_candidate_since = None
            return True
        return False

    def _start_approaching(self, color):
        self._patrol_saved_targets = self.targets
        self._patrol_saved_index = self.target_index
        self._approach_pole_center = None  # 雷达还没确认，等_approaching_step内部轮询到再冻结
        self._approach_color = color
        self._approach_lost_since = None
        # 跟x_pid/y_pid切换目标时的reset()同一套约定：避免PID内部
        # _last_time/_last_input/_last_error残留上一次APPROACHING episode
        # (可能是另一根杆塔)的状态，污染这次接近的第一次微分项计算。
        self.approach_y_pid.reset()
        if self.pole_vision is not None:
            self.pole_vision.set_locked_color(color)
        self.nav_mode = "APPROACHING"
        logger.warning(f"视觉确认{color}杆塔，开始视觉接近(雷达坐标待_approaching_step内确认)")

    def _log_approaching_telemetry(self, pos, yaw, vx, vy, target, pole_dist=None,
                                    confirmed_poles_list=None, dx_px=None, azimuth_rad=None,
                                    vision_fresh=None, vision_age_s=None):
        """2026-07-14新增：_approaching_step原本完全不写飞行日志(navigate()一进
        APPROACHING分支就return，跳过了navigate()自己的日志写入代码块)，导致
        APPROACHING阶段(纯视觉接近+雷达接管后的距离阶梯控制)全程没有位置/速度
        数据可查——真机测试观察到"没有正对着杆子飞、靠近后来回震荡"这类现象时，
        完全没有数据能验证/排查，只能事后靠回忆。这里补一份跟navigate()同结构
        (但简化掉of1/roll_pitch等navigate()专属字段)的日志写入。
        2026-07-15补充：原版本只记了PID算出来的指令vx/vy，没记视觉伺服的原始
        输入(dx_px/azimuth)和T265实测速度(t265_vel)/飞控yaw(fc_yaw_deg)交叉校验，
        导致没法区分"震荡"是视觉检测本身在跳还是控制律过冲——这里补上。
        2026-07-15再补充：真机测试观察到dx_px在APPROACHING阶段剧烈跳变但
        t265_yaw_deg平稳、fc_yaw_deg异常(超出±180°范围)，用户目视确认飞机
        没有明显yaw旋转，但roll/pitch有肉眼可见的抖动——之前"简化掉
        roll_pitch"的假设(APPROACHING阶段不需要姿态数据)被推翻，这里补上，
        跟navigate()主日志块的roll_pitch同源(re_fc[1]/re_fc[2])。"""
        now = time.time()
        if not (self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL):
            return
        tv = self.realsense.get_velocity() if (self.t265_ok and self.realsense) else (0.0, 0.0, 0.0)
        with lock:
            fc_yaw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else 0.0
            roll_deg = self.re_fc[1] / 100.0 if len(self.re_fc) > 1 else 0.0
            pitch_deg = self.re_fc[2] / 100.0 if len(self.re_fc) > 2 else 0.0
        try:
            with self._log_lock:
                self._log_file.write(json.dumps({
                    "t": round(now, 3),
                    "state": self.state,
                    "nav_mode": self.nav_mode,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy,
                    "t265_yaw_deg": round(math.degrees(yaw), 2),
                    "fc_yaw_deg": round(fc_yaw_deg, 2),
                    "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                    "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                    "dx_px": dx_px,
                    "azimuth_deg": round(math.degrees(azimuth_rad), 2) if azimuth_rad is not None else None,
                    "vision_fresh": vision_fresh,
                    "vision_age_s": round(vision_age_s, 3) if vision_age_s is not None else None,
                    "pole_hover": self._pole_hovering,
                    "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                    "confirmed_poles": confirmed_poles_list or [],
                }) + "\n")
                self._log_file.flush()
        except Exception:
            pass
        self._last_log_time = now

    def _approaching_step(self, pos, yaw):
        # 雷达轮询+悬停避让(2026-07-14全量review发现的缺口修复)：原实现直接
        # 进入下面的接近控制逻辑，完全跳过了navigate()里其他nav_mode都会走的
        # 雷达轮询/悬停避让代码块，设计文档承诺的"APPROACHING阶段其余未处理
        # 杆塔仍触发悬停避让"从未生效。这里补一份跟navigate()同构的悬停避让
        # 逻辑(排除当前接近目标+已环绕杆塔，复用_exclude_circle_target，语义
        # 与PATROL/TO_LANDING/RETREAT一致)。
        pole_hover = False
        pole_dist = None
        confirmed_poles_list = []
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

            # 雷达坐标尚未冻结时尝试从这次轮询里确认出来(2026-07-14改为纯视觉
            # 触发APPROACHING后，雷达坐标不再是触发时就有的，要在这里补上)。
            # 必须放在下面悬停避让判断之前：否则还没冻结时，自己正在接近的这根
            # 杆塔会被悬停避让判断误当成"其余未处理的杆塔"，把自己悬停住。
            if self._approach_pole_center is None:
                new_pole = self._find_new_pole(confirmed)
                if new_pole is not None:
                    self._approach_pole_center = (new_pole["x"], new_pole["y"])
                    logger.warning(
                        f"APPROACHING阶段雷达确认坐标"
                        f"({new_pole['x']:.2f},{new_pole['y']:.2f})，切换到距离阶梯控制"
                    )

            hover_check_poles = self._exclude_circle_target(confirmed)
            pole_dist = nearest_confirmed_pole_dist(hover_check_poles, pos[0], pos[1])
            if self._pole_hovering:
                pole_hover = pole_dist is not None and pole_dist < POLE_RESUME_DIST_M
            else:
                pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M

        if pole_hover:
            if not self._pole_hovering:
                logger.warning(f"APPROACHING阶段检测到其他杆子距离{pole_dist:.2f}m，悬停等待")
                self._pole_hovering = True
                self._hover_hold_pos = (pos[0], pos[1])
                self._hover_start_time = time.time()
                self.x_pid.set_target(self._hover_hold_pos[0])
                self.y_pid.set_target(self._hover_hold_pos[1])
            elif time.time() - self._hover_start_time >= POLE_HOVER_TIMEOUT_S:
                logger.warning(f"APPROACHING阶段悬停超过{POLE_HOVER_TIMEOUT_S:.0f}秒仍未恢复，原地触发降落")
                self._pole_hovering = False
                self._hover_hold_pos = None
                self._hover_start_time = None
                self.state = "LAND"
                return
            vx = int(self.limit(self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE, 40))
            vy = int(self.limit(self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE, 40))
            # 高度也一并冻结(不调用_step_ramp_z)，跟navigate()的悬停分支同构：
            # 悬停避让的语义是"停一切、原地保持"，2026-07-14 code review发现这里
            # 原本多了一行_step_ramp_z(cruise_z)会让高度在悬停期间继续爬升到
            # 巡航高度，跟navigate()真正的"位置+高度都冻结"行为不一致，已去掉。
            self._log_approaching_telemetry(
                pos, yaw, vx, vy,
                target=(self._hover_hold_pos[0], self._hover_hold_pos[1], self._cruise_z),
                pole_dist=pole_dist, confirmed_poles_list=confirmed_poles_list,
            )
            self.set_speed(vx, vy, 0, int(self._ramp_z_cm))
            return
        elif self._pole_hovering:
            logger.info("APPROACHING阶段杆子确认已消失，恢复接近")
            self._pole_hovering = False
            self._hover_hold_pos = None
            self._hover_start_time = None

        vision = self.pole_vision.latest() if self.pole_vision is not None else None
        vision_fresh = (vision is not None and vision["t"] > 0
                         and time.time() - vision["t"] < POLE_VISION_STALE_S)

        if not vision_fresh:
            if self._approach_lost_since is None:
                self._approach_lost_since = time.time()
            elif time.time() - self._approach_lost_since >= POLE_VISION_STALE_S:
                logger.warning("APPROACHING视觉目标丢失，退回PATROL")
                self._abort_approaching()
                return
        else:
            self._approach_lost_since = None

        if self._approach_pole_center is not None:
            cx, cy = self._approach_pole_center
            dist = math.hypot(pos[0] - cx, pos[1] - cy)

            if dist <= APPROACH_CIRCLE_TRIGGER_DIST_M:
                self._start_circling_from_approach(pos)
                return

            x_speed = APPROACH_X_SPEED_FAR if dist > APPROACH_X_SWITCH_DIST_M else APPROACH_X_SPEED_NEAR
            vx = int(x_speed if cx >= pos[0] else -x_speed)
        else:
            # 雷达坐标还没冻结：纯视觉控制(2026-07-14决定)——杆子在画面中居中
            # (|dx_px|<APPROACH_CENTERED_DX_PX)前只修正y不前进，避免斜着冲向
            # 杆子；居中后才用固定慢速朝摄像头朝向(世界+x)前进。
            if vision_fresh and vision["dx_px"] is not None and abs(vision["dx_px"]) < APPROACH_CENTERED_DX_PX:
                vx = APPROACH_X_SPEED_NEAR
            else:
                vx = 0

        vy = 0
        azimuth = None
        if vision_fresh and vision["dx_px"] is not None:
            azimuth = azimuth_from_dx(vision["dx_px"])
            self.approach_y_pid.set_target(0.0)
            vy_raw = self.approach_y_pid.get_pid(azimuth * POLE_VISION_Y_SIGN)
            vy = int(self.limit(vy_raw, APPROACH_Y_VEL_MAX))

        self._step_ramp_z(int(self._cruise_z * 100))
        log_target = ((self._approach_pole_center[0], self._approach_pole_center[1], self._cruise_z)
                      if self._approach_pole_center is not None else (pos[0], pos[1], self._cruise_z))
        self._log_approaching_telemetry(
            pos, yaw, vx, vy, target=log_target,
            pole_dist=pole_dist, confirmed_poles_list=confirmed_poles_list,
            dx_px=vision["dx_px"] if vision is not None else None,
            azimuth_rad=azimuth,
            vision_fresh=vision_fresh,
            vision_age_s=(time.time() - vision["t"]) if vision is not None and vision["t"] > 0 else None,
        )
        self.set_speed(vx, vy, 0, int(self._ramp_z_cm))

    def _abort_approaching(self):
        if self.pole_vision is not None:
            self.pole_vision.set_locked_color(None)
        self.targets = self._patrol_saved_targets
        self.target_index = self._patrol_saved_index
        self.last_target_index = -1
        self._approach_pole_center = None
        self._approach_color = None
        self._approach_lost_since = None
        self.nav_mode = "PATROL"

    def _start_circling_from_approach(self, pos):
        cx, cy = self._approach_pole_center
        direction = POLE_COLOR_DIRECTION[self._approach_color]
        waypoints = generate_circle_waypoints(
            cx, cy, pos[0], pos[1],
            radius=POLE_CIRCLE_RADIUS_M, n_points=POLE_CIRCLE_N_POINTS,
            direction=direction, z=self._cruise_z,
        )
        self.targets = waypoints
        self.target_index = 0
        self.last_target_index = -1
        self._detour_checked_index = -1
        self.nav_mode = "CIRCLING"
        logger.warning(
            f"进入环绕：{self._approach_color}杆塔({cx:.2f},{cy:.2f})，"
            f"方向{direction}，{len(waypoints)}个航点"
        )

    def _on_circle_complete(self, pos):
        cx, cy = self._approach_pole_center
        color = self._approach_color
        logger.info(f"{color}杆塔({cx:.2f},{cy:.2f})环绕完成")
        self.circled_poles.append((cx, cy, color))
        self.circled_colors.add(color)
        self._approach_pole_center = None
        self._approach_color = None
        if self.pole_vision is not None:
            self.pole_vision.set_locked_color(None)
        # 先退回x=0基准线，再决定继续巡航还是转向降落(见2026-07-14设计文档)
        self.targets = [[0.0, pos[1], self._cruise_z]]
        self.target_index = 0
        self.last_target_index = -1
        self.nav_mode = "RETREAT"

    def _on_retreat_complete(self):
        if len(self.circled_colors) >= self.pole_total:
            logger.info(f"已绕完全部{self.pole_total}种颜色，前往降落点")
            self.targets = self.load_waypoints('landing_router.txt')
            self.target_index = 0
            self._detour_checked_index = -1
            self.nav_mode = "TO_LANDING"
        else:
            self.targets = self._patrol_saved_targets
            self.target_index = self._patrol_saved_index
            self.nav_mode = "PATROL"
        self.last_target_index = -1

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
