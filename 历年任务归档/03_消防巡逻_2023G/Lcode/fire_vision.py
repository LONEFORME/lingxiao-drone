"""下视摄像头红色火源检测。见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md
"覆盖巡逻路径"/"APPROACH"一节。

2026-07-16确认：下视摄像头硬件是IMX219 CSI摄像头(ubuntu-pi/RDK X5, /dev/video10)，
不是标准USB摄像头——官方文档明确警告不能直接`cv2.VideoCapture(device)`("当前 RDK
驱动在通用 OpenCV/V4L2 取流路径中可能超时")，必须通过板载专用封装取流。改用
`Lcode/rdk_imx219_jupyter_preview.VisionSystem`（该模块内部持有自己的后台采集
线程+V4L2 MMAP专用C生产程序，从ubuntu-pi板子`Desktop/IMX219/`复制进本仓库纳入
版本控制，避免依赖仓库外未跟踪路径——那个目录还有一份不完整的旧版本，见
project_imx219_camera_bringup记忆，不要弄混）。

`detect_fire()`/`SmoothedFireDetector`是纯函数/纯逻辑，跟摄像头取流方式无关，
不受这次硬件方案调整影响。
"""
import os
import threading
import time
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np

from Lcode.Logger import logger
from Lcode.rdk_imx219_jupyter_preview import VisionSystem

IMX219_DEVICE = "/dev/video10"  # VSE缩放输出节点，官方文档推荐的实时预览/取流节点
IMX219_WIDTH = 960
IMX219_HEIGHT = 540
# 2026-07-16台架测试用值(文档"综合推荐"档)，画面偏暗噪点重是已知问题
# (见project_imx219_camera_bringup记忆)——2026-07-17真机测试发现2645在实际飞行
# 高度下反而普遍过曝(meanY~233，接近纯白)，跟台架近距离测试时偏暗的印象相反，
# 地面按更接近实际取景距离重新扫描曝光值(200~600区间meanY 66~180)后，500量级
# 亮度更均衡(不过曝也不丢暗部细节)。降到500。
# 注意：地面近距离测试只能验证亮度，验证不了圆度检测——近距离能看清灯罩锡纸
# 反光罩的褶皱细节，形状天然不规则(圆度0.03~0.25)，跟飞行高度下被距离/糊化
# 压缩成的小圆斑视觉效果完全不同，圆度阈值(MIN_CIRCULARITY)是否合适仍需要
# 下次真机测试验证。
# 2026-07-17简化悬停测试(灯罩放在旁边)复测：500在当天的光照条件下仍然普遍
# 过曝(多张快照接近纯白)，只有偶尔几帧曝光正常。按地面曝光扫描数据(300对应
# meanY≈115，比500更居中)进一步降到300。
IMX219_EXPOSURE = 300
IMX219_VERTICAL_BLANKING = 2446

# 火源面积范围：灯罩高度不超过10cm、俯视为近似圆形光斑。上限用于排除大面积
# 反光/其他红色物体误触发(见设计文档审查发现1"误触发风险不可逆")，具体像素
# 数值需现场标定(取决于飞行高度/摄像头视场角)，这里给经验初始值。
# 2026-07-16台架测试(bench_test_snapshot.png/2)实测：室内测试距离下红色光晕连通域
# 面积298832/129534px，原50000的上限在室内可达到的任何测试距离下都会被撑爆——
# 按距离平方反比外推，要降到50000以内大约需要再拉远1.6倍距离，室内空间达不到。
# 上限本意是挡"占满大半画面"级别的明显异常反光，不是精确卡目标尺寸，因此放宽到
# 200000(能通过这两次台架数据，仍能挡住293万像素总画幅的~40%以上占比这种极端情况)。
# 真实18dm飞行高度下光源占比会小得多，这个上限届时基本不会被触及，留待真机测试验证。
MIN_FIRE_AREA_PX = 200
MAX_FIRE_AREA_PX = 200000

# 圆度过滤：4*pi*面积/周长^2，完美圆形=1.0，越扁/越不规则越接近0。真实火源
# 灯罩俯视是近似圆形光斑，用这个过滤能排除面积恰好落在范围内、但形状是矩形/
# 长条形的干扰物(比如小车车身反光)——2026-07-16真机测试实测触发了一次误检测，
# 检测到的目标后来确认是"起点边上小车启动的红色矩形"，不是真实火源，面积上限
# 挡不住(矩形恰好在合理面积范围内)，需要额外靠形状区分。
# 2026-07-17从0.5上调到0.7：真机测试复现了另一种绕过方式——下视摄像头视场里当时
# 有一条固定遮挡物(疑似脚架腿，已现场挪开)把一块红色矩形地标切成两个连通域，
# 完整矩形圆度0.326被正确挡住，但被切碎后较小的那块碎片圆度0.639混过了0.5的
# 阈值，触发误判。0.7当时能挡住这类碎片。
# 2026-07-17当天晚些时候又从0.7降回0.45：短程+1m靠近灯罩复测，3帧灯罩完整/
# 部分入镜的画面圆度只有0.535/0.598/0.654，全部被0.7挡在门外——近距离下摄像头
# 分辨出灯罩真实外壳形状(偏方形PCB支架+螺丝)，不再是远距离/模糊糊成的近圆
# 光斑，0.7是照着远距离场景(唯一确认命中圆度0.790)标定的，换到近距离必然失效。
# 真实火源圆度样本(0.535~0.790)跟问题30那次碎片误判圆度(0.639~0.651)是重叠的
# ——圆度这一个特征已经不能同时满足"近距离也要能识别"和"挡住那次碎片"两个
# 要求，两者不可兼得。用户明确要求"能成功识别即可"，选择优先保证识别，把
# 圆度降到0.45(比最低的真实样本0.535留了余量)；问题30那次误判的防线改为
# 依赖"遮挡物已从现场物理挪开"这个根本修复，不再靠圆度做双重保险——如果
# 那类固定遮挡物未来又出现在视场里，碎片有重新触发误判的风险，见known_issues.md。
MIN_CIRCULARITY = 0.45

# 红色HSV阈值，色相环绕0/180两段取并集。经验初始值，现场需按实际LED光源标定
# (参照circle_pole真机标定红杆子的经验：偏暗/偏亮光源饱和度差异很大)。
HSV_RANGES_RED = [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))]

# 2026-07-17新增：真机测试实测发现，灯罩强光在画面里经常呈现"星芒/爆闪"形态
# (高光衍射产生多条细尖刺，不是一个实心圆盘)，导致轮廓圆度只有0.068，远低于
# MIN_CIRCULARITY——不是曝光或距离问题，无论怎么调曝光，只要高光是星芒状，
# 圆度永远上不去。用闭运算(先膨胀再腐蚀)把尖刺之间的小缝隙填平、拼成一个
# 接近实心圆盘的轮廓再算圆度：真机实测能把这类星芒灯罩的圆度从0.068拉到0.76，
# 通过0.7阈值。经验证核大小15不会影响已有的防误判能力——纯矩形/圆形测试
# 用例结果不变(闭运算只填补小缝隙，对已经连续无缝隙的实心形状没有任何影响)；
# 问题30记录的"矩形被遮挡切碎"真实图片，碎片圆度0.639只被抬高到0.651，仍然
# 低于0.7阈值，不会重新引入那次误触发。
FIRE_MASK_CLOSE_KERNEL_SIZE = 15


def detect_fire(frame_bgr, min_area: int = MIN_FIRE_AREA_PX,
                 max_area: int = MAX_FIRE_AREA_PX,
                 min_circularity: float = MIN_CIRCULARITY) -> Optional[Tuple[float, float]]:
    """在BGR图像里找面积在[min_area, max_area]范围内、形状接近圆形(圆度>=
    min_circularity)的最大红色连通域，返回(dx_px, dy_px) = 质心像素坐标 - 画面
    中心，没找到时返回None。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    height, width = frame_bgr.shape[:2]

    mask = None
    for lower, upper in HSV_RANGES_RED:
        m = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = m if mask is None else (mask | m)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (FIRE_MASK_CLOSE_KERNEL_SIZE, FIRE_MASK_CLOSE_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_area = 0
    best_cx, best_cy = None, None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter <= 0:
            continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < min_circularity:
            continue
        if area > best_area:
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            best_area = area
            best_cx = moments["m10"] / moments["m00"]
            best_cy = moments["m01"] / moments["m00"]

    if best_cx is None:
        return None
    return best_cx - width / 2.0, best_cy - height / 2.0


class SmoothedFireDetector:
    """对detect_fire()逐帧结果做滑动平均，减少单帧噪声导致APPROACH阶段误修正
    (见设计文档"独立的、更保守的伺服增益"一节)。任意一帧丢失目标(None)时清空
    窗口重新积累，不能用陈旧偏移量继续参与平均。"""

    def __init__(self, window: int = 5):
        self.window = window
        self._buf = deque(maxlen=window)

    def update(self, detection: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if detection is None:
            self._buf.clear()
            return None
        self._buf.append(detection)
        if len(self._buf) < self.window:
            return None
        avg_dx = sum(d[0] for d in self._buf) / len(self._buf)
        avg_dy = sum(d[1] for d in self._buf) / len(self._buf)
        return avg_dx, avg_dy


class FireVision:
    """后台线程持续从IMX219取帧+红色检测+滑动平均，主循环每tick只读`latest()`
    共享的最新结果，不阻塞30ms主循环(风格与Serial_fc.listen_fc()一致)。

    底层`VisionSystem`已经有自己的后台采集线程和帧缓存，`get_frame()`是非阻塞的
    (立即返回缓存的最新帧或None)——本类自己的`_loop()`只是按固定节奏轮询它、
    跑detect_fire()，不需要也不应该假设`get_frame()`会阻塞到下一帧就绪
    (那是cv2.VideoCapture.read()的行为，VisionSystem不是这样)。

    摄像头打不开时`start()`返回False，`latest()`永远返回全None——PATROL态"检测到
    火情"条件永远不满足，等同于纯巡逻场景，不会crash也不会阻塞主循环(见设计文档
    审查发现2"检测线程健康检查缺失"，此处只保证不crash，心跳超时告警留待真机
    测试阶段)。`VisionSystem.open()`打不开摄像头时会抛`RuntimeError`(不是像
    `cv2.VideoCapture.isOpened()`那样返回False)，这里用try/except吞掉转换成
    同样的"返回False"约定，维持对`main.py`调用方透明。
    """

    def __init__(self, device: str = IMX219_DEVICE, smooth_window: int = 5,
                 debug_dir: str = None):
        self.device = device
        self._lock = threading.Lock()
        self._latest = {"dx_px": None, "dy_px": None, "t": 0.0}
        self._latest_frame = None  # 最新一帧原始BGR画面，供save_snapshot()存档用
        self._running = False
        self._thread = None
        self._vision = None
        self._smoother = SmoothedFireDetector(window=smooth_window)
        # 2026-07-16新增：火情触发时存一张现场画面方便事后调试分析。不像
        # circle_pole/Lcode/pole_vision.py那样持续存debug帧(那边默认要env var
        # 显式开启)——fire_patrol因为fire_triggered全程只触发一次，存图频率
        # 天然很低，默认就开(不需要环境变量才启用)，目录仍可用环境变量覆盖。
        self.debug_dir = debug_dir or os.getenv("DRONE_FIRE_DEBUG_DIR", "fire_debug")

    def start(self) -> bool:
        self._vision = VisionSystem(
            device=self.device,
            raw_width=IMX219_WIDTH,
            raw_height=IMX219_HEIGHT,
            display_width=IMX219_WIDTH,
            display_height=IMX219_HEIGHT,
            exposure=IMX219_EXPOSURE,
            analogue_gain=0,
            vertical_blanking=IMX219_VERTICAL_BLANKING,
            blue_gain=0.95,
            green_gain=1.06,
            red_gain=1.00,
            saturation=1.08,
            enable_bright_detection=False,  # 用自己的detect_fire()，不用模块自带的亮点检测
            capture_thread_daemon=True,     # 跟随主进程退出，不强制要求显式release()
            auto_open=False,
        )
        try:
            self._vision.open()
        except Exception as e:
            logger.error(f"下视摄像头(IMX219, {self.device})打不开: {e}，火情检测禁用")
            self._vision = None
            return False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        # 2026-07-17修复：真机测试(main.py退出时)实测到segfault——self._loop()
        # 每30ms轮询一次self._vision.get_frame()，之前这里只置位_running就立刻
        # release()，_loop()线程可能还在执行get_frame()的过程中，跟release()
        # 拆卸底层V4L2采集子进程/共享内存产生竞态(release()内部会terminate采集
        # 子进程、关闭管道)。跟ResourceMonitor.stop()一样的模式：先join让轮询
        # 线程真正退出，确认它不会再碰self._vision，才安全地release()。
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._vision is not None:
            self._vision.release()

    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def save_snapshot(self, reason: str = "fire_triggered") -> Optional[str]:
        """把最新一帧原始画面存到self.debug_dir，文件名带时间戳+reason，
        方便事后对照当时到底看到了什么(比如误检测复盘)。没有可用帧或写入失败
        时返回None，不抛异常——调用方(火情触发时)不应该因为存图失败被打断。"""
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
        if frame is None:
            logger.warning("save_snapshot: 当前没有可用帧，跳过存图")
            return None
        try:
            os.makedirs(self.debug_dir, exist_ok=True)
            path = os.path.join(
                self.debug_dir,
                f"{time.strftime('%Y%m%d_%H%M%S')}_{reason}.jpg",
            )
            cv2.imwrite(path, frame)
            logger.info(f"save_snapshot: 已存图 {path}")
            return path
        except Exception as e:
            logger.error(f"save_snapshot 失败: {e}")
            return None

    def _loop(self):
        while self._running:
            frame = self._vision.get_frame()
            if frame is not None:
                raw = detect_fire(frame)
                smoothed = self._smoother.update(raw)
                with self._lock:
                    self._latest_frame = frame
                    if smoothed is None:
                        self._latest = {"dx_px": None, "dy_px": None, "t": time.time()}
                    else:
                        self._latest = {"dx_px": smoothed[0], "dy_px": smoothed[1], "t": time.time()}
            time.sleep(0.03)
