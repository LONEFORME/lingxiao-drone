"""N10P 激光雷达（镭神智能）串口驱动。

协议参考：《镭神智能_N10 Plus_通讯协议_V1.0.0_20220926.pdf》
帧格式（108字节，大端）：
  Byte0-1   帧头 A5 5A
  Byte2     Length，整帧长度(含帧头到校验)，正常输出固定 0x6C=108
  Byte3-4   Speed_H/L，一个码盘周期时间(us)，转速Hz = 1e6/(值*24)  [注：协议原文按24点/圈换算]
  Byte5-6   Start_angle_H/L，起始角度*100
  Byte7-102 32组点(每组3字节：距离H/L(mm,大端) + 强度)
  Byte103-104 预留位
  Byte105-106 Stop_angle_H/L，结束角度*100
  Byte107   校验：byte0..byte106 字节和 & 0xFF
"""
import math
import serial
import threading
import time
from collections import deque
from Lcode.Logger import logger
from Lcode.global_variable import lock

FRAME_LEN = 108        # 整帧长度（含帧头）
BODY_LEN = FRAME_LEN - 3  # 去掉 A5 5A Length 之后剩余字节数
POINTS_PER_FRAME = 32

# 雷达角度 -> 机体坐标系映射（2026-07-07 现场核对当前安装位置得出，不是手册通用值，
# 换了安装位置/朝向需要重新核对）：0度对应机体 +X 方向，90度对应机体 -Y 方向。
def radar_angle_to_body_xy(angle_deg, distance_mm):
    """雷达(角度,距离mm) -> 机体坐标系(x_m, y_m)"""
    rad = math.radians(angle_deg)
    distance_m = distance_mm / 1000.0
    x = distance_m * math.cos(rad)
    y = -distance_m * math.sin(rad)
    return x, y


def body_to_world_xy(drone_x_m, drone_y_m, yaw_rad, bx_m, by_m, yaw_sign=1):
    """机体系坐标(bx_m, by_m) -> 世界系坐标(drone_x_m, drone_y_m 为飞机当前世界系位置)。

    yaw_rad 约定：跟 t265.py 的 get_orientation()[2] / pose_data[5] 同一个量，
    符号尚未标定（t265.py 内部经过轴重映射+取反+欧拉角提取，不是标准数学CCW正角度）。
    yaw_sign 用于以后真机标定时切换旋转方向，本次实现先固定不管调用方传几都能跑，
    具体该用 +1 还是 -1 留给下次真机/台架测试确定。
    """
    yaw = yaw_sign * yaw_rad
    cy, sy = math.cos(yaw), math.sin(yaw)
    wx = drone_x_m + bx_m * cy - by_m * sy
    wy = drone_y_m + bx_m * sy + by_m * cy
    return wx, wy


def world_to_body_angle_dist(world_x, world_y, drone_x_m, drone_y_m, yaw_rad, yaw_sign=1):
    """body_to_world_xy 的逆变换：已知一个世界系点和飞机当前位姿，反推该点在机体系
    的雷达(角度,距离mm)读数——只在离线合成测试数据时用，不是雷达驱动本身需要的功能。
    """
    dx = world_x - drone_x_m
    dy = world_y - drone_y_m
    yaw = yaw_sign * yaw_rad
    cy, sy = math.cos(yaw), math.sin(yaw)
    bx = dx * cy + dy * sy
    by = -dx * sy + dy * cy
    angle_deg = math.degrees(math.atan2(-by, bx)) % 360.0
    distance_mm = math.hypot(bx, by) * 1000.0
    return angle_deg, distance_mm


def _cluster_points(points, eps_m=0.15, min_samples=3):
    """按角度顺序对点聚类（仿镭神官方SLAM包 obstacle_detector.py 的 simple_clustering 思路）。

    points: [(angle_deg, distance_mm), ...]，需按角度升序排列，覆盖0~359。
    相邻两点的机体坐标系直线距离 < eps_m 则归为同一聚类；聚类点数 < min_samples 的整体丢弃
    （孤立的单点大概率是噪声/边缘效应导致的幻影，不是真实障碍物——这是2026-07-07台架测试
    里"细杆子命中率低、常出现孤立幽灵点"这个问题的应对方案）。
    首尾（359°与0°）相邻做环绕合并。
    """
    if not points:
        return []
    clusters = []
    current = [points[0]]
    for i in range(1, len(points)):
        x1, y1 = radar_angle_to_body_xy(*points[i - 1])
        x2, y2 = radar_angle_to_body_xy(*points[i])
        if math.hypot(x2 - x1, y2 - y1) < eps_m:
            current.append(points[i])
        else:
            if len(current) >= min_samples:
                clusters.append(current)
            current = [points[i]]
    if len(current) >= min_samples:
        clusters.append(current)

    if len(clusters) >= 2:
        x1, y1 = radar_angle_to_body_xy(*clusters[-1][-1])
        x2, y2 = radar_angle_to_body_xy(*clusters[0][0])
        if math.hypot(x2 - x1, y2 - y1) < eps_m:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()
    return clusters


class Serial_radar(object):
    def __init__(self, port, baudrate=460800):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1.0, write_timeout=1.0)
        self.listen_running = False
        # 最近一圈的点云缓存：{angle_deg(0~360, float): (distance_mm, intensity)}
        # 用字典按角度覆盖，避免转速抖动导致列表无限增长
        self._scan = {}
        self._last_frame_time = 0.0

    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("雷达串口状态：%s", self.ser.is_open)

    def listen_start(self):
        self.listen_running = True
        t = threading.Thread(target=self._listen_loop)
        t.daemon = True
        t.start()
        logger.info("雷达串口监听线程启动")

    def listen_end(self):
        self.listen_running = False
        logger.info("雷达串口监听线程关闭")

    def _listen_loop(self):
        while self.listen_running:
            b = self.ser.read(1)
            if len(b) == 0 or b[0] != 0xA5:
                continue
            b2 = self.ser.read(1)
            if len(b2) == 0 or b2[0] != 0x5A:
                continue
            length_byte = self.ser.read(1)
            if len(length_byte) == 0:
                continue
            length = length_byte[0]
            if length != FRAME_LEN:
                # 长度异常，跳过这帧，不强行按固定长度硬读，避免和下一帧错位
                continue
            body = self.ser.read(BODY_LEN)
            if len(body) < BODY_LEN:
                continue

            header = bytes([0xA5, 0x5A, length])
            checksum_calc = (sum(header) + sum(body[:-1])) & 0xFF
            checksum_recv = body[-1]
            if checksum_calc != checksum_recv:
                continue

            self._parse_frame(body)
            self._last_frame_time = time.time()

    def _parse_frame(self, body):
        # body 是去掉 A5 5A Length 后的 105 字节(对应协议 Byte3~Byte107)：
        # body[0:2]=speed  body[2:4]=start_angle  body[4:100]=32点(3字节/点)
        # body[100:102]=预留位  body[102:104]=stop_angle  body[104]=crc
        start_angle = ((body[2] << 8) | body[3]) / 100.0
        stop_angle = ((body[102] << 8) | body[103]) / 100.0

        span = stop_angle - start_angle
        if span < 0:
            span += 360.0
        step = span / (POINTS_PER_FRAME - 1) if POINTS_PER_FRAME > 1 else 0.0

        points = []
        offset = 4  # 跳过 speed(2) + start_angle(2)
        for i in range(POINTS_PER_FRAME):
            dist_h = body[offset]
            dist_l = body[offset + 1]
            intensity = body[offset + 2]
            distance_mm = (dist_h << 8) | dist_l
            angle = (start_angle + step * i) % 360.0
            points.append((angle, distance_mm, intensity))
            offset += 3

        with lock:
            for angle, distance_mm, intensity in points:
                self._scan[round(angle)] = (distance_mm, intensity)

    def get_scan(self):
        """返回最近一圈的点云快照：{angle_deg(int,0~359): (distance_mm, intensity)}"""
        with lock:
            return dict(self._scan)

    def get_nearest(self, min_intensity=0):
        """返回当前缓存点云里距离最近的点 (angle_deg, distance_mm)，无有效点返回 None。"""
        with lock:
            scan = list(self._scan.items())
        best = None
        for angle, (distance_mm, intensity) in scan:
            if distance_mm <= 0 or intensity < min_intensity:
                continue
            if best is None or distance_mm < best[1]:
                best = (angle, distance_mm)
        return best

    def get_nearest_xy(self, min_intensity=0):
        """返回当前缓存点云里距离最近的点，换算成机体坐标系(x_m, y_m)；无有效点返回 None。"""
        nearest = self.get_nearest(min_intensity)
        if nearest is None:
            return None
        angle, distance_mm = nearest
        return radar_angle_to_body_xy(angle, distance_mm)

    def get_obstacles(self, eps_m=0.15, min_samples=3, min_intensity=80):
        """对当前扫描做角度维度聚类，过滤孤立噪声点，返回障碍物列表：
        [{'angle_deg', 'distance_mm', 'x', 'y', 'num_points'}, ...]，按距离升序。"""
        with lock:
            scan = dict(self._scan)
        points = []
        for angle in sorted(scan.keys()):
            distance_mm, intensity = scan[angle]
            if distance_mm <= 0 or intensity < min_intensity:
                continue
            points.append((angle, distance_mm))

        clusters = _cluster_points(points, eps_m, min_samples)
        obstacles = []
        for cluster in clusters:
            angle, distance_mm = min(cluster, key=lambda p: p[1])
            x, y = radar_angle_to_body_xy(angle, distance_mm)
            obstacles.append({
                "angle_deg": angle,
                "distance_mm": distance_mm,
                "x": x,
                "y": y,
                "num_points": len(cluster),
            })
        obstacles.sort(key=lambda o: o["distance_mm"])
        return obstacles

    def get_nearest_obstacle(self, eps_m=0.15, min_samples=3, min_intensity=80):
        """聚类过滤后的最近障碍物，无有效聚类返回 None。"""
        obstacles = self.get_obstacles(eps_m, min_samples, min_intensity)
        return obstacles[0] if obstacles else None

    def is_alive(self, timeout=1.0):
        return (time.time() - self._last_frame_time) < timeout

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("雷达串口已关闭")


class PoleTracker(object):
    """细杆子(绕障目标)探测器 — 空间聚类 + 多帧时间持续性组合，世界系匹配。

    背景(2026-07-07台架测试结论)：细杆子单帧扫描通常只产生1个孤立点，跟真正的噪声在
    空间特征上无法区分，`Serial_radar.get_obstacles()`(min_samples>=2起)会把它和噪声一起
    过滤掉。真正能把两者分开的是时间维度——噪声不会重复出现在同一角度附近，杆子会。

    2026-07-07真机验证发现：早期版本用机体系角度/距离容差匹配历史候选，飞机接近一个不严格
    在正前方的目标时，视线方位角会随飞机位置摆动(实测19°)，远超固定容差，导致匹配失败——
    这是纯几何效应，跟传感器/算法实现无关。本版本改成把每次轮询的候选点转换到世界系坐标
    (需要调用方传入飞机当前位置+朝向)，历史匹配比较世界系距离而不是机体系角度/距离，静止的
    杆子在世界系里位置不变，不受飞机自身运动影响。

    用法：只关心 max_range_mm 以内的目标(默认1.2m，留一点余量，实际工作距离约1m)，导航/
    搜寻循环里周期性调用 update(radar, x_m, y_m, yaw_rad)，累计到 min_hits 次命中就能从
    confirmed_poles() 里读到确认的杆子(世界系坐标)。命中率参考2026-07-07实测：70cm~90%，
    1m~70%，1.55m~10%——在1m以内工作，min_hits=3 通常几次调用内就能确认。
    """
    def __init__(self, max_range_mm=1200,
                 window=6, min_hits=3, min_intensity=60,
                 cluster_eps_m=0.15, cluster_min_samples=3,
                 world_eps_m=0.2, yaw_sign=1):
        self.max_range_mm = max_range_mm
        self.min_hits = min_hits
        self.min_intensity = min_intensity
        self.cluster_eps_m = cluster_eps_m
        self.cluster_min_samples = cluster_min_samples
        self.world_eps_m = world_eps_m
        self.yaw_sign = yaw_sign
        self._history = deque(maxlen=window)  # 每项: 本次poll识别到的候选点世界坐标列表 [(wx,wy), ...]

    def update(self, radar, x_m, y_m, yaw_rad):
        """拉一次雷达当前scan，剔除"够格当大障碍物"的聚类，剩下的孤立/小聚类点转换成
        世界系坐标记入历史窗口。x_m/y_m/yaw_rad 是飞机当前位姿(同 Mission_GPT 里
        pos[0]/pos[1]/yaw，来自 t265 的世界系位置+朝向)。返回本次识别到的候选点世界坐标列表。
        """
        scan = radar.get_scan()
        points = []
        for angle in sorted(scan.keys()):
            distance_mm, intensity = scan[angle]
            if distance_mm <= 0 or distance_mm > self.max_range_mm or intensity < self.min_intensity:
                continue
            points.append((angle, distance_mm))

        all_clusters = _cluster_points(points, eps_m=self.cluster_eps_m, min_samples=1)
        candidates = []
        for cluster in all_clusters:
            if len(cluster) >= self.cluster_min_samples:
                continue  # 大障碍物，交给 get_obstacles() 处理，不算细目标候选
            angle, distance_mm = min(cluster, key=lambda p: p[1])
            bx, by = radar_angle_to_body_xy(angle, distance_mm)
            wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=self.yaw_sign)
            candidates.append((wx, wy))

        self._history.append(candidates)
        return candidates

    def confirmed_poles(self):
        """在滑动窗口内、世界系距离在 world_eps_m 以内重复出现次数 >= min_hits 的候选，
        判定为确认的杆子。返回 [{'x','y','hits'}, ...]，按距离世界系原点(通常是起飞点，
        不是飞机当前位置)由近到远排序——以后接入导航要找"离飞机当前最近的杆子"，调用方
        需要自己用飞机当前位置重新排序，这里不做。"""
        all_candidates = [c for frame in self._history for c in frame]
        n = len(all_candidates)
        used = [False] * n
        confirmed = []
        for i in range(n):
            if used[i]:
                continue
            x1, y1 = all_candidates[i]
            group = [(x1, y1)]
            used[i] = True
            for j in range(i + 1, n):
                if used[j]:
                    continue
                x2, y2 = all_candidates[j]
                if math.hypot(x2 - x1, y2 - y1) <= self.world_eps_m:
                    group.append((x2, y2))
                    used[j] = True
            if len(group) >= self.min_hits:
                avg_x = sum(x for x, _ in group) / len(group)
                avg_y = sum(y for _, y in group) / len(group)
                confirmed.append({"x": avg_x, "y": avg_y, "hits": len(group)})
        confirmed.sort(key=lambda o: math.hypot(o["x"], o["y"]))
        return confirmed

    def reset(self):
        self._history.clear()
