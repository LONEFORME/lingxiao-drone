"""
Intel T265 视觉里程计接口 (含模拟 fallback)
"""
import threading
import time
import numpy as np
import math
from Lcode.Logger import logger

try:
    import pyrealsense2 as rs
except ImportError:
    logger.warning("未找到 pyrealsense2，使用模拟模式")
    rs = None

try:
    import transformations as tf
except ImportError:
    logger.warning("未找到 transformations，使用简化坐标转换")
    tf = None


class t265_class:
    def __init__(self):
        self.running = False
        self.pose_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.velocity_data = np.array([0.0, 0.0, 0.0])
        self.raw_imu_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # ax,ay,az,gx,gy,gz
        self.lock = threading.Lock()
        self.error_count = 0
        self.max_error_count = 10
        self.calibration_offset = np.array([0.0, 0.0, 0.0])
        # 未收到首个真实Pose帧前必须保持0，禁止用默认满置信度绕过启动门禁。
        self.last_confidence = 0
        self.last_pose_monotonic = 0.0
        self.pose_frame_count = 0
        self.pipe = None
        self.cfg = None
        self.use_simulation = rs is None

        # T265 坐标 → 航空坐标
        self.H_aeroRef_T265Ref = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
        self.H_T265body_aeroBody = np.linalg.inv(self.H_aeroRef_T265Ref)

        self.InitialAngle_flag = False
        self.InitialAngle_Yaw = 0.0

        self.prev_position_x = 0.0
        self.prev_position_y = 0.0
        self.prev_position_z = 0.0
        self.prev_velocity_x = 0.0
        self.prev_velocity_y = 0.0
        self.prev_velocity_z = 0.0

        # 低通滤波前的原始值（诊断用，不参与飞控闭环，不影响 get_position()/get_velocity() 的既有行为）
        self.raw_pose_data = np.array([0.0, 0.0, 0.0])
        self.raw_velocity_data = np.array([0.0, 0.0, 0.0])

    def low_pass_filter(self, new_val, prev_val, alpha=0.15):
        return alpha * new_val + (1 - alpha) * prev_val

    def start(self):
        try:
            logger.info("T265 启动中...")
            if not self.use_simulation:
                self.pipe = rs.pipeline()
                self.cfg = rs.config()
                self.cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
                # 2026-07-10新增：单独订阅加速度计/陀螺仪流，用于问题7降落确认异常的
                # 独立验证——电机转动的振动特征不依赖凌霄IMU自己上报的任何遥测字段
                self.cfg.enable_stream(rs.stream.accel)
                self.cfg.enable_stream(rs.stream.gyro)
                self.pipe.start(self.cfg)
                logger.info("T265 启动成功（真实设备）")
            else:
                time.sleep(1)
                logger.info("T265 启动成功（模拟模式）")

            self.running = True
            self.data_thread = threading.Thread(target=self._data_acquisition)
            self.data_thread.daemon = True
            self.data_thread.start()
            return True
        except Exception as e:
            logger.error(f"T265 启动失败: {e}")
            return False

    def _data_acquisition(self):
        """数据采集线程 (200Hz raw → 滤波后 ~30Hz 有效)"""
        while self.running:
            try:
                if not self.use_simulation:
                    frames = self.pipe.wait_for_frames()
                    pose = frames.get_pose_frame()
                    if pose:
                        data = pose.get_pose_data()
                        tracking_confidence = getattr(data, 'tracker_confidence', 3)
                        with self.lock:
                            self.last_confidence = tracking_confidence
                            self.last_pose_monotonic = time.monotonic()
                            self.pose_frame_count += 1

                        if tracking_confidence < 2:
                            if not hasattr(self, '_last_conf_warn') or time.time() - self._last_conf_warn > 2.0:
                                logger.warning(f"T265 置信度过低 ({tracking_confidence})，位置冻结")
                                self._last_conf_warn = time.time()
                            self.error_count = 0
                            continue

                        qx, qy, qz, qw = data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w
                        tx, tv = data.translation, data.velocity

                        if tf:
                            H = tf.quaternion_matrix([qw, qx, qy, qz])
                            H[:3, 3] = [tx.x, tx.y, tx.z]
                            Ha = self.H_aeroRef_T265Ref @ H @ self.H_T265body_aeroBody
                            rpy_rad = np.array(tf.euler_from_matrix(Ha, 'sxyz'))
                            raw_x = -Ha[1, 3]
                            raw_y = -Ha[0, 3]
                            raw_z = -Ha[2, 3]
                        else:
                            roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
                            pitch = np.arcsin(2*(qw*qy - qz*qx))
                            yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                            rpy_rad = np.array([roll, pitch, yaw])
                            raw_x = -tx.z
                            raw_y = -tx.x
                            raw_z = +tx.y

                        with self.lock:
                            if not self.InitialAngle_flag:
                                self.InitialAngle_Yaw = rpy_rad[2]
                                self.InitialAngle_flag = True
                        yawerr = -(rpy_rad[2] - self.InitialAngle_Yaw)
                        if yawerr > math.pi:
                            yawerr -= 2*math.pi
                        elif yawerr < -math.pi:
                            yawerr += 2*math.pi

                        px = self.low_pass_filter(raw_x, self.prev_position_x)
                        py = self.low_pass_filter(raw_y, self.prev_position_y)
                        pz = self.low_pass_filter(raw_z, self.prev_position_z)
                        self.prev_position_x, self.prev_position_y, self.prev_position_z = px, py, pz

                        rvx = -tv.z
                        rvy = -tv.x
                        rvz = +tv.y
                        vx = self.low_pass_filter(rvx, self.prev_velocity_x)
                        vy = self.low_pass_filter(rvy, self.prev_velocity_y)
                        vz = self.low_pass_filter(rvz, self.prev_velocity_z)
                        self.prev_velocity_x, self.prev_velocity_y, self.prev_velocity_z = vx, vy, vz

                        with self.lock:
                            self.pose_data[:] = [px, py, pz, rpy_rad[0], rpy_rad[1], yawerr]
                            self.velocity_data[:] = [vx, vy, vz]
                            self.raw_pose_data[:] = [raw_x, raw_y, raw_z]
                            self.raw_velocity_data[:] = [rvx, rvy, rvz]

                    # 2026-07-10新增：加速度计/陀螺仪是独立的帧类型，跟pose帧一起从
                    # 同一个frameset里取，缺失时保留上一次的值(不强制置零，避免瞬时
                    # 缺帧导致振动特征分析出现假的"静止"读数)
                    accel_frame = frames.first_or_default(rs.stream.accel)
                    gyro_frame = frames.first_or_default(rs.stream.gyro)
                    if accel_frame:
                        a = accel_frame.as_motion_frame().get_motion_data()
                        with self.lock:
                            self.raw_imu_data[0:3] = [a.x, a.y, a.z]
                    if gyro_frame:
                        g = gyro_frame.as_motion_frame().get_motion_data()
                        with self.lock:
                            self.raw_imu_data[3:6] = [g.x, g.y, g.z]
                else:
                    # 模拟模式
                    time.sleep(0.03)
                    rx = self.pose_data[0] + np.random.normal(0, 0.01)
                    ry = self.pose_data[1] + np.random.normal(0, 0.01)
                    rz = 1.0
                    px = self.low_pass_filter(rx, self.prev_position_x)
                    py = self.low_pass_filter(ry, self.prev_position_y)
                    pz = self.low_pass_filter(rz, self.prev_position_z)
                    self.prev_position_x, self.prev_position_y, self.prev_position_z = px, py, pz

                    roll = np.random.normal(0, 0.01)
                    pitch = np.random.normal(0, 0.01)
                    yaw_v = self.pose_data[5] + np.random.normal(0, 0.005)
                    with self.lock:
                        if not self.InitialAngle_flag:
                            self.InitialAngle_Yaw = yaw_v
                            self.InitialAngle_flag = True
                    yawerr = -(yaw_v - self.InitialAngle_Yaw)
                    if yawerr > math.pi:
                        yawerr -= 2*math.pi
                    elif yawerr < -math.pi:
                        yawerr += 2*math.pi

                    rvx = np.random.normal(0, 0.05)
                    rvy = np.random.normal(0, 0.05)
                    rvz = 0.0
                    vx = self.low_pass_filter(rvx, self.prev_velocity_x)
                    vy = self.low_pass_filter(rvy, self.prev_velocity_y)
                    vz = self.low_pass_filter(rvz, self.prev_velocity_z)
                    self.prev_velocity_x, self.prev_velocity_y, self.prev_velocity_z = vx, vy, vz

                    # 模拟悬停时的振动特征：重力分量(az≈9.8)+小噪声，陀螺仪接近0
                    sim_ax = np.random.normal(0, 0.05)
                    sim_ay = np.random.normal(0, 0.05)
                    sim_az = 9.8 + np.random.normal(0, 0.05)
                    sim_gx = np.random.normal(0, 0.02)
                    sim_gy = np.random.normal(0, 0.02)
                    sim_gz = np.random.normal(0, 0.02)

                    with self.lock:
                        self.pose_data[:] = [px, py, pz, roll, pitch, yawerr]
                        self.velocity_data[:] = [vx, vy, vz]
                        self.raw_pose_data[:] = [rx, ry, rz]
                        self.raw_velocity_data[:] = [rvx, rvy, rvz]
                        self.raw_imu_data[:] = [sim_ax, sim_ay, sim_az, sim_gx, sim_gy, sim_gz]
                        self.last_confidence = 3
                        self.last_pose_monotonic = time.monotonic()
                        self.pose_frame_count += 1

                self.error_count = 0
            except Exception as e:
                self.error_count += 1
                if self.error_count > self.max_error_count:
                    logger.error(f"T265 连续失败 {self.max_error_count} 次，停止")
                    self.running = False
                    if not self.use_simulation and self.pipe:
                        try:
                            self.pipe.stop()
                        except Exception:
                            pass
                else:
                    logger.warning(f"T265 采集异常: {e}")
                time.sleep(0.1)

    def get_position(self):
        with self.lock:
            return self.pose_data[:3].copy() - self.calibration_offset[:3]

    def apply_position_continuity_correction(self, delta_xyz):
        """Shift the software origin without restarting or re-zeroing T265.

        ``delta_xyz`` is an impossible discontinuity observed in the already
        calibrated position.  Adding it to ``calibration_offset`` removes the
        same discontinuity from all subsequent ``get_position()`` calls while
        preserving the original H-point coordinate frame.
        """
        delta = np.asarray(delta_xyz, dtype=float)
        if delta.shape != (3,) or not np.all(np.isfinite(delta)):
            raise ValueError("delta_xyz must contain three finite values")
        with self.lock:
            self.calibration_offset[:3] += delta
            return self.pose_data[:3].copy() - self.calibration_offset[:3]

    def get_orientation(self):
        with self.lock:
            return self.pose_data[3:].copy()

    def get_yaw_deg_x100(self):
        with self.lock:
            return int(math.degrees(self.pose_data[5]) * 100)

    def get_velocity(self):
        with self.lock:
            return self.velocity_data.copy()

    def get_raw_position(self):
        """诊断用：低通滤波前的原始位置(未做calibration_offset校准)。不用于飞控闭环。"""
        with self.lock:
            return self.raw_pose_data.copy()

    def get_raw_velocity(self):
        """诊断用：低通滤波前的原始速度(T265 SDK直接输出)。不用于飞控闭环。"""
        with self.lock:
            return self.raw_velocity_data.copy()

    def get_raw_imu(self):
        """返回(ax,ay,az,gx,gy,gz)：T265自带加速度计/陀螺仪原始数据(m/s^2, rad/s)，
        不经过任何位置/速度融合，独立于凌霄IMU自己上报的任何遥测字段。用于诊断
        问题7类降落确认异常——电机转动的振动特征应该能在这份数据里独立体现。"""
        with self.lock:
            return self.raw_imu_data.copy()

    def get_tracking_confidence(self):
        with self.lock:
            return int(self.last_confidence)

    def get_pose_age_s(self, now=None):
        """最后一个真实/模拟Pose帧的年龄；从未收到时返回正无穷。"""
        now = time.monotonic() if now is None else float(now)
        with self.lock:
            timestamp = float(self.last_pose_monotonic)
        return math.inf if timestamp <= 0.0 else max(0.0, now - timestamp)

    def get_pose_frame_count(self):
        with self.lock:
            return int(self.pose_frame_count)

    def autoset(self):
        logger.info("T265 自动校准...")
        positions = [self.get_position() for _ in range(10)]
        avg = np.mean(positions, axis=0)
        with self.lock:
            self.calibration_offset[:3] = avg
            self.InitialAngle_flag = False
            self.InitialAngle_Yaw = 0.0
        logger.info(f"T265 校准完成, 偏移: {avg}")
        return True

    def is_running(self):
        return self.running

    def stop(self):
        self.running = False
        if hasattr(self, 'data_thread') and self.data_thread.is_alive():
            self.data_thread.join(timeout=2.0)
        if not self.use_simulation and self.pipe:
            try:
                self.pipe.stop()
            except Exception:
                pass
        logger.info("T265 已停止")
