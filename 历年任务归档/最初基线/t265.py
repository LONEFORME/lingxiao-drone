import threading
import time
import numpy as np
import math
from Lcode.Logger import logger

# 尝试导入RealSense SDK
try:
    import pyrealsense2 as rs
except ImportError:
    logger.warning("未找到pyrealsense2模块，将使用模拟数据")
    rs = None

# 尝试导入transformations模块
try:
    import transformations as tf
except ImportError:
    logger.warning("未找到transformations模块，将使用简化的坐标转换")
    tf = None

class t265_class:
    def __init__(self):
        """初始化T265相机"""
        self.running = False
        self.pose_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # x, y, z, roll, pitch, yaw
        self.velocity_data = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
        self.lock = threading.Lock()
        self.error_count = 0
        self.max_error_count = 10
        self.calibration_offset = np.array([0.0, 0.0, 0.0])  # 校准偏移量
        self.last_confidence = 3  # 最后已知追踪置信度 (0=失败,1=低,2=中,3=高)
        self.pipe = None
        self.cfg = None
        self.use_simulation = rs is None
        
        # T265坐标转航空坐标的变换矩阵
        self.H_aeroRef_T265Ref = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
        self.H_T265body_aeroBody = np.linalg.inv(self.H_aeroRef_T265Ref)
        
        # 初始姿态
        self.InitialAngle_flag = False
        self.InitialAngle_Yaw = 0.0
        
        # 上一帧滤波用变量
        self.prev_position_x = 0.0
        self.prev_position_y = 0.0
        self.prev_position_z = 0.0
        self.prev_velocity_x = 0.0
        self.prev_velocity_y = 0.0
        self.prev_velocity_z = 0.0
        
    def start(self):
        """启动T265相机和数据获取线程"""
        try:
            logger.info("T265相机启动中...")
            
            if not self.use_simulation:
                # 使用真实的RealSense SDK
                self.pipe = rs.pipeline()
                self.cfg = rs.config()
                self.cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
                self.pipe.start(self.cfg)
                logger.info("T265相机启动成功（真实设备）")
            else:
                # 使用模拟数据
                time.sleep(1)  # 模拟启动时间
                logger.info("T265相机启动成功（模拟模式）")
            
            self.running = True
            self.data_thread = threading.Thread(target=self._data_acquisition)
            self.data_thread.daemon = True
            self.data_thread.start()
            
            return True
        except Exception as e:
            logger.error(f"T265相机启动失败: {str(e)}")
            return False
    
    def low_pass_filter(self, new_val, prev_val, alpha=0.15):
        """低通滤波器

        Args:
            new_val: 新值
            prev_val:  previous value
            alpha: 滤波系数 (0-1)，0.15≈8Hz截止频率，平衡噪声抑制和相位滞后
            
        Returns:
            滤波后的值
        """
        return alpha * new_val + (1 - alpha) * prev_val
    
    def _data_acquisition(self):
        """数据获取线程"""
        while self.running:
            try:
                if not self.use_simulation:
                    # 使用真实的RealSense SDK获取数据
                    frames = self.pipe.wait_for_frames()
                    pose = frames.get_pose_frame()
                    if pose:
                        data = pose.get_pose_data()

                        # 检查追踪置信度（0=失败, 1=低, 2=中, 3=高）
                        # 飞行振动时置信度会下降，此时位置数据不可靠
                        tracking_confidence = getattr(data, 'tracker_confidence', 3)
                        # 追踪置信度缓存（供外部读取）
                        with self.lock:
                            self.last_confidence = tracking_confidence

                        if tracking_confidence < 2:
                            # 置信度过低，保持上一帧有效值不变（位置冻结）
                            # 以较低频率输出警告，避免刷屏
                            if not hasattr(self, '_last_conf_warn') or time.time() - self._last_conf_warn > 2.0:
                                logger.warning(f"T265追踪置信度过低({tracking_confidence})，位置数据冻结")
                                self._last_conf_warn = time.time()
                            self.error_count = 0
                            continue

                        # 处理姿态数据
                        qua0 = data.rotation.x
                        qua1 = data.rotation.y
                        qua2 = data.rotation.z
                        qua3 = data.rotation.w
                        
                        tx = data.translation
                        tv = data.velocity
                        
                        if tf:
                            # 使用齐次变换矩阵，保证位置和姿态在同一坐标系
                            H_T265Ref_T265body = tf.quaternion_matrix([qua3, qua0, qua1, qua2])
                            H_T265Ref_T265body[:3, 3] = [tx.x, tx.y, tx.z]
                            H_aeroRef_aeroBody = self.H_aeroRef_T265Ref @ H_T265Ref_T265body @ self.H_T265body_aeroBody
                            rpy_rad = np.array(tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))
                            # 旧t265轴系 (世界锁死): x=-前(tz), y=-右(tx), z=上(ty)
                            raw_pos_x = -H_aeroRef_aeroBody[1, 3]  # -forward
                            raw_pos_y = -H_aeroRef_aeroBody[0, 3]  # -right
                            raw_pos_z = -H_aeroRef_aeroBody[2, 3]  # -down → up
                        else:
                            # 简化的四元数转欧拉角 + 旧t265轴系坐标转换
                            w, x, y, z = qua3, qua0, qua1, qua2
                            roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                            pitch = np.arcsin(2*(w*y - z*x))
                            yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                            rpy_rad = np.array([roll, pitch, yaw])
                            # 旧t265轴系 (世界锁死): x=-前(tz), y=-右(tx), z=上(ty)
                            raw_pos_x = -tx.z
                            raw_pos_y = -tx.x
                            raw_pos_z = +tx.y
                        
                        # 初始航向角处理
                        with self.lock:
                            if not self.InitialAngle_flag:
                                self.InitialAngle_Yaw = rpy_rad[2]
                                self.InitialAngle_flag = True
                        yawerr = -(rpy_rad[2] - self.InitialAngle_Yaw)
                        # 归一化到 [-PI, PI]，防止 >180° 时 PID 走错方向
                        if yawerr > math.pi: yawerr -= 2*math.pi
                        elif yawerr < -math.pi: yawerr += 2*math.pi
                        
                        # 滤波处理
                        position_x = self.low_pass_filter(raw_pos_x, self.prev_position_x)
                        position_y = self.low_pass_filter(raw_pos_y, self.prev_position_y)
                        position_z = self.low_pass_filter(raw_pos_z, self.prev_position_z)
                        self.prev_position_x = position_x
                        self.prev_position_y = position_y
                        self.prev_position_z = position_z
                        
                        # 速度 (旧t265轴系 世界锁死): vx=-前, vy=-右, vz=上
                        raw_vel_x = -tv.z
                        raw_vel_y = -tv.x
                        raw_vel_z = +tv.y
                        
                        velocity_x = self.low_pass_filter(raw_vel_x, self.prev_velocity_x)
                        velocity_y = self.low_pass_filter(raw_vel_y, self.prev_velocity_y)
                        velocity_z = self.low_pass_filter(raw_vel_z, self.prev_velocity_z)
                        self.prev_velocity_x = velocity_x
                        self.prev_velocity_y = velocity_y
                        self.prev_velocity_z = velocity_z
                        
                        with self.lock:
                            # 更新姿态数据
                            self.pose_data[0] = position_x
                            self.pose_data[1] = position_y
                            self.pose_data[2] = position_z
                            self.pose_data[3] = rpy_rad[0]
                            self.pose_data[4] = rpy_rad[1]
                            self.pose_data[5] = yawerr  # 使用相对于初始位置的偏航角
                            
                            # 更新速度数据
                            self.velocity_data[0] = velocity_x
                            self.velocity_data[1] = velocity_y
                            self.velocity_data[2] = velocity_z
                else:
                    # 模拟数据
                    time.sleep(0.03)  # 30Hz采样率
                    
                    # 模拟位置数据（x, y, z）
                    raw_pos_x = self.pose_data[0] + np.random.normal(0, 0.01)
                    raw_pos_y = self.pose_data[1] + np.random.normal(0, 0.01)
                    raw_pos_z = 1.0  # z (固定高度)
                    
                    # 滤波处理
                    position_x = self.low_pass_filter(raw_pos_x, self.prev_position_x)
                    position_y = self.low_pass_filter(raw_pos_y, self.prev_position_y)
                    position_z = self.low_pass_filter(raw_pos_z, self.prev_position_z)
                    self.prev_position_x = position_x
                    self.prev_position_y = position_y
                    self.prev_position_z = position_z
                    
                    # 模拟姿态数据（roll, pitch, yaw）
                    roll = np.random.normal(0, 0.01)
                    pitch = np.random.normal(0, 0.01)
                    yaw = self.pose_data[5] + np.random.normal(0, 0.005)
                    
                    # 初始航向角处理
                    with self.lock:
                        if not self.InitialAngle_flag:
                            self.InitialAngle_Yaw = yaw
                            self.InitialAngle_flag = True
                    yawerr = -(yaw - self.InitialAngle_Yaw)
                    # 归一化到 [-PI, PI]
                    if yawerr > math.pi: yawerr -= 2*math.pi
                    elif yawerr < -math.pi: yawerr += 2*math.pi
                    
                    # 模拟速度数据
                    raw_vel_x = np.random.normal(0, 0.05)
                    raw_vel_y = np.random.normal(0, 0.05)
                    raw_vel_z = 0.0
                    
                    # 滤波处理
                    velocity_x = self.low_pass_filter(raw_vel_x, self.prev_velocity_x)
                    velocity_y = self.low_pass_filter(raw_vel_y, self.prev_velocity_y)
                    velocity_z = self.low_pass_filter(raw_vel_z, self.prev_velocity_z)
                    self.prev_velocity_x = velocity_x
                    self.prev_velocity_y = velocity_y
                    self.prev_velocity_z = velocity_z
                    
                    with self.lock:
                        # 更新数据
                        self.pose_data[0] = position_x
                        self.pose_data[1] = position_y
                        self.pose_data[2] = position_z
                        self.pose_data[3] = roll
                        self.pose_data[4] = pitch
                        self.pose_data[5] = yawerr
                        
                        self.velocity_data[0] = velocity_x
                        self.velocity_data[1] = velocity_y
                        self.velocity_data[2] = velocity_z
                
                self.error_count = 0  # 重置错误计数
            except Exception as e:
                self.error_count += 1
                if self.error_count > self.max_error_count:
                    logger.error(f"T265数据获取连续失败{self.max_error_count}次，停止数据获取")
                    self.running = False
                    if not self.use_simulation and self.pipe:
                        try:
                            self.pipe.stop()
                        except Exception:
                            pass
                else:
                    logger.warning(f"T265数据获取失败: {str(e)}")
                time.sleep(0.1)
    
    def get_pose(self):
        """获取当前位姿数据
        
        Returns:
            np.array: [x, y, z, roll, pitch, yaw] 单位：米，弧度
        """
        with self.lock:
            result = self.pose_data.copy()
            result[:3] -= self.calibration_offset[:3]
            return result
    
    def get_position(self):
        """获取当前位置数据
        
        Returns:
            np.array: [x, y, z] 单位：米
        """
        with self.lock:
            return (self.pose_data[:3].copy() - self.calibration_offset[:3])
    
    def get_orientation(self):
        """获取当前姿态数据

        Returns:
            np.array: [roll, pitch, yaw] 单位：弧度
        """
        with self.lock:
            return self.pose_data[3:].copy()

    def get_yaw_deg_x100(self):
        """获取偏航角（归一化到 [-180,180]），单位 0.01°，用于串口传输

        Returns:
            int: yaw * 100，范围 [-18000, 18000]
        """
        with self.lock:
            return int(math.degrees(self.pose_data[5]) * 100)
    
    def get_velocity(self):
        """获取当前速度数据

        Returns:
            np.array: [vx, vy, vz] 单位：米/秒
        """
        with self.lock:
            return self.velocity_data.copy()

    def get_tracking_confidence(self):
        """获取最后已知的T265追踪置信度

        Returns:
            int: 0=失败, 1=低, 2=中, 3=高
        """
        with self.lock:
            return int(self.last_confidence)
    
    def autoset(self):
        """自动设置初始位置为原点"""
        try:
            logger.info("T265自动校准中...")
            # 采集多次数据求平均作为初始位置
            positions = []
            for i in range(10):
                positions.append(self.get_position())
                time.sleep(0.1)
            
            avg_position = np.mean(positions, axis=0)
            with self.lock:
                self.calibration_offset[:3] = avg_position
                # 重置初始航向角
                self.InitialAngle_flag = False
                self.InitialAngle_Yaw = 0.0
            
            logger.info(f"T265校准完成，偏移量: {self.calibration_offset[:3]}")
            return True
        except Exception as e:
            logger.error(f"T265校准失败: {str(e)}")
            return False
    
    def set_origin(self, x=0, y=0, z=0):
        """手动设置原点位置
        
        Args:
            x: X轴偏移量
            y: Y轴偏移量
            z: Z轴偏移量
        """
        with self.lock:
            self.calibration_offset = np.array([x, y, z])
            # 重置初始航向角
            self.InitialAngle_flag = False
            self.InitialAngle_Yaw = 0.0
        logger.info(f"T265原点设置为: {self.calibration_offset}")
    
    def stop(self):
        """停止T265相机"""
        self.running = False
        if hasattr(self, 'data_thread') and self.data_thread.is_alive():
            self.data_thread.join(timeout=2.0)
        
        # 停止RealSense管道
        if not self.use_simulation and self.pipe:
            try:
                self.pipe.stop()
                logger.info("T265管道已停止")
            except Exception as e:
                logger.warning(f"停止T265管道时出错: {str(e)}")
        
        logger.info("T265相机已停止")
    
    def is_running(self):
        """检查T265是否正在运行
        
        Returns:
            bool: 是否正在运行
        """
        return self.running
    
    def get_status(self):
        """获取T265状态信息
        
        Returns:
            dict: 状态信息
        """
        return {
            'running': self.running,
            'error_count': self.error_count,
            'calibration_offset': self.calibration_offset.tolist(),
            'last_pose': self.get_pose().tolist()
        }

# 测试代码
if __name__ == "__main__":
    t265 = t265_class()
    
    if t265.start():
        # 等待相机稳定
        time.sleep(2)
        t265.autoset()
        
        # 持续在同一行刷新（无限循环，想停按 Ctrl+C）
        while True:
            position = t265.get_position()
            orientation = t265.get_orientation()
            velocity = t265.get_velocity()
            
            # 核心：\r 回到行首 + end='' 不换行
            print(
                f"\r位置: {position[0]:+8.3f} {position[1]:+8.3f} {position[2]:+8.3f} | "
                f"姿态: {orientation[0]:+7.2f} {orientation[1]:+7.2f} {orientation[2]:+7.2f} | "
                f"速度: {velocity[0]:+7.2f} {velocity[1]:+7.2f} {velocity[2]:+7.2f}",
                end=''
            )
            
            time.sleep(0.1)
        
        t265.stop()
    else:
        print("T265启动失败")
