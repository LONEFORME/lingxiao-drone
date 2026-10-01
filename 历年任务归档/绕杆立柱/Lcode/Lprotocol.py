"""
串口通信模块 — 仅飞控 (Serial_fc)，无地面站

帧协议:
  AA 01: T265 速度帧 (vx, vy, yaw)  @100Hz
  AA 02: 指令帧 (task_sta, com_x/y/z/yaw)  @50Hz
  AA frame_id len DATA CK FF: 飞控下行遥测
    frame_id=0x01 飞行关键帧 @50Hz（姿态/解锁状态/激光高度/光流速度）
    frame_id=0x02 调试扩展帧 @2Hz（飞控速度估计/光流IMU原始数据/电机PWM非零掩码）
"""
import serial
import threading
import time
from typing import List
from Lcode.Logger import logger
from Lcode.global_variable import lock, fc_last_rx_time


class Serial_fc(object):
    def __init__(self, port, baudrate):
        # 帧1/帧2均为整帧一次性阻塞发送(飞控侧 Send_str_by_len)，不再逐字节分段，
        # read()超时1.0s留了充足余量；write_timeout同样必须设置，否则串口写缓冲区
        # 写不出去时 ser.write() 会无限阻塞且不抛异常，两个发送线程会静默卡死
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1.0, write_timeout=1.0)
        self.fclisten_running = False
        self.t265send_running = False
        self.cmdsend_running = False
        self._last_laser_height_cm = 0.0  # 最后已知激光高度 (m)
        self.debug_data = {}  # 调试扩展帧(0x02)最新数据: fc_vel/of_acc/of_gyr

    def listen_start(self, rxbuffer: List[int]):
        self.fclisten_running = True
        self.ser.reset_input_buffer()  # 丢弃启动前残留在缓冲区里的陈旧字节
        t = threading.Thread(target=Serial_fc.listen_fc, args=(self, rxbuffer))
        t.daemon = True
        t.start()
        logger.info("飞控串口监听线程启动")

    def listen_end(self):
        self.fclisten_running = False
        logger.info("飞控串口监听线程关闭")

    def listen_fc(self, rxbuffer: List[int]):
        """接收下行帧: AA | frame_id | len | DATA[len] | checksum | 0xFF
        frame_id=0x01 飞行关键帧(24B, 50Hz): mission_stage/rol/pit/yaw/fusion_state/unlock_sta/x_int/y_int/laser/of1_dx/of1_dy/of_quality/of_link_sta/of_work_sta
        frame_id=0x02 调试扩展帧(19B, 2Hz):  fc_vel_xyz / of_acc_xyz / of_gyr_xyz / motor_pwm_mask
        """
        while self.fclisten_running:
            try:
                byte_data = self.ser.read()
            except serial.SerialException as e:
                logger.error(f"飞控串口读取失败: {e}")
                break
            if not byte_data:
                time.sleep(0.01)  # 真正空闲(超时无数据)才睡，避免忙等
                continue
            if byte_data == b'\xAA':
                header = self.ser.read(2)
                if len(header) < 2:
                    continue
                frame_id, length = header[0], header[1]
                body = self.ser.read(length + 2)
                if len(body) < length + 2:
                    continue
                data = body[:length]
                checksum_recv = body[length]
                end_byte = body[length + 1]
                if end_byte != 0xFF:
                    continue
                checksum = (sum(header) + sum(data)) & 0xFF
                if checksum != checksum_recv:
                    continue

                def to_signed16(v):
                    return v - 0x10000 if v >= 0x8000 else v

                if frame_id == 0x01 and length == 24:
                    mission_stage = data[0]
                    roll_x100 = to_signed16(data[1] | (data[2] << 8))
                    pitch_x100 = to_signed16(data[3] | (data[4] << 8))
                    yaw_x100 = to_signed16(data[5] | (data[6] << 8))
                    fusion_state = data[7]
                    unlock_sta = data[8]
                    integral_x = (data[9] | (data[10] << 8)) - 0x4000
                    integral_y = (data[11] | (data[12] << 8)) - 0x4000
                    laser_height_cm = data[13] | (data[14] << 8) | (data[15] << 16) | (data[16] << 24)
                    of1_dx = to_signed16(data[17] | (data[18] << 8))
                    of1_dy = to_signed16(data[19] | (data[20] << 8))
                    of_quality = data[21]
                    of_link_sta = data[22]
                    of_work_sta = data[23]
                    with lock:
                        rxbuffer.clear()
                        rxbuffer.append(mission_stage)
                        rxbuffer.append(roll_x100)
                        rxbuffer.append(pitch_x100)
                        rxbuffer.append(yaw_x100)
                        rxbuffer.append(fusion_state)
                        rxbuffer.append(unlock_sta)
                        rxbuffer.append(integral_x)
                        rxbuffer.append(integral_y)
                        rxbuffer.append(laser_height_cm)
                        rxbuffer.append(of1_dx)
                        rxbuffer.append(of1_dy)
                        rxbuffer.append(of_quality)
                        rxbuffer.append(of_link_sta)
                        rxbuffer.append(of_work_sta)
                    if laser_height_cm > 5:
                        with lock:
                            self._last_laser_height_cm = float(laser_height_cm) / 100.0
                    fc_last_rx_time.value = time.time()

                elif frame_id == 0x02 and length == 19:
                    fc_vel_x = to_signed16(data[0] | (data[1] << 8))
                    fc_vel_y = to_signed16(data[2] | (data[3] << 8))
                    fc_vel_z = to_signed16(data[4] | (data[5] << 8))
                    of_acc_x = to_signed16(data[6] | (data[7] << 8))
                    of_acc_y = to_signed16(data[8] | (data[9] << 8))
                    of_acc_z = to_signed16(data[10] | (data[11] << 8))
                    of_gyr_x = to_signed16(data[12] | (data[13] << 8))
                    of_gyr_y = to_signed16(data[14] | (data[15] << 8))
                    of_gyr_z = to_signed16(data[16] | (data[17] << 8))
                    # 电机PWM非零位掩码(bit0~3=m1~m4)：诊断unlock_sta是否假阳性
                    # (问题7 2026-07-08：unlock_sta读到0但电机实际未停转)
                    motor_pwm_mask = data[18]
                    # 2026-07-10新增：记录帧2到达时刻，用于跟unlock_sta翻转时间对比，
                    # 核实motor_pwm_mask是不是滞后的旧快照(问题22追加分析：帧2降频约
                    # 2.5秒才更新一次，land()读到的可能是过期数据)
                    # 2026-07-12新增：bit4=纯超时兜底是否已放弃自动锁定(高度仍偏高)，
                    # 复用同一字节的空闲位，不扩展帧长度
                    land_timeout_gaveup = bool(motor_pwm_mask & 0x10)
                    with lock:
                        self.debug_data = {
                            "fc_vel": (fc_vel_x, fc_vel_y, fc_vel_z),
                            "of_acc": (of_acc_x, of_acc_y, of_acc_z),
                            "of_gyr": (of_gyr_x, of_gyr_y, of_gyr_z),
                            "motor_pwm_mask": motor_pwm_mask,
                            "motor_pwm_mask_t": time.time(),
                            "land_timeout_gaveup": land_timeout_gaveup,
                        }

    def _send_t265_loop(self, t265_obj, freq):
        """独立线程：发送 T265 速度帧 (AA 01)"""
        sleep_time = 1.0 / freq
        while self.t265send_running:
            if t265_obj is not None and t265_obj.is_running():
                vx, vy, _ = t265_obj.get_velocity()
                vx_cm = int(vx * 100)
                vy_cm = int(vy * 100)
                yaw_x100 = t265_obj.get_yaw_deg_x100()
                frame = [0xAA, 0x01,
                         (vx_cm >> 8) & 0xFF, vx_cm & 0xFF,
                         (vy_cm >> 8) & 0xFF, vy_cm & 0xFF,
                         (yaw_x100 >> 8) & 0xFF, yaw_x100 & 0xFF]
                ck = 0
                for b in frame[1:]:
                    ck ^= b
                frame.append(ck)
                frame.append(0xFF)

                # T265 位置帧 (AA 03)，简化版：直接发送 T265 自身坐标系下的位置(cm)，
                # 未做"解锁时机头方向对齐"的旋转变换
                x, y, z = t265_obj.get_position()
                x_cm = int(x * 100)
                y_cm = int(y * 100)
                z_cm = int(z * 100)
                pos_frame = [0xAA, 0x03]
                for v in (x_cm, y_cm, z_cm):
                    pos_frame += [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
                ck2 = 0
                for b in pos_frame[1:]:
                    ck2 ^= b
                pos_frame.append(ck2)
                pos_frame.append(0xFF)
                try:
                    self.ser.write(bytes(frame))
                    self.ser.write(bytes(pos_frame))
                except serial.SerialException as e:
                    logger.error(f"T265速度/位置帧发送失败，发送线程退出: {e}")
                    break
            time.sleep(sleep_time)

    def _send_command_loop(self, comlist, freq):
        """独立线程：发送指令帧 (AA 02)"""
        sleep_time = 1.0 / freq
        while self.cmdsend_running:
            with lock:
                values = list(comlist)
            ck = 0
            for b in values[1:-2]:
                ck ^= b
            values[-2] = ck
            try:
                self.ser.write(bytes(values))
            except serial.SerialException as e:
                logger.error(f"指令帧发送失败，发送线程退出: {e}")
                break
            time.sleep(sleep_time)

    def send_start(self, comlist=None, t265_obj=None, vel_freq=100, cmd_freq=50):
        self.t265send_running = True
        self.cmdsend_running = True

        if t265_obj is not None:
            t = threading.Thread(target=self._send_t265_loop, args=(t265_obj, vel_freq))
            t.daemon = True
            t.start()

        if comlist is not None:
            t = threading.Thread(target=self._send_command_loop, args=(comlist, cmd_freq))
            t.daemon = True
            t.start()

        parts = []
        if t265_obj is not None:
            parts.append("速度帧 %dHz" % vel_freq)
        if comlist is not None:
            parts.append("指令帧 %dHz" % cmd_freq)
        logger.info("飞控串口发送线程启动（%s）" % " + ".join(parts))

    def send_end(self):
        self.t265send_running = False
        self.cmdsend_running = False
        logger.info("飞控串口发送线程关闭")

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("飞控串口已关闭")
