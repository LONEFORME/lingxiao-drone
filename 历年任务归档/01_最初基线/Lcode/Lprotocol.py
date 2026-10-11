import serial
import threading
from typing import List
from Lcode.Logger import logger
import time
from Lcode.global_variable import lock,task_start_sign,fc_last_rx_time
DEBUG=False
class Serial_fc(object):
    def __init__(self,port,baudrate):
        # 帧1/帧2均为整帧一次性阻塞发送(飞控侧 Send_str_by_len)，不再逐字节分段，
        # read()超时1.0s留了充足余量；write_timeout同样必须设置，否则串口写缓冲区
        # 写不出去时 ser.write() 会无限阻塞且不抛异常，两个发送线程会静默卡死
        self.ser=serial.Serial(port=port,baudrate=baudrate,timeout=1.0,write_timeout=1.0)
        self.fclisten_running=False
        self.t265send_running=False
        self.cmdsend_running=False
        self.rate=460800
        self.startbyte=b'\xAA'
        self.endbyte=0xFF
        self._last_laser_height_cm = 0.0  # 最后已知激光高度(cm)
        self.debug_data = {}  # 调试扩展帧(0x02)最新数据: fc_vel/of_acc/of_gyr
    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("目前飞控串口状态：%s",self.ser.is_open)
    def listen_start(self,rxbuffer:List[int]):
        self.fclisten_running=True
        listen_thread=threading.Thread(target=Serial_fc.listen_fc,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("飞控串口监听线程启动")
    def listen_end(self):
        self.fclisten_running=False
        logger.info("飞控串口监听线程关闭")
    def listen_fc(self,rxbuffer:List[int]):
        global fc_last_rx_time
        # 新帧格式: AA | frame_id | len | DATA[len] | checksum | FF
        #   frame_id=0x01 飞行关键帧(24B, 50Hz): mission_stage/rol/pit/yaw/fusion_state/unlock_sta/x_int/y_int/laser/of1_dx/of1_dy/of_quality/of_link_sta/of_work_sta
        #   frame_id=0x02 调试扩展帧(19B, 2Hz):  fc_vel_xyz / of_acc_xyz / of_gyr_xyz
        while self.fclisten_running ==True:
            byte_data = self.ser.read()
            if byte_data == self.startbyte:
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
                if end_byte != self.endbyte:
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
                    intergral_x = (data[9] | (data[10] << 8)) - 0x4000
                    intergral_y = (data[11] | (data[12] << 8)) - 0x4000
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
                        rxbuffer.append(intergral_x)
                        rxbuffer.append(intergral_y)
                        rxbuffer.append(laser_height_cm)
                        rxbuffer.append(of1_dx)
                        rxbuffer.append(of1_dy)
                        rxbuffer.append(of_quality)
                        rxbuffer.append(of_link_sta)
                        rxbuffer.append(of_work_sta)
                    # 缓存激光高度供外部查询（有效值 > 10cm）
                    if laser_height_cm > 10:
                        with lock:
                            self._last_laser_height_cm = float(laser_height_cm) / 100.0
                    fc_last_rx_time.value = time.time()
                    if mission_stage==0x05:
                        task_start_sign.value=True
                    else:
                        task_start_sign.value=False
                    if DEBUG :
                        logger.info(rxbuffer)

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
                    with lock:
                        self.debug_data = {
                            "fc_vel": (fc_vel_x, fc_vel_y, fc_vel_z),
                            "of_acc": (of_acc_x, of_acc_y, of_acc_z),
                            "of_gyr": (of_gyr_x, of_gyr_y, of_gyr_z),
                            "motor_pwm_mask": motor_pwm_mask,
                            "motor_pwm_mask_t": time.time(),
                        }
            time.sleep(0.05)
    def _send_t265_loop(self, t265_obj, freq):
        """独立线程：发送 T265 速度+偏航帧 (0x01)"""
        sleep_time = 1.0 / freq
        while self.t265send_running:
            if t265_obj is not None and t265_obj.is_running():
                vx, vy, _ = t265_obj.get_velocity()
                vx_cm = int(vx * 100)
                vy_cm = int(vy * 100)
                yaw_x100 = t265_obj.get_yaw_deg_x100()
                # 帧格式: AA 01 vx_h vx_l vy_h vy_l yaw_h yaw_l CK FF
                t265_frame = [0xAA, 0x01,
                             (vx_cm >> 8) & 0xFF, vx_cm & 0xFF,
                             (vy_cm >> 8) & 0xFF, vy_cm & 0xFF,
                             (yaw_x100 >> 8) & 0xFF, yaw_x100 & 0xFF]
                ck = 0
                for b in t265_frame[1:]:
                    ck ^= b
                t265_frame.append(ck)
                t265_frame.append(0xFF)
                self.ser.write(bytes(t265_frame))

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
                self.ser.write(bytes(pos_frame))
            time.sleep(sleep_time)

    def _send_command_loop(self, comlist, freq):
        """独立线程：发送指令帧 (0x02)"""
        sleep_time = 1.0 / freq
        while self.cmdsend_running:
            with lock:
                values = list(comlist)
            # CK = XOR(02 ... sp_side)，位于 FF 前
            ck = 0
            for b in values[1:-2]:
                ck ^= b
            values[-2] = ck
            self.ser.write(bytes(values))
            time.sleep(sleep_time)

    def send_start(self, comlist=None, t265_obj=None, vel_freq=100, cmd_freq=50):
        self.t265send_running = True
        self.cmdsend_running = True

        if t265_obj is not None:
            t265_thread = threading.Thread(
                target=self._send_t265_loop, args=(t265_obj, vel_freq))
            t265_thread.daemon = True
            t265_thread.start()

        if comlist is not None:
            cmd_thread = threading.Thread(
                target=self._send_command_loop, args=(comlist, cmd_freq))
            cmd_thread.daemon = True
            cmd_thread.start()

        parts = []
        if t265_obj is not None:
            parts.append("速度帧 %dHz" % vel_freq)
        if comlist is not None:
            parts.append("指令帧 %dHz" % cmd_freq)
        logger.info("飞控串口发送线程启动（%s）", " + ".join(parts))

    def send_end(self):
        self.t265send_running = False
        self.cmdsend_running = False
        logger.info("飞控串口发送线程关闭")

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("飞控串口已关闭")

class Serial_dmz(object):
    def __init__(self,port,baudrate):
        self.ser=serial.Serial(port=port,baudrate=baudrate,timeout=0.05)
        self.dmzlisten_running=False
        self.dmzsend_running=False
        self.rate=115200
    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("目前地面站串口状态：%s",self.ser.is_open)
    def listen_start(self,rxbuffer:List[tuple]):
        self.dmzlisten_running=True
        listen_thread=threading.Thread(target=Serial_dmz.listen_dmz,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("地面站串口监听线程启动")
    def listen_end(self):
        self.dmzlisten_running=False
        logger.info("地面站串口监听线程关闭")
    def listen_dmz(self, rxbuffer: List[tuple]):
        while self.dmzlisten_running == True:
            byte_data = self.ser.read()
            if byte_data == b'\xAA':
                recv = self.ser.read(7)

                if len(recv) < 7:
                    continue

                if recv[6] == 0xFF:
                    with lock:
                        rxbuffer.clear()
                        for i in range(3):
                            a_val = recv[i * 2] - 0xA0
                            b_val = recv[i * 2 + 1] - 0xB0

                            a_str = f"A{a_val}"
                            b_str = f"B{b_val}"
                            rxbuffer.append((a_str, b_str))

                        if DEBUG:
                            logger.info("地面站反传禁飞区坐标: %s", rxbuffer)

            time.sleep(0.05)
    def send_dmz(self,comlist:List[int]):
        while self.dmzsend_running==True:
            self.ser.write(bytes(comlist))  # 单次发送整帧
            time.sleep(0.1)
    def send_start(self,comlist:List[int]):
        self.dmzsend_running=True
        dmzsend_thread=threading.Thread(target=Serial_dmz.send_dmz,args=(self,comlist))
        dmzsend_thread.daemon=True
        dmzsend_thread.start()
        logger.info("地面站串口发送线程启动")
    def send_end(self):
        self.dmzsend_running=False
        logger.info("地面站串口发送线程关闭")
    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("地面站串口已关闭")
