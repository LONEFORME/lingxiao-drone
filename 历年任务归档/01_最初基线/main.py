import os,sys
from Lcode.global_variable import sp_side, lock
import Lcode.Lprotocol
from Lcode.k230_client import K230Client
import time
from Lcode.Logger import logger

from Mission_GPT import mission
from t265 import t265_class

# ======================== 变量 ===========================
re_fc = [0] * 14  # [mission_stage, roll_x100, pitch_x100, yaw_x100, fusion_state, unlock_sta, integral_x, integral_y, laser_cm, of1_dx, of1_dy, of_quality, of_link_sta, of_work_sta]
re_dmz = [('A9', 'B1'),('A10', 'B2'),('A11', 'B3')]#地面站反传信息 三个禁飞区坐标/x/y

# 双帧协议: AA 02 task_sta com_x+sp com_y+sp com_z com_yaw+sp next_task sp_side CK FF
# com_z(索引5)占位为0：takeoff()会在触发task_sta前覆写为本次真正的目标高度，
# 这里绝不能填非0默认值——2026-07-06 曾因这里写死120导致一键起飞冲到1.2m（见CLAUDE.md已知问题6）
se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]
se_dmz = [0xAA, 0, 0xFF, 0, 0xFF]  # AA idx cls cnt FF — AA/FF始终有效，cls=FF+cnt=0=飞行中

run_sign = False

# ===================== 提前创建 T265 =====================
realsense = t265_class()

# ===================== 串口初始化 =====================
# 环境变量覆盖（可选，不设置则使用默认值）：
#   Linux/树莓派: export DRONE_FC_PORT=/dev/ttyS6   # 飞控
#                 export DRONE_DMZ_PORT=/dev/ttyS7   # 地面站
#                 export DRONE_K230_PORT=/dev/ttyS3  # K230视觉板
serial_fc = Lcode.Lprotocol.Serial_fc(os.getenv("DRONE_FC_PORT", "/dev/ttyS6"), 460800)
serial_fc.port_open()
serial_fc.listen_start(re_fc)
serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)  # 速度帧100Hz + 指令帧50Hz

serial_dmz = Lcode.Lprotocol.Serial_dmz(os.getenv("DRONE_DMZ_PORT", "/dev/ttyS7"), 115200)
serial_dmz.port_open()
serial_dmz.listen_start(re_dmz)
serial_dmz.send_start(se_dmz)

# ===================== K230 视觉板通信 =====================
k230 = K230Client(os.getenv("DRONE_K230_PORT", "/dev/ttyS3"), 115200)

# ==================== 只创建一次任务！====================
# 传递 serial_fc 引用，让 mission 可查询激光高度
mission1 = mission(re_fc, se_fc, re_dmz, se_dmz, realsense, k230, serial_fc)

while(1):
    if not run_sign:
        mission1.start()
        run_sign = True

    time.sleep(0.1)