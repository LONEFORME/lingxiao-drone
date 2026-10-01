"""不起飞的协议连接测试：只监听飞控下行帧，验证帧2(0x02)扩展到19字节后
Python侧能正常解析出motor_pwm_mask字段，不涉及任何解锁/起飞指令。

用法：
    DRONE_FC_PORT=/dev/ttyS6 python3 check_frame2_protocol.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.Lprotocol import Serial_fc

port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
serial_fc = Serial_fc(port, 460800)
re_fc = [0] * 14
serial_fc.listen_start(re_fc)

print(f"监听 {port}，观察 {15} 秒...")
t_start = time.time()
last_debug = None
frame1_seen = False
while time.time() - t_start < 15:
    if len(re_fc) > 5:
        frame1_seen = True
    if serial_fc.debug_data and serial_fc.debug_data != last_debug:
        last_debug = dict(serial_fc.debug_data)
        print(f"[{time.time()-t_start:5.1f}s] debug_data = {last_debug}")
    time.sleep(0.2)

print()
print("=" * 40)
print("frame1 (0x01) 收到过:", frame1_seen, "re_fc =", re_fc)
print("frame2 (0x02) 最后一次 debug_data:", last_debug)
if last_debug and "motor_pwm_mask" in last_debug:
    print("[OK] motor_pwm_mask 字段已正常解析出来")
else:
    print("[FAIL] 没有收到带 motor_pwm_mask 的帧2数据——检查固件是否已重新烧录")

serial_fc.listen_end()
