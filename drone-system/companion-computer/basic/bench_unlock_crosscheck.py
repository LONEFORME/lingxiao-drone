"""台架交叉验证：纯监听飞控下行帧，持续打印unlock_sta/motor_pwm_mask变化，
配合官方匿名上位机的"飞控状态"页同时观察，核对两边解锁状态是否一致。
不发送任何解锁/起飞指令，零风险，可以直接跑。

用法：
    DRONE_FC_PORT=/dev/ttyS6 python3 bench_unlock_crosscheck.py [持续秒数，默认60]
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.Lprotocol import Serial_fc

port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

serial_fc = Serial_fc(port, 460800)
re_fc = [0] * 14
serial_fc.listen_start(re_fc)

print(f"监听 {port}，持续 {duration:.0f} 秒——现在可以在遥控器上做解锁/上锁动作，"
      f"同时观察匿名上位机'飞控状态'页的锁定状态字段。")

t_start = time.time()
prev = (None, None)
while time.time() - t_start < duration:
    unlock_sta = re_fc[5] if len(re_fc) > 5 else None
    pwm = serial_fc.debug_data.get("motor_pwm_mask") if serial_fc.debug_data else None
    key = (unlock_sta, pwm)
    if key != prev:
        print(f"[{time.time()-t_start:6.2f}s] unlock_sta={unlock_sta} motor_pwm_mask={pwm}")
        prev = key
    time.sleep(0.03)

serial_fc.listen_end()
print("监听结束")
