#!/usr/bin/env python3
"""T265 濮挎€佹暟鎹�璇诲彇 - 鏄剧ず浣嶇疆鍧愭爣鍜屾湞鍚�"""

import pyrealsense2 as rs
import time

# 鍒濆�嬪寲 pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)  # T265 鐨勫Э鎬佹祦

try:
    pipe.start(cfg)
    print("T265 宸插惎鍔�锛岀瓑寰呮暟鎹�...\n")
    
    while True:
        frames = pipe.wait_for_frames()
        pose_frame = frames.get_pose_frame()
        
        if pose_frame:
            pose = pose_frame.get_pose_data()
            
            # 浣嶇疆鍧愭爣 (鍗曚綅: 绫�)
            tx = pose.translation.x
            ty = pose.translation.y
            tz = pose.translation.z
            
            # 閫熷害 (绫�/绉�)
            vx = pose.velocity.x
            vy = pose.velocity.y
            vz = pose.velocity.z
            
            # 鍥涘厓鏁版湞鍚�
            qw = pose.rotation.w
            qx = pose.rotation.x
            qy = pose.rotation.y
            qz = pose.rotation.z
            
            # 缃�淇″害 (0-3, 3=楂�)
            confidence = pose.tracker_confidence
            
            print(f"\r浣嶇疆: x={tx:7.3f} y={ty:7.3f} z={tz:7.3f} (m) | "
                  f"閫熷害: vx={vx:6.3f} vy={vy:6.3f} vz={vz:6.3f} (m/s) | "
                  f"鍥涘厓鏁�: w={qw:.3f} x={qx:.3f} y={qy:.3f} z={qz:.3f} | "
                  f"缃�淇″害: {confidence}/3", end="")
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\n鍋滄��...")
finally:
    pipe.stop()