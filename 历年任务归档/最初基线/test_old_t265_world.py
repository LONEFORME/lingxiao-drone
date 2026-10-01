"""
T265 坐标变换测试 (世界锁死 + 旧t265轴系)
============================================

坐标系定义 (旧t265 规定):
  x = 前 (T265 tx.z)
  y = 右 (T265 tx.x)
  z = 上 (T265 tx.y)

去掉偏航校正, 世界锁死: 旋转无人机不影响坐标值。

无杠杆臂补偿。

使用方法:
  # Pi 上接 T265 运行
  python test_old_t265_world.py --live

  # 看仿真
  python test_old_t265_world.py
"""

import numpy as np
import math
import time

# ===================== 核心转换 =====================

def transform_world_locked(tx, tv):
    """
    世界锁死 + 旧t265轴系

    T265 原生: tx = [right, up, forward]
    输出:      x = forward (tz), y = right (tx), z = up (ty)
    """
    pos = np.array([
        -tx[2],     # tz → x (前)
        -tx[0],     # tx → y (右)
        +tx[1]      # ty → z (上)
    ])
    vel = np.array([
        -tv[2],
        -tv[0],
        +tv[1]
    ])
    return pos, vel


# ===================== 测试用例 (仿真用) =====================

test_cases = [
    # (名称, T265原生tx[right, up, forward], tv, yaw度, 说明)
    ("T1 悬停 1m",  [0,   1.0, 0],   [0, 0, 0],   0,  "基线: x=0 y=0 z=1"),
    ("T2 前进 5m",  [0,   1.0, 5.0], [0, 0, 1.0], 0,  "-tz→x: x≈-5"),
    ("T3 右移 5m",  [5.0, 1.0, 0],   [1.0,0, 0],  0,  "-tx→y: y≈-5"),
    ("T4 上升 2m",  [0,   2.0, 0],   [0,0.5,0],   0,  "ty→z: z≈2"),
    ("T5 转90°前进", [0, 1.0, 5.0], [0, 0, 0],   90, "世界锁死: 同T2"),
    ("T6 转45°前进", [0, 1.0, 5.0], [0, 0, 0],   45, "世界锁死: 同T2"),
]


def make_quat(yaw_deg):
    rad = math.radians(yaw_deg)
    return np.array([math.cos(rad/2), 0, 0, math.sin(rad/2)])


def run_simulation():
    print("=" * 70)
    print("  T265 坐标变换测试 (世界锁死 + 旧t265轴系)")
    print("  输出: x=前(tz)  y=右(tx)  z=上(ty)")
    print("=" * 70)

    for name, tx_raw, tv, yaw_deg, note in test_cases:
        tx_arr = np.array(tx_raw, float)
        tv_arr = np.array(tv, float)

        pos, vel = transform_world_locked(tx_arr, tv_arr)

        print(f"\n  {name}")
        print(f"    位置: x={pos[0]:+.3f}  y={pos[1]:+.3f}  z={pos[2]:+.3f}")
        print(f"    速度: vx={vel[0]:+.3f}  vy={vel[1]:+.3f}  vz={vel[2]:+.3f}")
        print(f"    → {note}")

    print("\n" + "=" * 70)


def quat_to_aero_yaw(qw, qx, qy, qz):
    """
    从 T265 四元数提取航空坐标系偏航角 (弧度)

    经过 H_aeroRef_T265Ref 相似变换后,
    在 aero 坐标系 (x=右, y=前, z=下) 下的 yaw:
      yaw = atan2(2*(x*z - w*y), 1 - 2*(y*y + z*z))
    """
    return math.atan2(2*(qx*qz - qw*qy), 1 - 2*(qy*qy + qz*qz))


def run_live():
    """Pi 上接真实 T265 运行"""
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("[!] Pi 上需要 pyrealsense2")
        return

    print("初始化 T265 ...")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
    pipe.start(cfg)

    for _ in range(10):
        pipe.wait_for_frames()

    print(f"\n{'=' * 90}")
    print(f" T265 Live — 世界锁死  旧t265轴系: x=前(tz)  y=右(tx)  z=上(ty)")
    print(f"{'=' * 90}")
    print(f"{'帧号':<6} {'x(前)':>8} {'y(右)':>8} {'z(上)':>8}   "
          f"{'vx':>7} {'vy':>7} {'vz':>7}   {'yaw':>7}")
    print(f"{'─' * 68}")
    print(" 按 Ctrl+C 退出\n")

    frame_count = 0
    initial_yaw = None
    CURSOR_UP = "\033[F"
    LINES = 3

    try:
        while True:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if not pose:
                continue
            data = pose.get_pose_data()
            tx = np.array([data.translation.x, data.translation.y, data.translation.z], float)
            tv = np.array([data.velocity.x, data.velocity.y, data.velocity.z], float)
            qw, qx, qy, qz = data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z

            yaw = quat_to_aero_yaw(qw, qx, qy, qz)
            if initial_yaw is None:
                initial_yaw = yaw
                yawerr = 0.0
                print(f"\n  初始偏航锁定: {math.degrees(initial_yaw):.1f}°")
                continue

            yawerr = -(yaw - initial_yaw)
            while yawerr > math.pi:  yawerr -= 2*math.pi
            while yawerr < -math.pi: yawerr += 2*math.pi

            frame_count += 1
            pos, vel = transform_world_locked(tx, tv)

            if frame_count > 1:
                for _ in range(LINES):
                    print(CURSOR_UP, end="")

            print(f"{frame_count:<6} {pos[0]:>8.3f} {pos[1]:>8.3f} {pos[2]:>8.3f}   "
                  f"{vel[0]:>7.3f} {vel[1]:>7.3f} {vel[2]:>7.3f}   "
                  f"{math.degrees(yawerr):>7.1f}°")
            print(f"{'':6} {'':>8} {'':>8} {'':>8}   "
                  f"{'':>7} {'':>7} {'':>7}   "
                  f"  前推=x增  右推=y增  上抬=z增")
            print(f"{'─' * 68}")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        pipe.stop()
        print("T265 已停止")


# ===================== 主入口 =====================

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="T265 世界锁死 + 旧t265轴系")
    parser.add_argument("--live", action="store_true", help="接真实 T265")
    args = parser.parse_args()

    if args.live:
        run_live()
    else:
        run_simulation()
