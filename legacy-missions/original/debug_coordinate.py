"""
T265 坐标变换调试工具
=====================

两种模式:
  1) 模拟模式 (默认): python debug_coordinate.py
     用预设的测试用例对比三种变换

  2) 实况模式:       python debug_coordinate.py --live
     接真实 T265, 物理移动相机看三种变换的实时输出

三种变换:
  A) DataDeal.py 风格 — 偏航校正, z 朝上
  B) 旧 t265.py 风格  — 偏航校正, z 朝上, 整体取反
  C) 新 t265.py 风格  — 世界锁死 NED, z 朝下

操作建议 (实况模式):
  1) 把相机平放桌上, 镜头朝前 → 这就是初始前方 (tz 方向)
  2) 运行 --live, 观察初始读数 (InitialAngle 锁定)
  3) 手动向前推相机 → 看哪个变换的 x 增加、y≈0
  4) 手动向右推相机 → 看哪个变换的 x≈0、y 增加
  5) 水平旋转相机 90° → 再向前推 → 看偏航校正是否起作用
"""

import numpy as np
import math
import sys
import time
import argparse
from collections import OrderedDict

try:
    import transformations as tf
except ImportError:
    print("[!] 未找到 transformations 模块, 请先在 pi 上安装: pip install transformations")
    sys.exit(1)

# ============ 常数矩阵 (三个版本共用) ============
H_aeroRef_T265Ref = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
], dtype=float)
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)


def compute_rpy_and_yawerr(qua, initial_yaw):
    """根据四元数 → 欧拉角 → yawerr"""
    H = tf.quaternion_matrix(qua)
    H_aero = H_aeroRef_T265Ref @ H @ H_T265body_aeroBody
    rpy = np.array(tf.euler_from_matrix(H_aero, 'sxyz'))

    yawerr = -(rpy[2] - initial_yaw)
    # 归一化到 [-PI, PI]
    while yawerr > math.pi:   yawerr -= 2*math.pi
    while yawerr < -math.pi:  yawerr += 2*math.pi
    return rpy, yawerr


def make_quat(yaw_deg):
    """仅绕 Z 轴旋转的四元数 [w,x,y,z] (roll=pitch=0)"""
    rad = math.radians(yaw_deg)
    return np.array([math.cos(rad/2), 0, 0, math.sin(rad/2)])


# ===================== 三种变换 =====================

def transform_A_DataDeal(tx, tv, qua, initial_yaw):
    """DataDeal.py: 偏航校正 + 不取反 + z朝上"""
    rpy, yawerr = compute_rpy_and_yawerr(qua, initial_yaw)
    cy, sy = math.cos(yawerr), math.sin(yawerr)

    pos = np.array([tx[2]*cy + tx[0]*sy,   # tz*cos + tx*sin
                    -tx[2]*sy + tx[0]*cy,   # -tz*sin + tx*cos
                    tx[1]])                 # +ty 朝上
    vel = np.array([tv[2]*cy + tv[0]*sy,
                    -tv[2]*sy + tv[0]*cy,
                    tv[1]])
    return pos, vel, yawerr


def transform_B_old_t265(tx, tv, qua, initial_yaw):
    """旧 t265.py: 偏航校正 + 整体取反 + z朝上"""
    rpy, yawerr = compute_rpy_and_yawerr(qua, initial_yaw)
    cy, sy = math.cos(yawerr), math.sin(yawerr)

    pos = np.array([-(tx[2]*cy + tx[0]*sy),     # DataDeal_x 取反
                    -(-tx[2]*sy + tx[0]*cy),     # DataDeal_y 取反
                    tx[1]])                      # +ty 朝上 (同 DataDeal)
    vel = np.array([-(tv[2]*cy + tv[0]*sy),
                    -(-tv[2]*sy + tv[0]*cy),
                    tv[1]])
    return pos, vel, yawerr


def transform_C_new_t265(tx, tv, qua, initial_yaw):
    """新 t265.py: 无偏航校正 + 世界锁死 NED + z朝下"""
    rpy, yawerr = compute_rpy_and_yawerr(qua, initial_yaw)

    pos = np.array([tx[2],     # tz → NED x (前)
                    tx[0],     # tx → NED y (右)
                    -tx[1]])   # -ty → NED z (下)
    vel = np.array([tv[2],
                    tv[0],
                    -tv[1]])
    return pos, vel, yawerr


# ============= 可调参数的自定义变换 D =============

def transform_D_custom(tx, tv, qua, initial_yaw,
                       yaw_correct=True, sign=1, z_up=True):
    """
    参数化的自定义变换 (组合了所有变量)

    Parameters:
        yaw_correct: True=偏航校正, False=世界锁死
        sign: +1 或 -1 (全局 X/Y 符号)
        z_up: True=z朝上(+ty), False=z朝下(-ty, NED)

    8 种组合 (2×2×2):
        组合编号  yaw_correct  sign  z_up    对应已知变换
         ①         True        +1   True  = A (DataDeal)
         ②         True        -1   True  = B (旧t265)
         ③         True        +1   False   (偏航校正+z朝下)
         ④         True        -1   False   (偏航校正+取反+z朝下)
         ⑤         False       +1   True    (世界锁死+z朝上)
         ⑥         False       -1   True    (世界锁死+取反+z朝上)
         ⑦         False       +1   False  = C (新t265 NED)
         ⑧         False       -1   False   (世界锁死+取反+z朝下)
    """
    rpy, yawerr = compute_rpy_and_yawerr(qua, initial_yaw)

    if yaw_correct:
        cy, sy = math.cos(yawerr), math.sin(yawerr)
        px = sign * (tx[2]*cy + tx[0]*sy)
        py = sign * (-tx[2]*sy + tx[0]*cy)
        vx = sign * (tv[2]*cy + tv[0]*sy)
        vy = sign * (-tv[2]*sy + tv[0]*cy)
    else:
        px = sign * tx[2]
        py = sign * tx[0]
        vx = sign * tv[2]
        vy = sign * tv[0]

    pz = tx[1] if z_up else -tx[1]
    vz = tv[1] if z_up else -tv[1]

    return np.array([px, py, pz]), np.array([vx, vy, vz]), yawerr


def describe_custom(yaw_correct, sign, z_up):
    """返回自定义变换的参数描述"""
    parts = []
    parts.append("偏航校正" if yaw_correct else "世界锁死")
    parts.append("取反" if sign == -1 else "正向")
    parts.append("z↑" if z_up else "z↓(NED)")
    return " ".join(parts)


def preset_name(yaw_correct, sign, z_up):
    """如果当前参数匹配某个已知变换, 返回名字"""
    if   yaw_correct and sign ==  1 and  z_up: return "= A (DataDeal)"
    elif yaw_correct and sign == -1 and  z_up: return "= B (旧t265)"
    elif yaw_correct and sign == -1 and not z_up: return "≈ B但z↓"
    elif yaw_correct and sign ==  1 and not z_up: return "≈ A但z↓"
    elif not yaw_correct and sign ==  1 and not z_up: return "= C (新t265)"
    elif not yaw_correct and sign == -1 and not z_up: return "新t265取反"
    elif not yaw_correct and sign == -1 and  z_up: return "世界锁死取反+z↑"
    elif not yaw_correct and sign ==  1 and  z_up: return "世界锁死+z↑"
    return ""


def param_label(yaw_correct, sign, z_up):
    """紧凑参数标签, 适合表格头"""
    c = "Y" if yaw_correct else "W"
    s = "S+" if sign == 1 else "S-"
    z = "U" if z_up else "D"
    return f"D.{c}{s}{z}"


# ===================== 测试用例 =====================
# tx = [T265_x, T265_y, T265_z] = [右, 上, 前]
# tv = [vx, vy, vz]
# yaw_deg: 飞机相对于 T265 世界坐标的偏航角

test_cases = [
    ("T1 悬停 1m",      [0,   1.0, 0],   [0, 0, 0],   0,
     "z 应反映高度"),
    ("T2 前进 5m",      [0,   1.0, 5.0], [0, 0, 1.0], 0,
     "无转弯前进: x>0 y≈0 为正确"),
    ("T3 右移 5m",      [5.0, 1.0, 0],   [1.0,0, 0],  0,
     "无转弯右移: x≈0 y>0 为正确"),
    ("T4 右转90°前进",  [0,   1.0, 5.0], [0, 0, 1.0], 90,
     "朝东飞. 世界tz=5(北). 偏航校正版x=初始前方分量"),
    ("T5 右转90°右移",  [5.0, 1.0, 0],  [1.0, 0, 0], 90,
     "朝东,世界tx=5(东). 偏航校正版从tx提取分量"),
    ("T6 上升 2m",      [0,   2.0, 0],   [0, 0.5,0],  0,
     "A/B: z>0,  C: z<0 (NED)"),
    ("T7 右转45°混动",  [3.0, 1.0, 5.0],[0.5,0, 1.0],45,
     "偏航校正版 x=初始前方分量, y=初始右侧分量"),
]


def print_header():
    print("=" * 110)
    print("  T265 坐标变换对比调试工具")
    print("  假设: initial_yaw = 0 (起飞时对准 T265 深度方向)")
    print("=" * 110)
    print(f"{'':6} {'方法':<10} {'x':>8} {'y':>8} {'z':>8}  "
          f"{'vx':>7} {'vy':>7} {'vz':>7}  {'yawerr':>7}")
    print("-" * 110)


def print_result(label, pos, vel, yawerr, note=""):
    print(f"{'':6} {label:<10} {pos[0]:>8.3f} {pos[1]:>8.3f} {pos[2]:>8.3f}  "
          f"{vel[0]:>7.3f} {vel[1]:>7.3f} {vel[2]:>7.3f}  "
          f"{math.degrees(yawerr):>7.1f}°  {note}")


def run_all(yaw_correct, sign, z_up):
    label_D = param_label(yaw_correct, sign, z_up)
    desc_D = describe_custom(yaw_correct, sign, z_up)
    preset_D = preset_name(yaw_correct, sign, z_up)

    print_header()

    for name, tx, tv, yaw_deg, note in test_cases:
        print(f"\n  【{name}】 yaw={yaw_deg}°  "
              f"tx=(tx={tx[0]},ty={tx[1]},tz={tx[2]})")
        print("-" * 110)

        qua = make_quat(yaw_deg)
        tx_a = np.array(tx, dtype=float)
        tv_a = np.array(tv, dtype=float)

        posA, velA, yawA = transform_A_DataDeal(tx_a, tv_a, qua, 0.0)
        posB, velB, yawB = transform_B_old_t265(tx_a, tv_a, qua, 0.0)
        posC, velC, yawC = transform_C_new_t265(tx_a, tv_a, qua, 0.0)
        posD, velD, yawD = transform_D_custom(tx_a, tv_a, qua, 0.0,
                                               yaw_correct, sign, z_up)

        print_result("A.DataDeal", posA, velA, yawA, "偏航校正, z↑")
        print_result("B.旧t265",  posB, velB, yawB, "偏航校正+取反, z↑")
        print_result("C.新t265",  posC, velC, yawC, "世界锁死NED, z↓")
        print_result(label_D,     posD, velD, yawD, desc_D + " " + preset_D)

    print("\n" + "=" * 110)
    print("  当前自定义 D | " + desc_D + " " + preset_D)
    print("  ─────────────────────────────────────────────")
    print("  用以下参数调整 D, 找到你要的坐标系:")
    print("    --yaw-correct   偏航校正 (位置随偏航旋转, 保持初始前方)")
    print("    --no-yaw-correct世界锁死 (位置固定在世界坐标)")
    print("    --sign 1        正向 (x:右=正, 前=正)")
    print("    --sign -1       取反 (x:右=负, 前=负)")
    print("    --z-up          z 朝上 (+ty)")
    print("    --z-down        z 朝下 (-ty, NED 航空规范)")
    print("  ─────────────────────────────────────────────")
    print("  判断方法:")
    print("    T2 前进→ x>0 y≈0,  T3 右移→ x≈0 y>0")
    print("    T4 偏航校正版 x=初始前方分量, 世界锁死版 x=世界北")
    print("    找到理想的 D 后, 把参数告诉我, 我移植回 t265.py")


# ===================== 全组合扫描模式 =====================

def run_scan():
    """显示全部 8 种组合在 T2+T3+T4 下的结果, 方便一次性对比"""
    param_sets = []
    for yc in [True, False]:
        for sg in [1, -1]:
            for zu in [True, False]:
                param_sets.append((yc, sg, zu))

    t2 = test_cases[1]  # 前进 5m
    t3 = test_cases[2]  # 右移 5m
    t4 = test_cases[3]  # 右转90°前进

    print("=" * 110)
    print("  全组合扫描: 8 组参数 × 3 个关键测试")
    print("=" * 110)

    for case_name, tx, tv, yaw_deg, note in [t2, t3, t4]:
        print(f"\n  【{case_name}】")
        print(f"  {'组合':<10} {'参数':<28} {'x':>8} {'y':>8} {'z':>8}  "
              f"{'vx':>7} {'vy':>7} {'vz':>7}  {'匹配':<16}")
        print("-" * 110)

        qua = make_quat(yaw_deg)
        tx_a = np.array(tx, dtype=float)
        tv_a = np.array(tv, dtype=float)

        for idx, (yc, sg, zu) in enumerate(param_sets):
            pos, vel, _ = transform_D_custom(tx_a, tv_a, qua, 0.0,
                                              yc, sg, zu)
            label = param_label(yc, sg, zu)
            desc = describe_custom(yc, sg, zu)
            pre = preset_name(yc, sg, zu)
            print(f"  #{idx+1:<4} {label:<8} {desc:<28} "
                  f"{pos[0]:>8.3f} {pos[1]:>8.3f} {pos[2]:>8.3f}  "
                  f"{vel[0]:>7.3f} {vel[1]:>7.3f} {vel[2]:>7.3f}  {pre:<16}")

    print("\n" + "-" * 110)
    print("  找 T2 x>0 y≈0 且 T3 x≈0 y>0 的组合, 就是你要的坐标系")
    print("  然后把对应的 --yaw-correct/--no-yaw-correct --sign --z-up/--z-down 记下来")


# ===================== 实况模式 (接真实T265) =====================

def run_live(yaw_correct, sign, z_up):
    """读取真实 T265, 实时显示四种变换对比"""
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("[!] 实况模式需要 pyrealsense2, 请在 pi 上安装后重试")
        sys.exit(1)

    label_D = param_label(yaw_correct, sign, z_up)
    desc_D = describe_custom(yaw_correct, sign, z_up)
    preset_D = preset_name(yaw_correct, sign, z_up)

    print("初始化 T265 ...")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
    pipe.start(cfg)
    print("T265 已启动, 正在获取首帧校准 ...")

    initial_yaw = None
    frame_count = 0
    CURSOR_UP = "\033[F"  # ANSI: 上移一行

    for _ in range(10):
        pipe.wait_for_frames()

    print("开始实时显示 (按 Ctrl+C 退出)")
    print("=" * 110)
    print(f"  D: {desc_D} {preset_D}")
    print("=" * 110)
    print(f"{'帧号':<6} {'方法':<10} {'x':>8} {'y':>8} {'z':>8}  "
          f"{'vx':>7} {'vy':>7} {'vz':>7}  {'yawerr':>7}")
    print("-" * 110)
    print(f"{'':6} {'A.DataDeal':<10} ...")
    print(f"{'':6} {'B.旧t265':<10} ...")
    print(f"{'':6} {'C.新t265':<10} ...")
    print(f"{'':6} {label_D:<10} ...")
    # 实况占 7 行: 2标题 + 表头 + 分隔 + 4数据
    LIVE_HEIGHT = 7

    try:
        while True:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if not pose:
                continue

            data = pose.get_pose_data()
            tx = np.array([data.translation.x, data.translation.y, data.translation.z], dtype=float)
            tv = np.array([data.velocity.x, data.velocity.y, data.velocity.z], dtype=float)
            qua = np.array([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z], dtype=float)

            if initial_yaw is None:
                rpy0, _ = compute_rpy_and_yawerr(qua, 0.0)
                initial_yaw = rpy0[2]
                print(f"   InitialAngle_Yaw = {math.degrees(initial_yaw):.2f}°  已锁定")
                continue

            frame_count += 1

            posA, velA, yawA = transform_A_DataDeal(tx, tv, qua, initial_yaw)
            posB, velB, yawB = transform_B_old_t265(tx, tv, qua, initial_yaw)
            posC, velC, yawC = transform_C_new_t265(tx, tv, qua, initial_yaw)
            posD, velD, yawD = transform_D_custom(tx, tv, qua, initial_yaw,
                                                   yaw_correct, sign, z_up)

            if frame_count > 1:
                for _ in range(LIVE_HEIGHT):
                    print(CURSOR_UP, end="")

            print("=" * 110)
            print(f"  D: {desc_D} {preset_D}")
            print("-" * 110)
            print(f"{frame_count:<6} {'A.DataDeal':<10} "
                  f"{posA[0]:>8.3f} {posA[1]:>8.3f} {posA[2]:>8.3f}  "
                  f"{velA[0]:>7.3f} {velA[1]:>7.3f} {velA[2]:>7.3f}  {math.degrees(yawA):>7.1f}°")
            print(f"{frame_count:<6} {'B.旧t265':<10} "
                  f"{posB[0]:>8.3f} {posB[1]:>8.3f} {posB[2]:>8.3f}  "
                  f"{velB[0]:>7.3f} {velB[1]:>7.3f} {velB[2]:>7.3f}  {math.degrees(yawB):>7.1f}°")
            print(f"{frame_count:<6} {'C.新t265':<10} "
                  f"{posC[0]:>8.3f} {posC[1]:>8.3f} {posC[2]:>8.3f}  "
                  f"{velC[0]:>7.3f} {velC[1]:>7.3f} {velC[2]:>7.3f}  {math.degrees(yawC):>7.1f}°")
            print(f"{frame_count:<6} {label_D:<10} "
                  f"{posD[0]:>8.3f} {posD[1]:>8.3f} {posD[2]:>8.3f}  "
                  f"{velD[0]:>7.3f} {velD[1]:>7.3f} {velD[2]:>7.3f}  {math.degrees(yawD):>7.1f}°")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        pipe.stop()
        print("T265 已停止")


# ===================== 入口 =====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="T265 坐标变换调试工具")
    parser.add_argument("--live", action="store_true",
                        help="实况模式: 接真实 T265 显示实时对比")
    parser.add_argument("--scan", action="store_true",
                        help="扫描模式: 显示全部 8 种参数组合的对比")
    parser.add_argument("--yaw-correct", action="store_true", default=True,
                        help="启用偏航校正 (位置随偏航旋转) [默认]")
    parser.add_argument("--no-yaw-correct", action="store_true",
                        help="禁用偏航校正 (世界锁死)")
    parser.add_argument("--sign", type=int, choices=[1, -1], default=1,
                        help="全局符号: 1=正向 [默认], -1=取反")
    parser.add_argument("--z-up", action="store_true", default=True,
                        help="z 朝上 (+ty) [默认]")
    parser.add_argument("--z-down", action="store_true",
                        help="z 朝下 (-ty, NED 规范)")
    args = parser.parse_args()

    # 解析参数 (--no-xxx 覆盖 --xxx)
    yaw_correct = not args.no_yaw_correct if args.no_yaw_correct else args.yaw_correct
    z_up = not args.z_down if args.z_down else args.z_up

    if args.scan:
        run_scan()
    elif args.live:
        run_live(yaw_correct, sign=args.sign, z_up=z_up)
    else:
        run_all(yaw_correct, sign=args.sign, z_up=z_up)
