"""T265 转盘纯旋转台架测试 — 只测T265本身，不涉及飞控/电机，不需要解锁飞机。

用途：验证T265在纯旋转（无平移，绕偏离T265本身一定半径的轴心转动）时，
(x,y)位置追踪是否内部自洽——即随着T265自己汇报的yaw变化，(x,y)轨迹
是否精确落在一个圆上(半径=T265到转轴的距离)。不需要转盘有精确角度刻度，
只需要在采集期间手动来回转动转盘、覆盖尽量大的角度范围。

2026-07-09第一次采集发现：慢速旋转自洽性良好，但快速旋转(78-91°/s)瞬间
残差明显变大。t265.py对x/y/z做了低通滤波(alpha=0.15)但yaw不滤波，无法区分
"纯粹是滤波滞后"还是"T265的VIO在快速旋转时真实存在追踪误差"。这次同时记录
低通滤波前的原始位置/速度(t265.py新增的get_raw_position()/get_raw_velocity()，
纯诊断用，不影响飞控闭环用的get_position()/get_velocity())，如果滤波后残差异常
但原始位置依然自洽，说明纯粹是我们自己的滤波滞后；如果原始位置同样异常，
说明T265的VIO本身在快速旋转时就有问题。

用法:
  python t265_turntable_log.py [采集秒数，默认90]

采集期间请手动缓慢转动转盘，务必让总转动范围覆盖大角度（建议单次尽量转足
180°以上，最好接近一整圈）——圆拟合对角度覆盖范围很敏感：实测覆盖不到60°
时拟合半径能偏差10%+，覆盖300°时偏差可以做到<1mm。来回小角度摆动不会
提升拟合质量，重点是转动的总角度跨度要大，不是转动次数多。转动间隙也保持
静止几秒方便后续区分静止段/旋转段；这次还请专门做几段明显更快的转动
（比之前更快甩一下再停），这样才有足够的快转样本用来对比滤波前后的差异。

输出：turntable_log_<时间戳>.jsonl，每行一条记录：
  {"t": 相对秒数, "x"/"y"/"z": m(滤波后), "raw_x"/"raw_y"/"raw_z": m(滤波前),
   "yaw_deg": 度, "vx"/"vy": m/s(滤波后), "raw_vx"/"raw_vy": m/s(滤波前), "confidence": 0-3}
"""
import sys
import time
import json
import math
from t265 import t265_class
from Lcode.Logger import logger


def main():
    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0

    realsense = t265_class()
    realsense.start()

    logger.info("等待T265追踪置信度达标...")
    t0 = time.time()
    while time.time() - t0 < 8.0:
        if realsense.get_tracking_confidence() >= 2:
            break
        time.sleep(0.1)
    conf = realsense.get_tracking_confidence()
    logger.info("T265追踪置信度=%d，开始采集，时长%.0f秒", conf, duration_s)

    out_path = f"turntable_log_{int(time.time())}.jsonl"
    start_t = time.time()
    n = 0
    with open(out_path, "w", encoding="utf-8") as f:
        while time.time() - start_t < duration_s:
            # 所有字段必须在同一次加锁内一起读出，否则多次独立调用中间背景线程
            # 可能已更新数据，导致同一条记录里各字段来自不同时刻。
            # 直接读 pose_data 而不经过 get_position()（后者会再减 calibration_offset）；
            # 本脚本从不调用 autoset()，calibration_offset 恒为[0,0,0]，两者等价。
            with realsense.lock:
                x, y, z, _roll, _pitch, yaw = realsense.pose_data.tolist()
                raw_x, raw_y, raw_z = realsense.raw_pose_data.tolist()
                vx, vy, _vz = realsense.velocity_data.tolist()
                raw_vx, raw_vy, _raw_vz = realsense.raw_velocity_data.tolist()
                conf = int(realsense.last_confidence)
            rec = {
                "t": round(time.time() - start_t, 3),
                "x": round(float(x), 4),
                "y": round(float(y), 4),
                "z": round(float(z), 4),
                "raw_x": round(float(raw_x), 4),
                "raw_y": round(float(raw_y), 4),
                "raw_z": round(float(raw_z), 4),
                "yaw_deg": round(math.degrees(yaw), 2),
                "vx": round(float(vx), 4),
                "vy": round(float(vy), 4),
                "raw_vx": round(float(raw_vx), 4),
                "raw_vy": round(float(raw_vy), 4),
                "confidence": conf,
            }
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            n += 1
            time.sleep(0.05)  # 约20Hz

    realsense.stop()
    logger.info("采集完成，共%d条记录，已保存到 %s", n, out_path)


if __name__ == "__main__":
    main()
