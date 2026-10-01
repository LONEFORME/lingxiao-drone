# 机载感知 · 机载雷达感知层

> **来源**：自 `rdk-x5-flight-system`（2026-10-01 并入）迁移的镭神 N10P 雷达感知工程。

## 内容

`basic_radar/` —— 镭神 N10P 雷达感知与立柱测距实战工程：

- `static_pole_check.py` — 地面静态测距锁定工具（真机/仿真 `--mock`）
- `radar_bench_test.py` — 板载串口压测与吞吐量验证
- `laser_height_monitor.py` / `yaw_monitor.py` — 激光定高与偏航监视
- `main.py` + 自带 `Lcode/` — 雷达任务主程序（含当时版本的协议驱动副本）
- `test_*.py` — pytest 单测套件

## 与兄弟目录的关系

- **避障算法上游库**：`../../`（本仓）外的 `04_激光雷达SLAM避障/2d激光雷达镭神/`——`Lradar.py`（免 ROS 直驱）与 `pole_tracker.py`（世界系滑窗立柱验证，带 16 项 pytest）。**改 PoleTracker/避障逻辑在上游仓改并跑绿单测**，本目录是机载落地侧。
- **现役任务**：2026-D 主任务在 `../机载上位机/competition_2026_d/`；历史练兵工程在 `../../历年任务归档/`。
- **T265 看门狗**：`../部署/watchdog/t265_monitor.py`（追踪置信度监视与硬件级复位自愈，板端以 systemd 服务部署）。
