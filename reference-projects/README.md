# 历年电赛与外部参考工程 (Reference Projects)

> 本目录单独收纳历年高校电赛优秀参考方案与机械结构设计，供算法选型、结构设计与方案对比参考。

---

## 📁 历年参考工程一览

```
reference-projects/
├── README.md               # 本索引说明文件
│
├── 2024-NUEDC-D/           # 2024 年全国大学生电子设计竞赛 D 题
│   └── NUEDC-2024-D-main/  # 视觉跟踪与目标处理主工程
│       ├── ANO_LX/         # 基于凌霄飞控的底层适配
│       └── README.md       # 主工程说明
│
├── 2022-HUST/              # 华中科技大学 2022 年电赛经典开源方案
│   ├── DriversBsp/         # 硬件驱动包
│   ├── DriversMcu/         # STM32F407 驱动
│   ├── FcSrc/              # 飞控核心控制
│   ├── ProjectSTM32F407/   # Keil 工程
│   ├── python_sdk/         # 机载 Python SDK（含串口通信、激光雷达与 GUI 调试面板）
│   └── readme.md           # 方案说明
│
└── UAV-2023/               # 2023 年无人机电赛项目
    └── UAV_2023-master/    # 工程主体
        ├── UAV-Code/       # 飞控与任务代码
        ├── MichanicalSolution/ # 完整机械设计资料（SolidWorks 零件模型与 3D 打印 STL 文件）
        ├── LICENSE         # 开源许可
        └── README.md       # 工程说明
```

---

## 📌 参考指南

1. **机械结构设计**：如需 3D 打印件或机架载荷改装，可重点参考 [`UAV-2023/UAV_2023-master/MichanicalSolution/`](UAV-2023/UAV_2023-master/MichanicalSolution/)。
2. **激光雷达与地面站 GUI**：如需使用 Python 搭建可视化上位机，可重点参考 [`2022-HUST/python_sdk/`](2022-HUST/python_sdk/)。
3. **自研主方案**：最新的自研实战闭环工程请使用根目录下的 [`drone-system/`](../drone-system/)。
