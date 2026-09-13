# 凌霄飞控无人机 — 官方基线、自研系统与电赛参考库

> **匿名凌霄飞控（ANO_LX）全栈工程资料库**  
> 融合 **官方原本基线与开发指南**、**全套自研闭环无人机系统（含作者实机飞行测试实录）** 以及 **历年电赛优秀参考工程**。

---

## 🏛️ 仓库三大支柱架构

本仓库按研发与实战需求严格解耦为三大模块：

```
lingxiao-drone/
├── 📚 official-guide/                  # 【支柱一：官方原本资料与开发指南】
│   ├── docs/                           # 官方手册、通信协议V7(权威版)、原理图PCB、IMU固件
│   ├── firmware-baseline/              # 凌霄官方纯净源码基线 (支持 STM32F407 / MSP432 / TM4C)
│   ├── examples/                       # 官方入门基础例程 (起飞降落、一键航线任务)
│   └── tools-env/                      # Keil MDK 安装包、J-Link 驱动、匿名上位机与环境配置手册
│
├── 🛸 drone-system/                    # 【支柱二：自研完整无人机实战系统】
│   ├── flight-controller/              # STM32F407 飞控源码（硬件级倾角防侧翻切断保护 + 自定义协议）
│   ├── edge-vision/                    # 01Studio K230 (CyberCAM) 边缘视觉目标检测 (AprilTag+色块)
│   ├── companion-computer/             # RDK X5 / 树莓派机载自主控制系统 (T265双目空间定位 + 状态机)
│   ├── config/                         # 引脚映射、空地蓝牙通信参数与开机自启动服务
│   └── demos/                          # 🎬 作者实测飞行演示视频库（二维码降落/火源/绕杆/精准着陆）
│
└── 🏆 reference-projects/              # 【支柱三：历年电赛与外部参考工程】
    ├── 2024-NUEDC-D/                   # 2024 年全国电赛 D 题主程序与飞控适配
    ├── 2022-HUST/                      # 华中科技大学 2022 经典方案（飞控 + Python SDK + 雷达避障）
    └── UAV-2023/                       # 2023 年无人机项目（含机械结构 SolidWorks / 3D打印 STL）
```

---

## 🎬 作者实测飞行演示 (Flight Demos)

以下均为**作者自研无人机系统实机飞行实录**，展示了自主识别、避障巡航、动态跟随与精准降落的全闭环表现：

| 演示项目 | 实测场景与核心技术 | 规格 | 视频文件 |
| :--- | :--- | :--- | :--- |
| **二维码识别降落** | 机载相机实时解算 AprilTag/二维码空间位姿，微调航向平稳着陆 | 720×1280 竖屏 (32MB) | [`drone-system/demos/二维码.mp4`](drone-system/demos/二维码.mp4) |
| **火源定位与处理** | 下视视觉色域自适应分割，高空悬停并执行目标处置 | 720×1280 竖屏 (11MB) | [`drone-system/demos/火源.mp4`](drone-system/demos/火源.mp4) |
| **自主绕杆避障** | T265 双目 V-SLAM 空间高精定位与连续避障航点平滑跟踪 | 720×406 横屏 (9MB) | [`drone-system/demos/绕杆.mp4`](drone-system/demos/绕杆.mp4) |
| **移动平台动态降落** | 动态锁定移动靶标小车，自适应地面效应与气流完成平稳着陆 | 960×720 横屏 (10MB) | [`drone-system/demos/降落.mp4`](drone-system/demos/降落.mp4) |

---

## 🚀 快速上手与使用指引

### 1. 新手入门与底层开发（看 `official-guide/`）
如果您是首次接触匿名凌霄飞控，或者需要查阅芯片原理图、原生通信协议：
- 阅读 [官方原本资料与开发指南](official-guide/README.md)；
- 查看 [`official-guide/tools-env/开发环境配置说明.md`](official-guide/tools-env/开发环境配置说明.md) 搭建 Keil MDK 与 J-Link 驱动；
- 打开 [`official-guide/firmware-baseline/`](official-guide/firmware-baseline/) 编译纯净官方源码；
- 学习 [`official-guide/examples/`](official-guide/examples/) 体验起飞与定高降落。

### 2. 实战部署自研系统（看 `drone-system/`）
如果您需要一套真正能在竞赛或实机测试中自主飞行的全套方案：
- 阅读 [自研完整无人机实战系统手册](drone-system/README.md)；
- 硬件连接与通信拓扑：STM32F407 飞控 (`/dev/ttyS1`) + K230 边缘相机 (`/dev/ttyS7`) + T265 双目定位 + 蓝牙空地链路 (`/dev/bt_serial`)；
- 进入 [`drone-system/companion-computer/`](drone-system/companion-computer/) 运行自主巡航状态机。

### 3. 高校电赛方案借鉴（看 `reference-projects/`）
如果您需要参考往届高校参赛思路或机械图纸：
- 查阅 [历年电赛参考工程索引](reference-projects/README.md)；
- 机械结构与 3D 打印件：参考 [`reference-projects/UAV-2023/MichanicalSolution/`](reference-projects/UAV-2023/MichanicalSolution/)；
- 地面站 GUI 与雷达避障：参考 [`reference-projects/2022-HUST/python_sdk/`](reference-projects/2022-HUST/python_sdk/)。

---

## ⚠️ 注意事项

1. **编译产物过滤**：本仓库已在 `.gitignore` 中配置过滤 Keil 中间编译产物（`.o`、`.axf`、`.d` 等）及 Python 缓存（`__pycache__`），保持代码库极度轻量整洁。
2. **大文件说明**：Keil DFP 支持包由于体积限制已通过 `.gitignore` 排除，下载地址详见环境说明文档。
3. **IMU 固件匹配**：119 版本以上 IMU 固件使用新版控制帧，请务必注意上位机与飞控固件版本协同。

---

## 📝 相关链接

- 匿名科技官网：http://www.anotc.com/
- GitHub 组织：https://github.com/LONEFORME
