# 地面协同小车系统 (Ground Vehicle)

本系统为 2026 年电赛空地协同任务（省赛 D 题）的地面移动目标平台，负责精准循线行驶、分段速度规划、一键联动起飞以及向空中无人机和地面站转发状态。

---

## 目录结构

```text
ground-vehicle/
├── gateway/                     # 车载通信网关（树莓派 Python 3）
│   ├── car_gateway.py           #   双串口通信调度与按键事件分发
│   ├── coordinate_protocol.py   #   世界坐标系统一映射算法
│   └── dcp_codec.py             #   DCP v1 编解码器（轻量级二进制流解析）
│
└── stm32-tracking/              # 小车底层运动控制（STM32F103C8T6 Keil MDK）
    ├── Project.uvprojx          #   Keil uVision5 工程文件
    ├── User/                    #   核心应用代码 (main.c, app_config.h, motor.c, line_sensor.c)
    ├── Library/                 #   STM32 标准外设固件库
    └── Start/                   #   STM32 启动文件
```

---

## 硬件接线与关键引脚

### 1. STM32 底层硬件
* **8 路数字量灰度模块**：I²C2 接口（`PB10 = SCL`，`PB11 = SDA`，设备地址 `0x2E`，100Hz 采样）
* **直流电机驱动板**：I²C1 接口（`PB6 = SCL`，`PB7 = SDA`，设备地址 `0x26`，双轮编码器速度闭环）
* **启动/停止按键**：`PB1`（内部下拉，按下为 3.3V 高电平，单击=任务A，双击/多次=任务B）
* **状态指示灯**：`PA8`（外接 3.3V LED，带限流电阻）与 `PC13`（板载 LED）

### 2. 车载树莓派网关
* **无人机通信链路**：`/dev/ttyUAV`（固定映射，115200 8N1，与无人机机载端双向 DCP v1 交互）
* **地面站通信链路**：`/dev/ttyGROUND`（固定映射，9600 8N1，DL-20 蓝牙透传，2Hz 降频转发）
* **启动按键**：BCM24（下拉输入）
* **反馈指示**：BCM23

---

## 分段速度控制策略 (A/B/C/D)

基于编码器行程自动在 A/B/C/D 四段赛道切换基准速度：
* **任务 A（伴飞抛投）**：A→B (50 mm/s) → B→C (150 mm/s) → C→D (250 mm/s) → D→终点 (200 mm/s)
* **任务 B（移动动态降落）**：A→B (250 mm/s) → B→C (200 mm/s) → C→D (50 mm/s) → D→终点 (180 mm/s)
