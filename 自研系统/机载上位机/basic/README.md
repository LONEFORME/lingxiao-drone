# basic — 基本飞行控制器

基于 Intel T265 视觉里程计的最小可飞版本。无 GPS、无视觉识别、无地面站通信，
是所有扩展版本（`basic_radar`、`competition_2026`、`warehouse_inventory` 等）的共同基础。

---

## 硬件要求

| 硬件 | 说明 |
|------|------|
| 地瓜派 RDK X5 | 板载计算单元，运行本程序 |
| Intel RealSense T265 | 视觉里程计，提供局部坐标系位姿 |
| 飞控（自制） | 通过串口 `/dev/ttyS6` 收发 AA 帧，460800 baud |
| 激光测高 | 由飞控下发，覆盖 Z 轴定高 |
| 一键起飞按钮 | BCM17，下降沿触发 |
| RGB 警示灯 | R=BCM23 / G=BCM25 / B=BCM24 |

---

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 真机起飞（在 RDK X5 上执行）
python main.py

# 桌面测试（不解锁飞控，电机不转）
DRONE_DRY_RUN=1 python main.py
```

### 起飞流程

1. 绿灯常亮 — 完成 T265 拔插后按下一键起飞按钮
2. T265 初始化，打开飞控串口
3. 红灯警示 **5 秒**，人员撤离
4. 状态机启动，按 `router.txt` 自主飞行
5. 末航点到达后自动降落，程序退出

---

## 航路文件

默认读取同目录 `router.txt`，每行一个航点，格式：

```
x,y,z
```

坐标单位：**米**，以起飞点为原点，T265 局部坐标系。

**默认示例（3 个航点）：**

```
0.0,0.0,1.0    # 悬停起飞点，高度 1m
-0.6,0.0,1.0   # 向前飞 0.6m
-0.6,0.0,0.2   # 原地下降至 0.2m，准备降落
```

`router_tests/` 和 `router_landing_test/` 目录下有若干预设测试路径可直接使用。

---

## 架构概览

```
main.py
 └─ 等待按钮 → 初始化 T265 + 串口 → Mission → 状态机主循环
        │
        ▼
Mission_GPT.py（30ms 控制周期）
 ┌──────────────────────────────────────────┐
 │  IDLE → TAKEOFF → NAVIGATE → LAND → END  │
 └──────────────────────────────────────────┘
        │              │
        ▼              ▼
   HeadingHold    XY/Z PID（Lpid.py）
   航向保持外环    位置控制外环
        │              │
        └──────┬────────┘
               ▼
         Lprotocol.py（串口双向通信）
         ├─ 监听线程    飞控→上位机  50Hz（姿态/激光高度）
         ├─ T265 发送  上位机→飞控  100Hz（速度）
         └─ 指令发送   上位机→飞控   50Hz（速度指令）
```

---

## 模块说明

### 任务层

| 文件 | 职责 |
|------|------|
| `main.py` | 程序入口，按键门禁 → 硬件初始化 → 任务启动 |
| `Mission_GPT.py` | 状态机主体，包含 PID 控制、到达判定、安全保护、飞行日志 |
| `t265.py` | T265 位姿/速度读取，含坐标系转换和模拟 fallback |

### Lcode 库

| 文件 | 职责 |
|------|------|
| `Lprotocol.py` | 飞控串口协议，三线程收发（监听 / T265 / 指令） |
| `Lpid.py` | PID 控制器封装（XY 位置环 + Yaw 角速度环） |
| `heading_hold.py` | 航向保持外环，起飞时锁定初始 Yaw |
| `navigation_profile.py` | 航点到达策略（`precision` / `cruise` 两种模式） |
| `global_variable.py` | 跨线程全局状态（串口偏置、锁、帧时间戳） |
| `gpio_button.py` | 一键起飞按键驱动，非板载环境自动空操作 |
| `gpio_led.py` | RGB 警示灯驱动，状态型无阻塞接口 |
| `Logger.py` | 日志模块，同时输出控制台（INFO）和 `fc_log.log`（DEBUG） |
| `resource_monitor.py` | 后台采样 CPU / 内存 / 温度，写入 `flight_data.jsonl` |

---

## 状态机详解

```
IDLE
 │  等待 start() 调用
 ▼
TAKEOFF
 │  盲飞离地至 TAKEOFF_LIFTOFF_CM（35cm）
 │  等待 T265 置信度 ≥ 2，超时 8s
 │  激活航向保持，开始 PID 爬升至目标高度
 ▼
NAVIGATE
 │  逐航点飞行
 │  每个航点：PID 控制 → 滑动窗口到达确认 → 悬停 1.5s → 下一个
 ▼
LAND
 │  缓降至地面，等待飞控上锁（连续 5 帧 unlock_sta==0）
 │  超时兜底 25s
 ▼
END
   清理资源，写飞行日志
```

**安全保护：**
- 飞控帧超时 **2s** → 急停
- T265 位姿丢失 → 急停
- 激光高度异常值过滤（上限 10m，防传感器错误码污染）
- 航向保持故障检测 + 跑飞检测

---

## 航点到达策略

通过环境变量 `DRONE_NAV_PROFILE` 切换，默认 `precision`。

### precision（精确模式）

要求滑动窗口内 **60%** 帧同时满足：
- XY 距离 ≤ 0.15m
- Z 距离 ≤ 0.20m
- T265 速度均值 ≤ 0.05 m/s

窗口大小：15 帧（连续约 0.45s）。

### cruise（巡航模式）

进入 0.15m 半径内 + 连续 3 帧确认即视为到达；
首尾航点自动降级为 `precision` 模式保证精度。
含进度超时检测（默认 25s + 按距离动态扩展）。

---

## 环境变量参考

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DRONE_DRY_RUN` | `0` | `1` = 桌面测试，不解锁飞控 |
| `DRONE_FC_PORT` | `/dev/ttyS6` | 飞控串口路径 |
| `DRONE_NAV_PROFILE` | `precision` | 航点到达策略（`precision` / `cruise`） |
| `DRONE_HEADING_HOLD` | `1` | `0` = 关闭航向保持 |
| `DRONE_HEADING_HOLD_KP` | — | 航向保持 P 增益 |
| `DRONE_CRUISE_RADIUS_M` | `0.15` | cruise 模式到达半径（m） |
| `DRONE_CRUISE_TIMEOUT_S` | `25.0` | cruise 模式超时基准（s） |
| `DRONE_YAW_TEST_BURST` | `0` | `1` = 旧 Yaw 诊断脉冲（与航向保持互斥） |

---

## 运行测试

```bash
# 全量测试（支持 Windows / 非板载环境）
pytest

# 带详情输出
pytest -v

# 单个模块
pytest test_heading_hold.py
```

共 **18 个测试文件**，覆盖到达判定、航向保持、GPIO 驱动、串口协议、导航模式等核心逻辑。所有测试无需硬件，`DRY_RUN` 模式下可在任意平台执行。

---

## 飞行日志

飞行过程中自动追加写入 `flight_data.jsonl`（不清空），每行一条 JSON 记录，包含：
- 时间戳、状态机状态
- T265 位姿与速度
- 飞控帧数据（姿态、解锁状态、激光高度）
- CPU / 内存 / 温度（来自 `resource_monitor`）

> ⚠️ **飞行前请先备份并移走旧的 `flight_data.jsonl`**，否则新数据追加进旧文件难以区分。

---

## 依赖

**运行时（`requirements.txt`）：**

```
numpy>=1.21.0
simple_pid>=0.7.0
pyserial>=3.5
psutil>=5.9.0
pyrealsense2>=2.50.0   # 真机必须；桌面 DRY_RUN 模式可缺省，t265.py 自动降级为模拟
```

**开发/测试（`requirements-dev.txt`）：**

```
pytest>=7.0.0
```

板子上只需安装 `requirements.txt`；本地开发运行测试用 `requirements-dev.txt`：

```bash
pip install -r requirements-dev.txt
```
