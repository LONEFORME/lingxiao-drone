# 地平线 RDK X5 核心依赖库与环境版本清单 (Dependency & Library Versions)

本文档系统梳理了地平线 RDK X5（Ubuntu 22.04.5 aarch64）板载的核心运行时、Python 库、BPU 专有加速库、ROS 2 及系统外设环境版本，用于在新板卡部署或环境复现时精准对齐。

> 📄 **完整 pip 冻结清单**：见同级目录下的 [`requirements_rdk_x5.txt`](requirements_rdk_x5.txt)（包含全部 276 个已安装 Python 包精确版本）。

---

## 1. 基础系统与编译器环境

| 组件 | 版本 | 说明 |
| :--- | :--- | :--- |
| **操作系统** | Ubuntu 22.04.5 LTS (Jammy Jellyfish) | 官方 aarch64 镜像 |
| **Linux 内核** | `6.1.83 #1 SMP PREEMPT` | 针对 D-Robotics RDK X5 V1.0 定制优化内核 |
| **系统架构** | `aarch64 (ARMv8 Cortex-A55 8-Core)` | 运行于 64-bit 模式 |
| **Python 版本** | **Python 3.10.12** (`/usr/bin/python3`) | 系统级默认 Python 解释器 |
| **GCC / G++** | 11.4.0 (`gcc (Ubuntu 11.4.0-1ubuntu1~22.04) 11.4.0`) | C/C++ 默认编译链 |
| **GLIBC** | 2.35 | Ubuntu 22.04 标准 C 库 |

---

## 2. 地平线专有硬件与 BPU (NPU) 算力库

| 库 / 模块名称 | 版本 | 核心用途与接口 |
| :--- | :--- | :--- |
| **`hobot_dnn`** | **3.0.9** | 地平线 BPU 深度学习模型推理加速核心（支持加载 `.bin` 模型） |
| **`hbm_runtime`** | **3.0.9** | 地平线板载统一内存（Heterogeneous Memory）高效显存分配运行库 |
| **`hobot_vio`** | **3.0.9** | 硬件级图像与视频处理加速引擎（ISP、Scaler、VPS 零拷贝管道） |
| **`Hobot.GPIO`** | **0.0.2** | 官方 40-Pin GPIO 控制库，兼容 Jetson/RPi.GPIO 语法规范 |

---

## 3. 视觉、算法与科学计算依赖

| 库名称 | 精确版本 | 典型调用场景 |
| :--- | :--- | :--- |
| **`opencv-python`** | **4.11.0.86** | 靶标轮廓识别、圆形检测、巡线、降落对准视觉算法 |
| **`numpy`** | **1.26.4** | 点云矩阵运算、坐标旋转变换、数学数组处理 |
| **`scipy`** | **1.10.1** | 空间位姿优化、滤波拟合 |
| **`matplotlib`** | **3.10.9** | 点云离线绘图、Jupyter 可视化 |
| **`pillow`** | **11.3.0** | 图像基本 IO 与处理 |
| **`pyzbar`** | **0.1.9** | 仓库盘点任务中的一维条码与二维码检测解码 |
| **`simple-pid`** | **2.0.1** | 无人机速度与航向角闭环 PID 控制器 |
| **`pytest`** | **6.2.5** | 避障与坐标系算法自动化单元测试套件 |

---

## 4. 机器人与空间感知 (ROS 2 & Sensors)

| 模块 / 传感器 | 版本 / 协议 | 配置详情与说明 |
| :--- | :--- | :--- |
| **ROS 2 发行版** | **ROS 2 Humble Hawksbill** | 基础路径 `/opt/ros/humble`，系统预装 280+ 核心功能包 |
| **colcon 构建套件** | `colcon-core 0.20.1`, `colcon-common-extensions 0.3.0` | ROS 2 工作空间多包并发编译工具 |
| **rosdep / 依赖** | `rosdep 0.26.0`, `rosdepc 1.1.0` | 国内环境快速依赖解析 |
| **DDS 中间件** | **eProsima Fast DDS (Fast RTPS)** | 支持 Discovery Server（连接 A7Z 服务端:11811）或独立模式 |
| **串口通信库** | **`pyserial 3.5`** | 飞控与激光雷达双路 460800 高频通信的核心依赖 |
| **Intel RealSense** | **`librealsense 2.50.0`** | 针对 T265 (Myriad VPU) 编译支持，提供 6-DoF VIO 实时位姿 |

---

## 5. 硬件总线与底层外设库

| 库名称 | 版本 | 对应物理外设 |
| :--- | :--- | :--- |
| **`smbus2`** | 0.6.1 | I2C 总线控制（`/dev/i2c-0`），驱动 0.96寸 OLED 屏幕 |
| **`i2cdev`** | 1.2.4 | 底层 I2C 硬件通信接口 |
| **`spidev`** | 3.7 | SPI 高速通信接口 |
| **`Adafruit_SSD1306`** | 1.6.2 | OLED 屏幕驱动库 |
| **`Adafruit_GPIO`** | 1.0.3 | 传感器通用 GPIO 抽象层 |
| **`wiringpi`** | 2.60.1 | C 语言风格引脚速控库 |

---

## 6. 交互式开发与 Web 流媒体服务

| 服务 / 库 | 版本 | 访问方式与端口 |
| :--- | :--- | :--- |
| **`jupyterlab`** | **4.5.7** | 系统服务后台守护，直访 `http://192.168.137.131:8888` (密码: 1) |
| **`ipykernel`** | 6.29.5 | Jupyter Python 3 内核 |
| **`Flask`** | 3.1.3 | Web 调试界面与简单控制端 |
| **`tornado`** | 6.5.5 | 高并发异步 Web 服务器 |
| **`websockets`** | 15.0.1 | 实时遥测数据长连接推送 |

---

## 7. 快速在新环境一键安装关键依赖

如果您需要在另一台 Ubuntu 22.04 或 RDK X5 上复现相同环境，可直接执行：

```bash
# 1. 基础编译与串口依赖
sudo apt update && sudo apt install -y python3-pip python3-dev build-essential libzbar0

# 2. 从导出的 requirements_rdk_x5.txt 安装对应版本 (国内源极速安装)
pip3 install -r requirements_rdk_x5.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

# 3. 若只需核心最小实战依赖
pip3 install pyserial==3.5 numpy==1.26.4 opencv-python==4.11.0.86 simple-pid==2.0.1 pyzbar==0.1.9 pytest==6.2.5
```
