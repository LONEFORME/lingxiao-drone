# 01 飞控源码 · STM32F407 凌霄飞控固件

无人机底层飞控固件（Keil MDK-ARM 工程），跑在 **STM32F407** 上。

- `FcSrc/` — 输入输出主程序：采遥控与传感器 → 把速度目标打成 **0x41 实时控制帧**发给匿名 IMU 核 → 收 `pwm_to_esc` 输出 PWM
- `ProjectSTM32F407/` / `ProjectMSP432/` / `ProjectTM4C123/` — 各芯片 Keil 工程入口
- `Mycode/` `DriversBsp/` `DriversMcu/` `Doc/` — 自研层、板级/芯片驱动与文档

⚠️ **姿态 / 位置闭环在匿名 IMU 核固件里（不公开），本目录没有控制律**——别在这找 PID。
改通信前先读 [`../08_协议与规范/communication_protocol.md`](../08_协议与规范/communication_protocol.md)。
