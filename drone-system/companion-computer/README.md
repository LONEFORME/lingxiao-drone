# 无人机机载计算上位机系统 (Companion Computer)

本目录为 2026 年电赛空地协同任务（省赛 D 题）无人机机载端的全套核心软件系统，已实飞验证并适配 **RDK X5** 与 **树莓派 4B**。

---

## 目录架构

```text
companion-computer/
├── competition_2026_d/          # 【核心业务】2026 电赛 D 题主任务决策与状态机
│   ├── auto_start.py            #   任务启动统一主入口（安全门禁、任务分发、心跳守护）
│   ├── task1_flight.py          #   任务一（伴飞与抛投）实时飞行控制循环
│   ├── task1_mission.py         #   任务一航线与阶段状态机（起飞、截获、伴飞、抛投、返航）
│   ├── task2_flight.py          #   任务二（移动平台动态降落与复飞）飞行控制
│   ├── task2_mission.py         #   任务二阶梯下降、触地确认与二次起飞状态机
│   ├── config.json              #   完赛最新标定参数（高度、速度、增益、超时阈值）
│   ├── control/                 #   路径与航线控制器 (task1_path_controller)
│   ├── vision/                  #   视觉观测融合与滤波 (platform_tracker / cybercam_reader)
│   └── test_*.py                #   200+ 项单元测试套件
│
├── shared/                      # 【通信协议】DCP v1 统一空地二进制流协议库
│   ├── competition_2026_d_protocol.py   #   帧格式打包、解包、CRC16 校验与流解析器
│   └── test_competition_2026_d_protocol.py # 协议自测试套件 (100% 通过)
│
├── basic/                       # 【底层驱动】飞控交互、VIO 定位与状态指示
│   ├── t265.py                  #   Intel RealSense T265 双目 VIO 里程计驱动
│   ├── Mission_GPT.py           #   导航底层坐标转换与速度前馈基座
│   ├── Lcode/                   #   底层工具库
│   │   ├── Lprotocol.py         #     凌霄飞控私有二进制串口通信驱动
│   │   ├── Logger.py            #     多级别飞行日志记录器
│   │   ├── gpio_led.py          #     三色状态指示灯 (set_rgb_led)
│   │   ├── gpio_button.py       #     物理按键去抖与模式切换
│   │   ├── heading_hold.py      #     航向锁定闭环控制器
│   │   └── resource_monitor.py  #     板载 CPU/内存状态监视
│   └── router.txt               #   航线航点定义文件
│
├── deploy/                      # 【运维部署】板载自启动服务与端口映射
│   ├── systemd/                 #   开机自启服务 (competition-2026-d-autostart.service)
│   ├── udev/                    #   Linux 设备固定规则 (99-drone-serial.rules, 99-realsense-libusb.rules)
│   └── tools/                   #   运维同步与日志分析工具 (sync_to_board.sh, flight_log_analyzer.py)
│
└── conftest.py                  # 测试环境命名空间自动注入配置
```

---

## 快速安装与本地验证

### 1. 环境依赖
推荐使用 Python 3.10+ 环境：
```bash
pip install pytest pyserial simple-pid numpy psutil opencv-python
```

### 2. 运行单元测试
在 `companion-computer` 目录下运行：
```bash
pytest shared/
pytest competition_2026_d/
```

---

## 板载开机自启部署 (RDK X5 / Ubuntu)

1. **配置 Udev 规则（固定串口别名）**：
   ```bash
   sudo cp deploy/udev/99-drone-serial.rules /etc/udev/rules.d/
   sudo cp deploy/udev/99-realsense-libusb.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   * 蓝牙透传串口固定映射为 `/dev/bt_serial`
   * 激光雷达串口固定映射为 `/dev/radar`

2. **配置开机自启动服务**：
   ```bash
   sudo cp deploy/systemd/competition-2026-d-autostart.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable competition-2026-d-autostart.service
   sudo systemctl start competition-2026-d-autostart.service
   ```
