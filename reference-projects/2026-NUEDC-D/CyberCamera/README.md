# CyberCAM 项目目录

## 结构说明

```
CyberCamera/
├── boards/
│   └── cybercam/          # CyberCAM（核桃派）端代码——运行在 CyberCAM 板上
│       ├── main.py        # 入口：开摄像头→检测→UART 发送
│       ├── detector.py    # 黑色方块检测（OpenCV, 1920×1080）
│       ├── protocol.py    # UART ASCII 协议定义
│       └── calib.py       # 焦距标定工具
├── k230/                   # 原 K230（CanMV）端代码（Q3 赛题）
│   ├── animal_detect/      # 动物检测
│   └── servo/              # QR 码视觉伺服
└── README.md               # 本文件
```

## 通信协议（CyberCAM → Pi）

ASCII 格式，每行一条检测结果，波特率 115200：

```
<dx>,<dy>,<found>\n
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `dx`  | 有符号 int | 方块中心相对图像中心的 X 偏移（像素），右为正 |
| `dy`  | 有符号 int | 方块中心相对图像中心的 Y 偏移（像素），下为正 |
| `found` | 0/1 | 是否检测到目标 |

示例：
```
-30,15,1\n    # 目标偏左 30px、偏下 15px，已找到
0,0,0\n       # 未找到目标
```

## Pi 端代码

Pi 端的协议解析和视觉伺服控制在 `drone_control/competition_2026/vision/` 下：
- `cyber_cam_reader.py` — UART 读取 + 协议解析 → `Detection` 对象
- `servo_controller.py` — 接收 `Detection`，计算速度修正

## 2026 D题 Cyber Camera 调试图片记录

`boards/cybercam_d/main.py` 使用外接 CSI 摄像头持续检测并发送 VS1。调试图片记录复用已经打开的相机数据流，不会每秒重新打开 CSI。

开启每秒一张标注图片：

```bash
cd /home/pi/FJJ/cybercam_d
./run_flight.sh \
  --camera 0 --width 640 --height 480 \
  --target blue_square --serial /dev/ttyS2 \
  --debug-record on \
  --debug-record-dir /home/pi/FJJ/cybercam_d/debug_frames \
  --debug-record-interval 1
```

关闭记录：

```bash
--debug-record off
```

记录默认关闭。开启后，每次程序启动都会在 `--debug-record-dir` 下创建一个时间戳子目录，图片包含图像中心、目标轮廓、目标中心、质量、像素误差和处理帧率。JPEG保存失败只写入标准错误，不会中断 VS1 串口发送。
