# 2026 省赛 D 题地面站

这是当前联调使用的只读遥测地面站。程序只监听 DL-20 串口，不向小车或无人机发送控制指令。

## 正式运行所需文件

- `main.py`：地面站界面与任务显示逻辑。
- `dcp_protocol.py`：DCP v1 协议编解码与流解析。
- `场地图 - 打印用.png`：正式显示地图。
- `场地图.png`：场地图数据参考图。
- `car.jpg`、`plane.png`：小车和无人机图标。
- `小车10Hz模拟路径点_模式A_B.xlsx`：小车轨迹原始路径点。
- `requirements.txt`：Python 依赖。

地图和其他资源均按 `main.py` 所在目录读取，因此整个文件夹移动或解压到其他位置后仍能使用。

## Windows 安装与启动

建议使用 Python 3.10 或 3.11：

```powershell
cd "解压后的地面站目录"
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe main.py
```

也可以双击 `地面站.bat`。该脚本优先使用当前目录的 `.venv`，不再写死原工程路径，因此启动的一定是同目录内的最新版 `main.py`。

## N100（Ubuntu）安装与启动

```bash
cd ~/Desktop/26省赛
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
chmod +x start_ground_station.sh
./start_ground_station.sh
```

如果 PySide6 报 `xcb` 插件缺少系统库，可执行：

```bash
sudo apt update
sudo apt install -y libxcb-cursor0 libxkbcommon-x11-0 libxcb-xinerama0 libegl1
```

`start_ground_station.sh` 和两个 `.desktop` 文件按 N100 的固定目录 `/home/n100/Desktop/26省赛` 编写。开机自启动文件应放到 `~/.config/autostart/ground-station.desktop`。

## 当前串口配置

- 波特率：`9600 8N1`。
- Windows 默认候选：`COM20`；Linux 默认候选：`ttyUSB0`。
- 界面会优先列出 CH340/CH341/USB-Serial 设备，忽略明显无关的串口。
- 启动 2 秒后若默认候选端口存在，会自动以只读方式连接；也可以在界面中手动选择。

COM 号由 Windows 实际枚举决定，不固定为 COM20。若 DL-20 显示为其他端口，请直接选择实际端口。

## 当前任务显示逻辑

- 任务一：收到“伴飞”后，无人机图标吸附到小车；收到“抛投”后解除吸附并继续使用无人机实测坐标。
- 任务二：无人机到 C 点附近后等待小车，小车到达图标下方后共同移动；收到“从小车起飞”后恢复实测坐标，并保留 C-D 右侧 10 cm 边界及起飞锚点左下方返航区域。
- 小车轨迹按实测分段时间的绝对时间轴播放，界面短暂卡顿后会自动追帧，不逐段累积延迟。
- 接收到关键无人机事件时，状态日志显示简洁中文，并通过 Windows 当前默认扬声器播报；建议将默认输出设为“扬声器 (Realtek(R) Audio)”。
- Windows 使用普通可缩放窗口；N100 使用全屏界面。

## 正式包说明

正式运行包只包含地面站运行和部署所需文件，不包含测试程序、模拟器、示例日志、串口扫描工具或远程维护脚本。

程序运行时产生的任务日志会自动保存在程序目录下的 `logs/`。
