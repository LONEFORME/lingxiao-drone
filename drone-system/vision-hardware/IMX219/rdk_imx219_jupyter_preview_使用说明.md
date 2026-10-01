# rdk_imx219_jupyter_preview 使用说明

> 适用环境：RDK X5、IMX219 FOV79、`V4L2 sif-isp-vse` 模式  
> Python 模块：`rdk_imx219_jupyter_preview.py`  
> V4L2 生产程序：`rdk_imx219_stream`  
> 推荐设备：`/dev/video10`，`960x540 NV12`

## 1. 模块用途

`rdk_imx219_jupyter_preview.py` 是面向 JupyterLab 的 IMX219 摄像头采集和简单视觉识别模块，提供：

- 后台线程持续采集最新 NV12 图像。
- JupyterLab 实时预览。
- 曝光、模拟增益和 VBlank 设置。
- 软件白平衡和饱和度修正。
- 红、黄、绿、蓝区域识别。
- 红色正方形识别和中心坐标输出。
- 单帧 NV12、PNG 和 JSON 参数保存。
- 函数式接口和 `VisionSystem` 类式接口。
- 普通 Python 脚本和无界面开机服务调用，不强制依赖 Jupyter。

推荐使用 `VisionSystem` 类。它与普通 `cv2.VideoCapture` 类似，但底层使用已经验证的 V4L2 MMAP 生产程序，适配 RDK X5 的 IMX219 NV12 节点。

## 2. 运行结构

```text
IMX219
→ CSI0
→ SIF0
→ ISP0
→ VSE0 /dev/video10
→ rdk_imx219_stream（C/MMAP）
→ Python 后台采集线程
→ OpenCV 颜色/形状识别
→ ipywidgets.Image 实时显示
```

不要直接把 `/dev/video10` 替换到普通 USB 摄像头代码的：

```python
cv2.VideoCapture("/dev/video10")
```

当前 RDK 驱动在通用 OpenCV/V4L2 取流路径中可能超时，因此模块使用 `rdk_imx219_stream` 持续输出完整 NV12 帧。

## 3. 运行前提

### 3.1 摄像头模式

开发板应处于：

```text
sudo srpi-config
→ Interface Options
→ I7 V4L2
→ V4L2 sif-isp-vse
→ CAM0 = IMX219
→ CAM1 = None / Disable
→ 重启
```

HBN 模式不能使用本模块的 `/dev/video10` 采集路径。

### 3.2 核对设备节点

```bash
v4l2-ctl --list-devices
sudo media-ctl -d /dev/media1 -p
```

USB 摄像头会改变节点编号。本文按当前已验证拓扑说明：

```text
/dev/v4l-subdev1   IMX219 控制节点
/dev/video6        ISP 1920x1080 NV12
/dev/video10       VSE 缩放 NV12，实时预览推荐
```

如果实际编号不同，应同时修改模块参数和 `SENSOR_SUBDEVICE`。

### 3.3 Python 依赖

```bash
python3 - <<'PY'
import cv2
import numpy
import ipywidgets

print("OpenCV:", cv2.__version__)
print("NumPy:", numpy.__version__)
print("ipywidgets:", ipywidgets.__version__)
PY
```

还需要：

```bash
command -v v4l2-ctl
```

## 4. 部署文件

建议开发板目录：

```text
/app/cdev_demo/v4l2/rdk_imx219_stream.c
/app/cdev_demo/v4l2/rdk_imx219_stream
/app/cdev_demo/v4l2/rdk_imx219_jupyter_preview.py
```

编译 C 生产程序：

```bash
cd /app/cdev_demo/v4l2

sudo gcc \
  -O2 \
  -Wall \
  -Wextra \
  -std=c11 \
  -o rdk_imx219_stream \
  rdk_imx219_stream.c

sudo chmod +x rdk_imx219_stream
```

确认：

```bash
ls -lh /app/cdev_demo/v4l2/rdk_imx219_stream
file /app/cdev_demo/v4l2/rdk_imx219_stream
```

成功编译时 `gcc` 没有输出是正常现象。

## 5. 作为 Python 模块导入

### 5.1 从 `/app/cdev_demo/v4l2` 导入

Notebook 第一个 Cell：

```python
%load_ext autoreload
%autoreload 2

import sys

MODULE_DIRECTORY = "/app/cdev_demo/v4l2"
if MODULE_DIRECTORY not in sys.path:
    sys.path.insert(0, MODULE_DIRECTORY)

import rdk_imx219_jupyter_preview as rdk
```

以后不需要执行：

```python
%run /app/cdev_demo/v4l2/rdk_imx219_jupyter_preview.py
```

### 5.2 从 `/home/sunrise` 导入

如果为了在 JupyterLab 左侧文件栏中方便编辑，把模块放在 `/home/sunrise`：

```python
import sys

if "/home/sunrise" not in sys.path:
    sys.path.insert(0, "/home/sunrise")

import rdk_imx219_jupyter_preview as rdk
```

模块中的生产程序路径仍然是：

```text
/app/cdev_demo/v4l2/rdk_imx219_stream
```

### 5.3 修改模块后重新加载

先释放旧摄像头对象：

```python
vision.release()
```

再重新加载：

```python
import importlib

importlib.reload(rdk)
```

旧对象仍属于旧类定义，重新加载后必须重新创建 `VisionSystem`。

### 5.4 普通 Python 程序导入

模块和调用程序位于同一目录时，普通脚本直接使用：

```python
import rdk_imx219_jupyter_preview as vision_system

vision = vision_system.VisionSystem(0)
```

这里的 `0` 表示 CAM0，默认映射到当前已验证的 `/dev/video10`。也可以明确指定：

```python
vision = vision_system.VisionSystem(
    0,
    device="/dev/video10",
)
```

普通脚本中的采集和识别不需要 `ipywidgets` 或 IPython。只有 `run_preview()` 和 `show_preview()` 需要 Jupyter 组件。

## 6. 最小使用示例

### 6.1 创建视觉系统

```python
vision = rdk.VisionSystem(
    device="/dev/video10",
    raw_width=960,
    raw_height=540,
    display_width=960,
    display_height=540,
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
    blue_gain=0.95,
    green_gain=1.06,
    red_gain=1.00,
    saturation=1.08,
)
```

`VisionSystem` 是 `IMX219VisionSystem` 的别名：

```python
rdk.VisionSystem is rdk.IMX219VisionSystem
```

默认 `auto_open=True`，创建对象时会：

1. 设置 VBlank、曝光和模拟增益。
2. 启动 `rdk_imx219_stream`。
3. 启动后台采集线程。
4. 等待第一帧图像。

### 6.2 检查状态

```python
print("摄像头状态：", vision.is_camera_open())
print("采集 FPS：", vision.capture_fps)
print("图像中心：", vision.get_image_center())
```

典型图像中心：

```text
(480, 270)
```

## 7. 实时预览

### 7.1 只显示画面

```python
vision.show_preview()
```

按 `Ctrl+C` 停止。默认 `close_on_stop=True`，停止后自动释放摄像头。

### 7.2 停止显示但保留后台采集

```python
vision.show_preview(
    close_on_stop=False,
)
```

按 `Ctrl+C` 后，摄像头仍然打开。完成其他操作后必须手动释放：

```python
vision.release()
```

### 7.3 显示刷新参数

```python
vision.show_preview(
    display_interval=0.08,
    jpeg_quality=65,
)
```

| 参数 | 含义 |
| --- | --- |
| `display_interval=0.08` | 最快约 12.5 次/秒更新 Notebook |
| `jpeg_quality=65` | Jupyter 显示 JPEG 质量 |

状态栏分别显示采集 FPS 和显示 FPS。显示 FPS 低于采集 FPS，不代表传感器丢帧。

## 8. 识别红色正方形

### 8.1 实时识别

```python
vision.show_preview(
    detect_red_squares=True,
    minimum_square_area=800,
)
```

识别成功后画面显示：

```text
RED SQUARE (中心X,中心Y)
```

返回信息包含：

```python
{
    "label": "RED_SQUARE",
    "area": 3520,
    "center": (520, 310),
    "box": (480, 270, 80, 80),
    "side_ratio": 1.04,
}
```

检测条件：

- HSV 色调位于红色的两个区间。
- 轮廓面积大于最小面积。
- 轮廓拟合后有四个顶点。
- 轮廓为凸四边形。
- 四个角近似直角。
- 长边与短边比例接近 1。
- 轮廓对旋转矩形的填充率足够高。

### 8.2 只获取一次检测结果

```python
result_frame, detections = vision.detect_red_squares(
    minimum_area=800,
)

print(detections)
```

`result_frame` 是已经画出轮廓和中心点的 BGR 图像。

### 8.3 最小面积调整

```text
300-500     小目标，容易受噪点影响
800         当前 960x540 推荐值
1500-3000   只检测较大目标
```

面积单位是当前显示图像中的像素面积。改变分辨率后需要重新调整。

## 9. 识别红、黄、绿、蓝区域

### 9.1 实时识别

```python
vision.show_preview(
    detect_colors=True,
    minimum_color_area=800,
)
```

支持标签：

```text
RED
YELLOW
GREEN
BLUE
```

### 9.2 获取单帧结果

```python
result_frame, detections = vision.detect_colors(
    minimum_area=800,
)

print(detections)
```

颜色检测只判断颜色区域，不判断正方形、圆形或三角形。

不要同时开启：

```python
detect_colors=True
detect_red_squares=True
```

否则红色正方形会同时显示普通红色框和正方形轮廓。

## 10. 读取最新帧

后台线程始终覆盖保存最新帧：

```python
frame = vision.get_frame()

if frame is not None:
    print(frame.shape)
```

`960x540` 时：

```text
(540, 960, 3)
```

返回值是 BGR `numpy.ndarray`，已经应用构造对象时设置的白平衡、饱和度和可选降噪参数。

显示一帧：

```python
import cv2
import ipywidgets as widgets
from IPython.display import display

frame = vision.get_frame()
encoded, jpeg = cv2.imencode(".jpg", frame)

display(
    widgets.Image(
        value=jpeg.tobytes(),
        format="jpeg",
    )
)
```

## 11. 通用形状判断

`detect_shape()` 接收已经找到的 OpenCV 轮廓：

```python
shape = vision.detect_shape(contour)
print(shape)
```

可能返回：

```text
triangle
square
rectangle
pentagon
circle
polygon
None
```

该方法只判断轮廓形状，不负责生成二值图或查找轮廓。红色正方形检测应优先使用 `detect_red_squares()`。

## 12. 动态修改摄像头参数

```python
vision.set_controls(
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
)
```

可以只传需要修改的参数：

```python
vision.set_controls(exposure=2116)
```

当前推荐值：

| 目标 | Exposure | VBlank | Gain |
| --- | ---: | ---: | ---: |
| 低延迟/较少拖影 | `1587` | `683` | `0` |
| 综合推荐 | `2645` | `2446` | `0` |
| 静态低噪点 | `3174` | `2446` | `0` |

`2645/2446` 的传感器帧率约为 15 FPS。降低显示分辨率只能减少 Python 和 Jupyter 开销，不能突破长曝光的传感器帧周期。

## 13. 关闭和重新打开

### 13.1 关闭

以下两个方法等价：

```python
vision.close()
```

```python
vision.release()
```

### 13.2 重新打开

```python
vision.reopen()
```

### 13.3 上下文管理器

```python
with rdk.VisionSystem() as vision:
    frame = vision.get_frame()
    result_frame, detections = vision.detect_red_squares(
        minimum_area=800,
    )
    print(detections)
```

离开 `with` 后自动释放摄像头。

## 14. 构造参数说明

```python
rdk.VisionSystem(
    src=0,
    device="/dev/video10",
    raw_width=960,
    raw_height=540,
    display_width=960,
    display_height=540,
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
    blue_gain=0.95,
    green_gain=1.06,
    red_gain=1.00,
    saturation=1.08,
    luma_denoise=0,
    chroma_denoise=0,
    temporal_denoise=0.0,
    blur_kernel_size=7,
    threshold_value=60,
    min_contour_area=800,
    max_contour_area=200000,
    brightness_threshold=200,
    min_bright_area=100,
    max_bright_area=5000,
    enable_bright_detection=True,
    capture_thread_daemon=False,
    auto_open=True,
)
```

| 参数 | 说明 |
| --- | --- |
| `src` | 兼容旧视觉代码的摄像头编号；`0` 默认映射 CAM0 `/dev/video10` |
| `device` | V4L2 NV12 输出节点 |
| `raw_width/raw_height` | V4L2/VSE 实际输出尺寸 |
| `display_width/display_height` | Python 中处理和显示的尺寸 |
| `exposure` | IMX219 曝光行数 |
| `analogue_gain` | 模拟增益，建议保持 0 |
| `vertical_blanking` | 决定最大曝光和帧周期 |
| `blue/green/red_gain` | BGR 软件通道增益 |
| `saturation` | HSV 饱和度倍率 |
| `luma_denoise` | 亮度高斯核，0 表示关闭 |
| `chroma_denoise` | 色度高斯核，0 表示关闭 |
| `temporal_denoise` | 时间混合比例，0 表示关闭 |
| `blur_kernel_size` | 主轮廓预处理高斯核，自动修正为奇数 |
| `threshold_value` | 主轮廓二值化阈值 |
| `min/max_contour_area` | 通用轮廓面积范围 |
| `brightness_threshold` | 火源/亮点检测亮度阈值 |
| `min/max_bright_area` | 亮点面积范围 |
| `enable_bright_detection` | 是否在采集线程中持续更新亮点坐标 |
| `capture_thread_daemon` | 飞控主程序建议 False，由 `release()` 正常结束 |
| `auto_open` | 创建对象时是否自动打开摄像头 |

建议让原始尺寸和显示尺寸相同：

```python
raw_width=960
raw_height=540
display_width=960
display_height=540
```

这样由 VSE 完成硬件缩放，Python 不再重复缩放。

## 15. 函数式兼容接口

### 15.1 `run_preview()`

不创建对象也可以运行：

```python
rdk.run_preview(
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
    device="/dev/video10",
    raw_width=960,
    raw_height=540,
    display_width=960,
    display_height=540,
    blue_gain=0.95,
    green_gain=1.06,
    red_gain=1.00,
    saturation=1.08,
    detect_red_squares=True,
    minimum_square_area=800,
)
```

函数式接口在同一个循环中采集、处理和显示。类式接口使用后台线程持续维护最新帧，更适合后续综合视觉系统开发。

### 15.2 `capture_frame()`

保存分析帧：

```python
metadata = rdk.capture_frame(
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
    device="/dev/video6",
    raw_width=1920,
    raw_height=1080,
    blue_gain=0.95,
    green_gain=1.06,
    red_gain=1.00,
    saturation=1.08,
    output_dir="/home/sunrise/imx219-captures",
)
```

输出：

```text
*.nv12              原始 NV12 帧
*-direct.png         未软件修色的 PNG
*-corrected.png      修色后的 PNG
*.json               控制值和图像统计
```

调用 `capture_frame()` 前必须释放 `VisionSystem`，否则两个程序会同时占用摄像头：

```python
vision.release()
metadata = rdk.capture_frame(...)
```

## 16. 公开函数概览

| 接口 | 用途 |
| --- | --- |
| `VisionSystem` | 推荐的类式综合接口 |
| `IMX219VisionSystem` | `VisionSystem` 的完整类名 |
| `run_preview()` | 函数式实时预览 |
| `capture_frame()` | 保存单帧分析文件 |
| `set_camera_controls()` | 直接设置传感器控制 |
| `apply_color_correction()` | BGR 增益和饱和度修正 |
| `apply_fast_denoise()` | 简单亮度/色度高斯处理 |
| `draw_color_detections()` | 在 BGR 图像上检测颜色区域 |
| `draw_red_square_detections()` | 在 BGR 图像上检测红色正方形 |

`VisionSystem` 还提供与旧 `A_visual_deal` 相近的方法：

```text
get_frame
preprocess_frame
detect_main_contour
detect_shape
detect_color
detect_color_objects
detect_color_shape
detect_color_triangle
detect_circles
get_bright_spots
get_annotated_bright_frame
check_bright_spot_in_zone
show_frame
show_bright_frame
release
```

以下划线开头的函数属于内部实现，不建议在 Notebook 中直接依赖：

```text
_corner_cosine
_query_camera_controls
_query_video_format
_summarize_bgr
_summarize_luma
```

## 17. 常见问题

### 17.1 `ModuleNotFoundError`

确认脚本所在目录已加入 `sys.path`：

```python
import sys
sys.path.insert(0, "/app/cdev_demo/v4l2")
```

确认文件存在：

```bash
ls -l /app/cdev_demo/v4l2/rdk_imx219_jupyter_preview.py
```

### 17.2 找不到生产程序

错误路径：

```text
/app/cdev_demo/v4l2/rdk_imx219_stream
```

检查并重新编译：

```bash
ls -l /app/cdev_demo/v4l2/rdk_imx219_stream
```

### 17.3 `Device or resource busy`

不要同时运行：

- `VisionSystem`
- `run_preview()`
- `capture_frame()`
- 官方 `v4l2_demo`

先释放：

```python
vision.release()
```

再检查：

```bash
sudo fuser -v /dev/video6 /dev/video10
```

不要用 `Ctrl+Z` 停止摄像头程序，应该使用 `Ctrl+C`。

### 17.4 第一帧超时

检查：

```bash
v4l2-ctl --list-devices
sudo media-ctl -d /dev/media1 -p
```

确认当前不是 HBN 模式，并确认 `/dev/video10` 对应 IMX219 VSE 输出。

### 17.5 能看到红色但不识别正方形

依次调整：

```python
minimum_square_area=500
```

确保：

- 红色区域没有被严重反光切成多个区域。
- 正方形没有被其他物体遮挡。
- 目标在画面中至少有几十像素宽。
- 饱和度和照明足以让红色与背景分离。

### 17.6 误识别很多小红点

噪点较多时提高面积：

```python
minimum_square_area=1500
```

同时保持：

```python
analogue_gain=0
```

### 17.7 显示 FPS 较低

优先保持：

```text
960x540   综合推荐
640x360   更低显示延迟
```

降低 JPEG 质量：

```python
vision.show_preview(
    jpeg_quality=55,
)
```

识别、JPEG 编码和 Jupyter 控件刷新都会消耗 CPU。采集线程使用最新帧，不需要让显示 FPS 与传感器 FPS 完全相同。

### 17.8 修改代码后没有变化

先结束旧对象：

```python
vision.release()
```

再执行：

```python
import importlib
importlib.reload(rdk)

vision = rdk.VisionSystem(...)
```

## 18. 推荐 Notebook 布局

### Cell 1：导入

```python
%load_ext autoreload
%autoreload 2

import sys

sys.path.insert(0, "/app/cdev_demo/v4l2")
import rdk_imx219_jupyter_preview as rdk
```

### Cell 2：创建对象

```python
vision = rdk.VisionSystem(
    device="/dev/video10",
    raw_width=960,
    raw_height=540,
    display_width=960,
    display_height=540,
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
    blue_gain=0.95,
    green_gain=1.06,
    red_gain=1.00,
    saturation=1.08,
)
```

### Cell 3：红色正方形实时识别

```python
vision.show_preview(
    detect_red_squares=True,
    minimum_square_area=800,
)
```

### Cell 4：释放

```python
vision.release()
```

## 19. 当前限制

- 颜色阈值依赖环境照明和白平衡，改变光源后可能需要调整 HSV 范围。
- 红色正方形识别是传统 OpenCV 规则，不具备神经网络的泛化能力。
- 遮挡、强反光、过小目标和严重透视会降低识别稳定性。
- 当前 V4L2 驱动未开放 ISP 2DNR/3DNR 控制，图像噪点会影响小轮廓。
- 本模块不包含 YOLO、人脸识别或二维码模型。
- `show_preview()` 是 Notebook 阻塞循环，需要按 `Ctrl+C` 返回 Cell。

对于固定光源、固定背景和颜色形状目标，本模块适合快速原型和控制系统坐标输入。对于复杂场景和通用目标，应继续接入 RDK X5 的 BPU 模型推理接口。

## 20. 普通脚本测试与飞控集成

测试入口：

```text
test_rdk_imx219_vision.py
```

无界面测试红色正方形，适合 SSH 和开机服务：

```bash
cd /app/cdev_demo/v4l2
python3 test_rdk_imx219_vision.py
```

带 OpenCV 窗口测试：

```bash
python3 test_rdk_imx219_vision.py --display
```

测试主轮廓：

```bash
python3 test_rdk_imx219_vision.py \
  --mode main-contour \
  --display
```

测试火源/亮点：

```bash
python3 test_rdk_imx219_vision.py \
  --mode bright
```

飞控程序中使用：

```python
import rdk_imx219_jupyter_preview as vision_system

vision = vision_system.VisionSystem(
    0,
    device="/dev/video10",
    capture_thread_daemon=False,
)

try:
    while True:
        frame = vision.get_frame()
        processed = vision.preprocess_frame(frame)
        center, contour = vision.detect_main_contour(processed)

        if contour is not None:
            shape = vision.detect_shape(contour)
            color = vision.detect_color(contour, frame)
            # 在这里把 center、shape、color 写入飞控状态变量。
finally:
    vision.release()
```

开机自启动时不要调用 `cv2.imshow()`、`show_frame()` 或 `show_bright_frame()`。服务进程只运行采集和检测逻辑，并在收到 SIGTERM 后调用 `release()`。
