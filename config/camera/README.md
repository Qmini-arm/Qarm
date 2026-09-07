# LRCP20680 相机内参使用说明

此目录只保存后续图像定位可以使用的内参结果，不包含标定程序、训练照片、验证照片或逐图报告。
配置来自 2026-09-07 15:01（北京时间）补拍后完成的标定；不是厂家通用参数。

## 文件与适用条件

- `lrcp20680_1280x720_focus501.json`：推荐读取格式，包含 K、D、标定条件和质量摘要。
- `lrcp20680_1280x720_focus501.yaml`：相同数值的 OpenCV FileStorage 格式。
  文件中的 `%YAML:1.0` 和 `!!opencv-matrix` 为 OpenCV 格式，请使用 `cv2.FileStorage` 读取。

| 条件 | 配置 |
|---|---|
| 相机 | 本次实际标定的 LRCP20680；USB VID:PID 为 `0bda:3035` |
| 原始图像尺寸 | **1280×720** |
| 传输格式 | MJPEG |
| 连续自动对焦 | **关闭** |
| 手动焦点 | **501**，设备控制单位，不是毫米焦距 |
| 模型 | OpenCV 普通针孔 + 五参数畸变 |

同型号和 VID/PID 不保证是同一镜头/同一物理相机，不能据此向其他设备通用。
更换相机、镜头、焦点或输出裁剪模式后需重新验证/标定。不要直接将本配置用于 1080p。
本参数基于原始未旋转、未镜像、未裁剪、未去畸变的图像。

## 数据语义

JSON 的 `camera_matrix` 是 3×3 矩阵 K，`image_size` 顺序为 `[width, height]`。
`fx, fy, cx, cy` 的单位是像素。

```text
K = [[632.56069721,   0,            663.06472131],
     [  0,          630.76234420,  352.01938149],
     [  0,            0,            1          ]]
```

`distortion_coefficients` 顺序是 **k1, k2, p1, p2, k3**，无量纲。使用文件中的完整精度数值。
JSON 的 `board` 记录 10×7 内角点、标称/输入方格边长 15 mm，是标定来源说明，不是手眼外参。

本轮有 35 张训练图、11 张独立验证图，训练 RMS 约 **0.275 px**，验证 RMS 约 **0.246 px**。
验证误差固定 K/D、重新拟合每张图片的板位姿后计算；不代表独立的毫米测量精度。
此次通过初步内参质量检查，但未验收抓取精度，所以 `accepted_for_robot_control` 保持 `false`。
这个字段不是加载开关，也不要手工修改它来代替实机验收。

## 使用前恢复焦点

Linux / WSL 中先通过 `v4l2-ctl --list-devices` 找到实际视频节点（本机标定时为 `/dev/video0`）：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=501
v4l2-ctl -d /dev/video0 --get-ctrl=focus_automatic_continuous,focus_absolute
```

重插或重启后设置可能复位。确认图像在实际工作距离清晰，再进行定位。
后续采集程序必须检查实际输出确实为 1280×720，而非仅发出分辨率请求。

## 后续程序读取示例

JSON 可用 Python 标准库读取；下面的图像处理示例另外需要应用自己的 NumPy/OpenCV 环境。
本仓库不因保存结果而新增 OpenCV 运行依赖。代码以仓库根目录为当前工作目录：

```python
import json
from pathlib import Path

import cv2
import numpy as np

path = Path('config/camera/lrcp20680_1280x720_focus501.json')
params = json.loads(path.read_text(encoding='utf-8'))
K = np.asarray(params['camera_matrix'], dtype=np.float64)
D = np.asarray(params['distortion_coefficients'], dtype=np.float64)
size = tuple(params['image_size'])

# frame 由应用采集，必须符合上述相机/焦点/图像条件。
assert frame.shape[1::-1] == size
undistorted = cv2.undistort(frame, K, D, None, K)
```

上述调用明确使用 K 作为输出相机矩阵，因此去畸变图的后续几何计算使用 **K + 零畸变**。
如果改用 `getOptimalNewCameraMatrix`，应使用它返回的新矩阵；如果进一步裁剪图像，还需更新主点坐标。
不要对去畸变图再次套用原 D。

在原始图上定位已知尺寸目标时，可将原始像素点与 K/D 一起传给 `solvePnP`；
仅有目标像素和内参无法确定未知目标深度。

YAML 读取方式：

```python
fs = cv2.FileStorage(
    'config/camera/lrcp20680_1280x720_focus501.yaml', cv2.FILE_STORAGE_READ
)
try:
    if not fs.isOpened():
        raise OSError('Cannot open camera calibration')
    K = fs.getNode('camera_matrix').mat()
    D = fs.getNode('distortion_coefficients').mat()
finally:
    fs.release()
```

## 接入机械臂的边界

此配置只描述相机成像，不包含相机到 `tool0` 的手眼变换，不会自动修改关节零位、配置电机或执行运动。
后续抓取还需手眼标定及已知工作平面/物体几何或其他深度来源，并用实际目标检查位置误差。
运动层使用米时，应显式转换视觉位姿中的毫米单位。

## 本地工具与数据

下列路径已被 Git 忽略，不会随克隆取得：

- `python/qmini_arm_vision/`、`tests/python/test_camera_calibration.py`：本地相机工具与测试。
- `docs/camera_calibration.md`：本地全流程标定操作说明。
- `build/camera/`：原始照片、角点图、报告、验证输出。
- `config/camera/generated/`：每轮求解的完整原始输出。

已有独立目录 `/home/wyt06/unitree-arm/lrcp_camera_viewer` 也保持不变。
仓库的包依赖和入口恢复为不包含相机标定工具的状态；Git 中不要求安装或运行它们。
本机仍可在已安装 OpenCV 的环境中从仓库根目录运行：

```bash
PYTHONPATH="$PWD/python${PYTHONPATH:+:$PYTHONPATH}" .venv/bin/python -m qmini_arm_vision.calibration --help
```

若之后同步仓库虚拟环境导致 OpenCV 被移除，可使用独立目录的原环境运行本地包。
共享的新内参应先验证，再显式更新这里两个同名配置文件，并同步适用条件和质量摘要。
