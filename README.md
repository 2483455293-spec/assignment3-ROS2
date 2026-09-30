# Hikrobot Camera ROS 2 Driver

基于海康机器人 MVS SDK 的 ROS 2 Humble 相机驱动。节点负责发现并连接 Hikrobot USB3 Vision / GigE 相机、采集图像、发布 ROS 图像消息，并提供曝光、增益、目标帧率和像素格式参数。

## 功能

- 通过相机 IP 或序列号选择设备；选择器都为空时，仅在发现一台相机时自动连接。
- 发布 `sensor_msgs/msg/Image`，图像话题可配置。
- 连接失败或采集期间设备掉线时周期性重试；重新连接后重新应用当前参数。
- 支持设置曝光时间、增益、目标帧率和像素格式。手动设置曝光/增益时关闭相应自动模式；切换像素格式前暂停采集，并验证该格式是否受相机支持。
- 在独立话题发布实际采集/发布帧率，和相机设置的目标帧率区分。

## 环境与依赖

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`、`rosdep` 及 `package.xml` 声明的 ROS 依赖
- 海康机器人 Linux MVS SDK，与目标 CPU 架构及相机型号兼容

ROS 依赖可以由 `rosdep` 安装。MVS 是厂商 SDK，没有对应的 `rosdep` 规则，必须从[海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0)另行下载和安装。下载适用于目标 Linux 发行版和架构的 SDK，并按 SDK 安装包文档完成安装。当前开发环境链接的是 `libMvCameraControl.so.4.8.2.1`；其他 SDK 版本请以其附带的开发文档为准。

构建系统需要能找到 MVS 头文件 `MvCameraControl.h` 和库 `libMvCameraControl.so`。若 SDK 安装在非标准位置，在构建命令中传入其根目录，例如 `/opt/MVS`。运行时若找不到共享库，请依照对应 MVS 版本的安装说明配置系统动态库搜索路径。

> 未安装 MVS 时，功能包可以编译，但真实相机节点不能连接设备或采集图像。

## 获取 ROS 依赖与构建

在工作空间根目录打开终端：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select hikrobot_camera \
  --cmake-args -DMVS_ROOT=/opt/MVS
```

如果 MVS 已安装在系统标准路径，可不传 `-DMVS_ROOT`。构建完成后，在当前终端加载工作空间：

```bash
source install/setup.bash
```

本仓库根目录就是 colcon 工作空间，不需要将整个仓库再嵌套到另一个工作空间的 `src` 目录中。

## 连接并运行相机

确认相机已接通、系统能够识别，且没有被 MVS 客户端等其他程序占用。运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hikrobot_camera camera.launch.py
```

成功打开相机时，节点日志会显示 `Camera connected`。连接失败时节点会按配置周期重试，并输出 MVS 错误码。若通过 WSL 使用 USB 相机，需先在 Windows 上用 `usbipd` 将设备附加到 WSL；`lsusb` 能列出设备并不一定表示 USB 数据传输正常。

可通过自定义参数文件启动：

```bash
ros2 launch hikrobot_camera camera.launch.py \
  params_file:=/absolute/path/to/camera.yaml
```

## 参数

默认值位于 [`src/hikrobot_camera/config/camera.yaml`](src/hikrobot_camera/config/camera.yaml)。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---:|---|
| `camera_ip` | string | `""` | GigE 相机 IP；留空则不按 IP 筛选。 |
| `serial_number` | string | `""` | 相机序列号；留空则不按序列号筛选。多个相机匹配时需指定选择器。 |
| `image_topic` | string | `/image_raw` | 图像发布话题。 |
| `image_qos_reliability` | string | `reliable` | 图像话题的 QoS 可靠性，取值 `reliable` 或 `best_effort`；只能在启动时设置。 |
| `actual_frame_rate_topic` | string | `/camera/actual_fps` | 实际图像发布帧率话题，消息类型为 `std_msgs/msg/Float64`。 |
| `exposure_us` | double | `10000.0` | 手动曝光时间，单位微秒，允许范围 `[1, 10000000]`；相机仍可能有更窄的能力范围。 |
| `gain_db` | double | `0.0` | 手动增益，单位 dB，允许范围 `[0, 100]`；相机仍可能有更窄的能力范围。 |
| `frame_rate` | double | `30.0` | 写入相机的目标采集帧率，允许范围 `(0, 1000]`；不等同于实际帧率。 |
| `pixel_format` | string | `BGR8` | 输出格式；支持请求 `BGR8`、`RGB8`、`MONO8`，并由相机 SDK 验证是否可用。 |
| `reconnect_period_ms` | integer | `1000` | 相机断开或未找到时的重试间隔（毫秒；实际定时器间隔至少 100 毫秒）。 |

`image_qos_reliability` 决定图像发布端的 QoS 可靠性：`reliable` 发布者同时能匹配 `reliable` 和 `best_effort` 订阅者，因此默认值可直接与 RViz2 的 Image 显示通信；改成 `best_effort` 后，只有 `best_effort` 订阅者（例如把 Image 显示的 **QoS → Reliability Policy** 改为 *Best effort*，或用 `ros2 topic hz`）才收得到图像。发布端 QoS 不能在运行时修改，只能启动时指定。

设置参数会校验类型、范围及 SDK 返回值；设备拒绝设置时，ROS 参数请求会失败并返回原因。相机断开时，合法的运行时参数会保留，并在下次连接时应用。

例如，修改运行中节点的曝光、增益、目标帧率和像素格式：

```bash
ros2 param set /hikrobot_camera exposure_us 5000.0
ros2 param set /hikrobot_camera gain_db 3.0
ros2 param set /hikrobot_camera frame_rate 15.0
ros2 param set /hikrobot_camera pixel_format MONO8
```

## 图像和帧率检查

查看图像话题是否持续发布：

```bash
ros2 topic hz /image_raw
```

查看节点报告的实际发布帧率：

```bash
ros2 topic echo /camera/actual_fps
```

也可以在 RViz 2 中添加 **Image** 显示并选择 `/image_raw`（默认 `image_qos_reliability: reliable` 与 RViz2 的默认 QoS 兼容，无需额外设置；若改成 `best_effort`，需同时把 Image 显示的 **QoS → Reliability Policy** 设为 *Best effort*）。若需要查看 topic 的消息类型和发布者：

```bash
ros2 topic info /image_raw
```

实际帧率会受到相机能力、曝光时间、接口带宽、主机负载和丢帧影响。目标帧率表示向相机配置的采集速率；`/camera/actual_fps` 是节点实际取得并发布的图像帧率。

## 已知限制与验收状态

- 支持的输出格式限于 `BGR8`、`RGB8` 和 `MONO8`；其他 MVS 格式不在本节点支持范围内。
- 相机硬件、USB/GigE 链路和相机固件决定具体参数范围及可用功能。节点构建成功不代表已完成实机验证。
- 在 WSL 中，USB 设备转发可能影响 MVS 控制和图像传输；若出现 USB 错误，请先确认 Windows 端设备已附加到 WSL，再检查 SDK 错误码和 USB 内核日志。
- 当前环境中曾遇到相机枚举成功但 `MV_CC_OpenDevice` 返回 `0x80000301`（MVS USB 写入错误）。因此真实相机的打开、连续图像发布、RViz 显示、参数成像效果及断线重连，需要在 USB 通信恢复后继续实机验证。


## 目录结构

```text
src/hikrobot_camera/
├── CMakeLists.txt
├── package.xml
├── config/camera.yaml
├── include/hikrobot_camera/camera_node.hpp
├── launch/camera.launch.py
└── src/
    ├── camera_node.cpp
    └── main.cpp
```
