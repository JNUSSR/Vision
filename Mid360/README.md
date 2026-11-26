# MID-360S 点云录制与定位

本项目在 Windows 的 WSL 2 Ubuntu 中采集 Livox MID-360S 的点云和 IMU，支持 ROS 2 bag 录制、数据检查、离线回放，以及 FAST-LIO 和 FAST-LIVO2 雷达＋IMU 实时定位。FAST-LIO 另提供静态可信度实验模式。各项目来源与许可信息见 [SOURCES.md](SOURCES.md)。

## 环境与编译

以下操作在 Ubuntu 22.04、ROS 2 Humble 中执行。先安装 ROS 2 Humble，并确认 `/opt/ros/humble/setup.bash` 存在。项目使用 Cyclone DDS。安装依赖后下载源码并编译。

```bash
sudo apt update
sudo apt install -y git cmake build-essential python3-colcon-common-extensions \
  libapr1-dev libeigen3-dev libopencv-dev libfmt-dev libboost-all-dev libpcl-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-sophus ros-humble-cmake-modules \
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2
git clone https://github.com/JNUSSR/Vision.git /mnt/f/Visual_Studying/Vision
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/build_workspace.sh
```

默认工作空间为仓库上一级目录下的 `mid360_ws`，例如 `/mnt/f/Visual_Studying/mid360_ws`，用于将编译产物保存在 F 盘。已有仓库可跳过上面的 `git clone`。编译需要一定时间和内存。需要指定其他路径时，在构建和运行的每个终端设置同一个变量，例如 `export MID360_WORKSPACE=~/my_mid360_ws`。减少并发可使用 `MID360_BUILD_JOBS=1 bash scripts/build_workspace.sh`。

已有分开的工作空间也可使用 `MID360_LIVOX_WS`、`MID360_LIVO_WS` 和 `MID360_SDK_PREFIX` 指定路径。脚本默认 ROS 路径可用 `MID360_ROS_SETUP` 修改。工作空间必须包含与此仓库源码兼容的驱动、算法及转换节点。

## 雷达网络

雷达通过有线网口发送数据，可以使用 USB 转以太网适配器连接电脑。按设备实际配置设置电脑网卡与雷达 IP，并确保 WSL 能访问该网卡。这里的默认配置只是本次设备的例子。

- 电脑网卡 `192.168.1.50`，子网掩码 `255.255.255.0`。
- 雷达 `192.168.1.191`。
- 主机 UDP 端口 `56000、56101、56201、56301、56401、56501`。

Windows 和 WSL 需要允许相应网络流量。采用 WSL 镜像网络时，可在 Windows 用户目录的 `.wslconfig` 启用 `networkingMode=mirrored`，执行 `wsl --shutdown` 后重新打开 Ubuntu。防火墙应允许上述 UDP 端口以及实际雷达 IP。运行前使用 `ip -4 addr` 检查 Ubuntu 可见的网卡地址。关闭 LivoxViewer2 后启动驱动，避免同时争用设备或端口。

默认网络配置在 `livox_ros_driver2/config/MID360s_config.json`。更换设备时，复制该文件并按实际情况修改 `host_ip` 和 `lidar_configs` 中的 `ip`，在启动驱动或实时程序的终端设置以下变量。无需修改仓库中的示例配置。

```bash
cp /mnt/f/Visual_Studying/Vision/Mid360/livox_ros_driver2/config/MID360s_config.json ~/mid360_network.json
nano ~/mid360_network.json
export MID360_LIDAR_CONFIG=~/mid360_network.json
```

## 录制和检查

每次打开 Ubuntu 后先进入仓库。以下各命令分别在独立终端执行。

终端 1 启动驱动，保持运行。

```bash
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh driver
```

终端 2 查看数据话题。

```bash
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh topics
```

确认出现 `/livox/lidar` 和 `/livox/imu` 后录制，路径每次使用新名称。开始移动前保持雷达静止，并确认终端显示点云和 IMU 持续增加。按 Ctrl+C 停止，等待保存结果。

```bash
mkdir -p /mnt/f/Visual_Studying/bags
bash scripts/mid360_ros.sh record-raw /mnt/f/Visual_Studying/bags/mid360_001
```

录制器每秒显示两路数据数量和接收频率，连续 5 秒收不到数据时会警告。停止后会检查实际写入的消息数量，任一路为零会报告失败。已有目录不会被覆盖，请更换名称。

```bash
bash scripts/mid360_ros.sh info /mnt/f/Visual_Studying/bags/mid360_001
bash scripts/mid360_ros.sh inspect /mnt/f/Visual_Studying/bags/mid360_001
```

`info` 显示 bag 总时长及话题数量，`inspect` 检查前几帧点云及 IMU。若没有数据，检查供电、USB 网卡、网卡地址、WSL 网络、防火墙和驱动日志。

## 实时坐标

启动后保持雷达静止，等待点云、IMU 和所选算法开始更新。程序通常会自动启动驱动，结束时按 Ctrl+C。

```bash
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh live --algorithm lio
bash scripts/mid360_ros.sh live --algorithm livo2
bash scripts/mid360_ros.sh live --algorithm both
```

三条命令分别运行 FAST-LIO、FAST-LIVO2 雷达＋IMU、两者同时运行。各算法以自己的首帧位置为原点，显示 X、Y、Z，单位为厘米，坐标轴由算法初始化确定。单独运行某个算法有助于减少计算负载。

“最近更新”是程序收到最新定位消息的时间间隔。FAST-LIVO2 的“数据延迟”是最新 IMU 传感器时间戳减去定位结果对应的传感器时间戳，反映处理滞后。延迟达到 1 秒时会提示数据积压。它依赖两种消息使用可比较的传感器时间戳。运行日志和逐秒 `latency.csv` 保存在 `~/.ros/log/mid360_live_日期_时间_进程号/`。

较长时间运行时 FAST-LIVO2 曾出现数据积压，具体原因仍待进一步排查。坐标仍持续更新不代表它一定对应当前时刻。

## 用同一份 bag 离线回放

关闭实时驱动和 LivoxViewer2。在终端 1 启动算法，`--no-driver` 使用回放数据。

```bash
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh live --algorithm both --no-driver
```

在终端 2 回放数据。

```bash
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh play /mnt/f/Visual_Studying/bags/mid360_001
```

回放结束后，在终端 1 按 Ctrl+C。重新启动并选择 `lio` 或 `livo2` 可重复使用同一份输入。也可以使用 `replay-compare` 代替 `play`，播放器会先暂停，按空格开始。开始时静止几秒，给算法完成初始化。

FAST-LIO 静态可信度实验使用两个终端，算法模式可以选择 `baseline` 或 `weighted`。

```bash
# 终端 1
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh fastlio weighted

# 终端 2
cd /mnt/f/Visual_Studying/Vision/Mid360
bash scripts/mid360_ros.sh replay-compare /mnt/f/Visual_Studying/bags/mid360_001
```

如需保存 FAST-LIO 输出，在回放前另开一个终端，运行 `bash scripts/mid360_ros.sh record-odom /mnt/f/Visual_Studying/bags/odom_weighted_001`，回放结束后按 Ctrl+C 保存。换用 `baseline` 并为输出指定新名称，即可使用同一份 bag 比较。静态可信度默认在开始约 5 秒内采样，采样期间雷达应保持静止。如果采样无效或帧数不足，程序会退回基线权重，请检查终端日志。

示例中的新录制保存在 F 盘 `F:\Visual_Studying\bags`。已有录制仍可使用原来的 `~/bags` 路径。录制文件、定位结果、日志和编译产物保存在本机，不属于仓库内容。
