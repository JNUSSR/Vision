# MID-360S 定位对比

本项目基于 FAST-LIO2 的 ROS 2 移植版 Ericsii/FAST_LIO_ROS2，基线提交为 `2fffc570a25d0df172720bac034fbdb6a13d2162`。同一套程序以 `static_confidence` 启动参数选择两种模式。关闭时为该移植版的基线行为，开启时加入静态参考权重。

## 方法与限制

开启后，算法开始得到有效位姿时采集 5 秒。在此期间保持雷达和支架静止。每帧将点转换到初始地图坐标系，统计 0.5 米体素在多少帧出现。至少 20 帧且出现比例达到 0.6 的体素被认为可信。采样期间估计位移超过 0.1 米或旋转超过 0.1 弧度、帧数不足、没有可信体素时，程序提示错误并回退到基线权重。位姿估计本身无法保证传感器确实静止，实验前应手动固定设备。

完成采样后参考地图冻结。点到平面匹配的五个地图邻居中，可信邻居比例越高，该项匹配权重越接近 1，其他匹配的默认权重为 0.3。权重同时作用于残差和雅可比。地图本身仍会加入环境中的点，因此这不是行人检测或动态点剔除，也不能保证动态场景定位准确。一直静止的人也可能被判成可信；初始视野外的新场景会被赋予较低权重。参数是实验起点，需用实测数据调整。初始场景应包含不同朝向的墙、地面等稳定结构，单面平墙可能导致部分运动方向约束不足。

## 当前设备状态

已在这台 Windows + WSL Ubuntu 22.04 + ROS 2 Humble 电脑上验证设备发现、点云 UDP、IMU UDP 和 ROS 发布。设备发现包报告 `dev_type=9`，Livox SDK2 将其识别为 `MID360`；因此 `MID360s_config.json` 中使用 `MID360` 配置键，设备序列号为 `47MCN8A0037291`。当前网络地址为雷达 `192.168.1.191`、主机有线网卡 `192.168.1.50`。

驱动发布 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`，约 10 Hz）和 `/livox/imu`（`sensor_msgs/msg/Imu`，约 200 Hz）。原始 bag 已成功录制 54 秒，包含 542 帧点云和 10,834 帧 IMU；抽样点云每帧约 2 万点，4 条线和 FAST-LIO 使用的 tag 均有效。FAST-LIO 曾输出约 10 Hz `/Odometry`，加权模式曾采到 51 帧静态参考；但后续重复运行出现扫描为空和静态参考回退，且同一 bag 的离线重放尚未稳定复现。因此端到端的可重复定位对比仍未验收，先按下面流程实测，并以每次运行的日志为准。

## 启动

在 Ubuntu/WSL 终端中，每个步骤各开一个终端。Viewer2 需要退出，避免它和 ROS 驱动争用雷达连接。

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh driver
```

检查话题和频率：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=0
ros2 topic list -t
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

查看实时定位。启动后先固定设备，确认有 `/Odometry` 输出；加权模式还必须看到 `Static reference ready`，出现回退错误时该次不算加权实验：

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh fastlio baseline
```

要运行加权版本，把 `baseline` 改成 `weighted`。只有确认 `/Odometry` 持续更新，且日志出现 `Static reference ready` 后，才开始移动。日志出现 `No point, skip this scan` 持续刷屏或静态参考回退时，停止该次运行并检查输入话题/时间同步；不能把它记录为有效加权组。

## 同数据对比

先实机录一份同时包含点云和 IMU 的原始 bag，再分别让基线和加权模式从头处理这同一份 bag。录制开头至少静止 10 秒；之后沿地面标记走直线 1 米，再原路返回。用卷尺标记起点、终点和 1 米位置。不要录完以后再移动设备，移动过程必须包含在这份 bag 里。

### 录制输入

退出 LivoxViewer2。在 Ubuntu/WSL 终端 1 启动驱动：

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh driver
```

终端 2 开始录制，目录名每次必须不同：

```bash
mkdir -p ~/bags
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh record-raw ~/bags/mid360_1m_20261119
```

看到 rosbag 开始订阅后，保持设备固定 10 秒，再完成 1 米直线往返；整个过程都在录制期间完成。随后在录制终端按 Ctrl+C，等待 rosbag 写完索引，再在驱动终端按 Ctrl+C。不要直接关闭终端。

检查记录完整性：

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh inspect ~/bags/mid360_1m_20261119
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh info ~/bags/mid360_1m_20261119
```

检查输出应同时包含 `/livox/lidar` 和 `/livox/imu`，且点云样本有大量有效点。检查不通过就重新录制。

### 生成基线轨迹

每次离线处理用三个 WSL 终端。确认实时驱动已停止。

终端 1：

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh fastlio baseline
```

终端 2：

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh record-odom ~/bags/baseline_odom
```

终端 3 以暂停状态打开原始 bag：

```bash
bash /mnt/f/Visual_Studying/Mid360/scripts/mid360_ros.sh replay-compare ~/bags/mid360_1m_20261119
```

播放器启动后先等约 10 秒，让话题发布者、订阅者有时间建立连接，再在播放器终端按空格开始。观察 FAST-LIO 终端确认没有持续的 `No point, skip this scan`，且 `/Odometry` 在更新；否则按 Ctrl+C 停止这组，不能把空轨迹当结果。回放自然结束后，按 Ctrl+C 停止终端 2 和终端 1。确保等终端 2 完成 bag 写入。

### 生成加权轨迹

用新的三个终端重复相同步骤：终端 1 把 `baseline` 改成 `weighted`，终端 2 把输出名改成 `weighted_odom`，终端 3 仍播放同一份 `mid360_1m_20261119`。加权节点必须显示 `Static reference ready`；如果超时、持续报告 `No point, skip this scan` 或静态参考回退，该组结果无效。

最后比较两份 `/Odometry`：走到 1 米标记处的估计位移、返回起点后的残余偏移，以及轨迹是否抖动。地面标记和卷尺是距离真值。两组收到同一 bag 能控制传感器运动和人员经过的差异，但不能替代真实距离测量。
