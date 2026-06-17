# Gazebo VLP-16 Point Time Field

## 背景

FAST-LIO2 需要每个 LiDAR 点在一帧扫描内的相对时间，用于结合 IMU 做运动去畸变。这个相对时间通常会被放进 `PointCloud2` 的某个字段，并在本仓库中转换到 `pcl::PointXYZINormal::curvature`。

不同雷达驱动或仿真插件使用的字段名可能不同：

- Livox 或部分点云转换节点常见字段名：`timestamp`
- Gazebo Velodyne/VLP-16 插件常见字段名：`time`

如果前端只识别 `timestamp`，而 Gazebo VLP-16 发布的是 `time`，程序就会认为点云没有点级时间，从而进入兜底估算逻辑。兜底时间不一定符合真实扫描模型，运动时容易导致去畸变错误。

## 现象

典型表现包括：

- 机器人静止时地图看起来基本正常；
- 机器人开始运动或转弯后，轨迹快速漂移；
- 墙面、柱子等结构被拉弯、撕裂或重影；
- 调整 `lidar_cov_inv`、`scan_resolution`、`map_resolution` 后改善有限；
- PGO 后端无法根治，因为前端 LIO 输出已经有较大误差。

## 当前 go2 仿真中的字段

当前工程的 VLP-16 来自：

```text
Quadruped_sim/velodyne_simulator/velodyne_gazebo_plugins/src/GazeboRosVelodyneLaser.cpp
```

该插件发布的 `PointCloud2` 字段中包含：

```text
x, y, z, intensity, ring, time
```

其中 `time` 字段为 `FLOAT32`，偏移量为 18。

需要注意：该插件源码中目前把每个点的 `time` 写成 `0.0`。这意味着虽然字段名存在，但没有真实的点级相对扫描时间。此时程序会把所有点视为同一时刻采集，避免错误的长时间跨度，但高速运动时仍可能存在运动畸变误差。

## 本仓库的处理

`fastlio2/src/utils.cpp` 中的 `Utils::cloud2PCL()` 已兼容两种字段名：

```text
timestamp
time
```

处理逻辑为：

- 如果存在 `timestamp` 或 `time`，且点时间有有效跨度，则使用该字段计算一帧内相对时间；
- 如果字段存在但所有点时间相同，例如 Gazebo VLP-16 全部为 `0.0`，则将相对时间置为 `0.0`；
- 如果没有点时间字段，则按一帧约 `100ms` 估算点的相对时间，只适合低速调试。

## 如何检查

启动仿真后查看点云字段：

```shell
ros2 topic echo /velodyne_points --once
```

重点检查：

- `fields` 中是否存在 `time` 或 `timestamp`；
- 字段数据类型是否为 `FLOAT32`、`FLOAT64` 或 `UINT32`；
- 点时间是否随扫描点递增；
- 是否所有点时间都为 `0.0`。

也可以只看 topic 类型和频率：

```shell
ros2 topic info /velodyne_points
ros2 topic hz /velodyne_points
```

## 与雷达型号的关系

这个问题看起来和“雷达型号”有关，但根因不是 VLP-16 几何模型本身，而是雷达驱动/仿真插件发布的点云字段格式。

同样是 VLP-16：

- 真实驱动可能提供正确的点级 `time`；
- Gazebo 插件可能只有字段但值全为 `0.0`；
- 其他转换节点可能改名为 `timestamp`，也可能丢掉点级时间。

因此更换雷达型号、驱动、仿真插件或点云转换节点后，都需要重新确认点云字段。

## 调参建议

如果点级时间不正确，优先修数据，而不是盲目调参数。

推荐排查顺序：

1. 确认 LiDAR 和 IMU 使用同一时钟源；
2. 确认点云存在正确的点级相对时间；
3. 确认 `r_il/t_il` 外参与 URDF 或标定结果一致；
4. 低速验证前端 LIO；
5. 再调整 `lidar_cov_inv`、`scan_resolution`、`map_resolution` 等参数。

当前 go2 + Gazebo VLP-16 的保守起点：

```yaml
lidar_filter_num: 2
scan_resolution: 0.2
map_resolution: 0.3
gravity_align: true
lidar_cov_inv: 800.0
```

如果仍然高速漂移，应考虑修改 Gazebo Velodyne 插件，让每个点写入真实的一帧内相对扫描时间。
