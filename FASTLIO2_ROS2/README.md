# FASTLIO2 ROS2

基于 [FASTLIO2](https://github.com/hku-mars/FAST_LIO) 的 ROS2 激光惯性里程计，包含建图、回环优化、地图保存与重定位功能。

## 环境依赖
- Ubuntu 22.04
- ROS2 Humble
- 依赖库：pcl、Eigen、Sophus、gtsam、livox_ros_driver2

> 编译依赖的详细安装步骤（LIVOX-SDK2 / livox_ros_driver2 / Sophus）见文末[附录](#附录编译依赖安装)。

## 编译
```shell
cd your_ws
colcon build
source install/setup.bash
```

## 一、建图与保存地图

建图核心是 `fastlio2`（实时里程计），但**保存地图的服务在 `pgo` 节点里**，所以建图时需要同时启动两者。

> 关于 `pgo`：`fastlio2` 自身只发布点云和里程计话题，不负责存图。`pgo` 全程订阅这些话题、累积关键帧，并提供 `/pgo/save_maps` 服务写出全局 `.pcd`。它同时会做回环检测 + 位姿图优化来消除漂移——**回环优化是附带能力（小场景可忽略），但只要想保存地图就必须启动 `pgo` 节点。**

### 1. 启动建图
```shell
# 终端1：启动里程计（建图核心）
ros2 launch fastlio2 lio_launch.py

# 终端2：启动 pgo 节点（提供保存地图服务 + 回环优化）
ros2 launch pgo pgo_launch.py

# 终端3：播放数据 / 接入实时雷达
ros2 bag play your_bag_file
```

> 如果只想看实时里程计、不需要存图，可只启动 `fastlio2`。

### 1.1 雷达点时间与漂移排查

FAST-LIO2 依赖每个 LiDAR 点的相对扫描时间做运动去畸变；漂移严重、点云撕裂、运动时地图拉花，往往不只是 `lio.yaml` 参数问题，而是点级时间、外参或同步问题。

> Gazebo VLP-16 的 `time` 字段问题可参考：[Gazebo VLP-16 Point Time Field](docs/gazebo_vlp16_time_field.md)。

不同雷达型号/驱动发布的 `PointCloud2` 字段并不完全一致：
- Livox/部分驱动可能使用 `timestamp` 字段；
- Velodyne Gazebo 插件通常使用 `time` 字段；
- 有些仿真插件虽然带 `time` 字段，但所有点都填 `0.0`，此时只能近似认为一帧内无点级时间，快速运动时仍可能有去畸变误差；
- 如果完全没有点级时间，程序会按一帧约 `100ms` 估算相对时间，这只适合低速调试，不适合作为高精度配置。

本仓库的 `fastlio2` 已兼容 `timestamp` 和 `time` 两种字段名。若更换雷达型号、仿真插件或点云转换节点，请先检查点云字段：
```shell
ros2 topic echo /velodyne_points --once
```

重点看 `fields` 中是否存在 `time` 或 `timestamp`，以及点时间是否随点序递增。若字段缺失、单位错误或所有点时间为 0，常见现象是：
- 机器人静止基本正常，运动后快速漂移；
- 转弯时点云墙面被拉弯或撕裂；
- `lidar_cov_inv`、体素分辨率等参数怎么调都不稳定。

遇到这类问题，优先处理顺序建议为：
1. 确认 LiDAR/IMU 时间戳使用同一时钟源；
2. 确认点云中有正确的点级相对时间；
3. 确认 `r_il/t_il` 与 URDF 或标定结果一致；
4. 再调 `lidar_cov_inv`、`scan_resolution`、`map_resolution` 等前端参数。

### 2. 保存地图
建图完成后调用保存服务，`file_path` 为保存目录：
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```
- 保存后目录下会生成 `your_map.pcd`，用于后续定位。
- `save_patches: true` 会额外保存子地图，供一致性地图优化（HBA）使用，不需要可设为 `false`。

## 二、定位与重定位

加载已保存的地图后，定位节点持续将里程计结果对齐到地图坐标系。

### 1. 启动定位节点
```shell
# 终端1：启动里程计
ros2 launch fastlio2 lio_launch.py

# 终端2：启动定位节点
ros2 launch localizer localizer_launch.py

# 终端3：播放数据 / 接入实时雷达
ros2 bag play your_bag_file
```

### 2. 设置初始位姿（重定位）
给定地图路径和初始位姿（位置 x/y/z，姿态 yaw/pitch/roll，单位米/弧度），触发重定位：
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{pcd_path: 'your_map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```
- 初始位姿只需大致准确，节点会通过由粗到细的两阶段 ICP 自动精配准。
- 重定位成功后会持续发布 `map -> local` 的 TF，并保持跟踪。

### 3. 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```
- `code: 0`：返回当前是否已成功定位（`valid` 字段）。
- `code: 1`：强制返回成功（用于跳过校验）。

### 4. 定位置信度话题
定位节点持续发布定位置信度，可用于监控定位质量或触发重定位：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/localizer/confidence` | `std_msgs/Float32` | 置信度，范围 `[0.0, 1.0]`，越接近 1 越可靠 |

置信度由 refine 阶段 ICP 的 fitness score（平均配准误差）映射得到：
- `score = 0` → 置信度 `1.0`
- `score ≥ refine_score_thresh` 或本帧配准失败 → 置信度 `0.0`

查看置信度：
```shell
ros2 topic echo /localizer/confidence
```

## 三、一致性地图优化（可选）
对保存的地图做全局一致性优化（需在保存地图时设置 `save_patches: true`）：
```shell
# 启动优化节点
ros2 launch hba hba_launch.py

# 调用优化服务，maps_path 为保存地图的目录
ros2 service call /hba/refine_map interface/srv/RefineMap "{maps_path: 'your maps directory'}"
```

## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb
--来自百度网盘超级会员v7的分享
```

## 附录：编译依赖安装

### 1. LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 2. livox_ros_driver2
```shell
mkdir -p ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3. Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```
> 新版 Sophus 依赖 fmt，可在 CMakeLists.txt 中添加 `add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)` 去除依赖，否则会报错。

## 特别感谢
- [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
- [BALM](https://github.com/hku-mars/BALM)
- [HBA](https://github.com/hku-mars/HBA)

## 性能提示
代码主要使用 `timerCB` 作为频率触发主函数。ROS2 中 timer、subscriber、service 的回调默认运行在同一线程，机器性能不足时可能出现回调阻塞，建议用多线程将耗时回调（如 `timerCB`）独立出来以提升性能。
