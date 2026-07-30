# nav_deploy

`nav_deploy` 是 SEA-Nav 的 Go2 ROS2 Humble Gazebo 部署包。它运行导航 ONNX
策略并发布 `/cmd_vel`，现有 `rl_sar/rl_sim` 再用 Go2 HIMLoco 策略把速度命令转换为
12 个关节控制量。这里不包含 A2，也不会绕过真实的四足低层控制链路。

## 数据链路

```text
/velodyne_points (PointCloud2)
             |
   pointcloud_to_laserscan
    [-120 deg, 120 deg]
             |
 /sea_nav/scan (41 ranges) + /gazebo/model_states + /goal_pose
             |
      SEA-Nav ONNX (550 -> 3)
             |
   /cmd_vel [vx, vy, yaw_rate]
             |
    rl_sar Go2 HIMLoco -> joints
```

策略观测严格保持训练顺序：10 帧历史，每帧为重力投影(3)、上一帧滤波命令(3)、
机体线速度(3)、机体角速度(3)、`log2` 激光距离(41)、机体坐标系目标点(2)，合计
`10 x 55 = 550`。首帧会填满整个历史缓存。输出滤波系数为 0.5，限幅为
`vx [-0.5, 2.0]`、`vy [-1.0, 1.0]`、`yaw_rate [-1.0, 1.0]`。

## 安装和快速验证

在包含本仓库各功能包的 ROS2 Humble 工作空间中执行：

```bash
source /opt/ros/humble/setup.bash
# 若 pointcloud_to_laserscan 是在其他工作空间源码安装的，先 source 该工作空间。
source /home/complexity/robot_ws/install/setup.bash
python3 -m pip install -r nav_deploy/requirements.txt
# 本机用户级 setuptools 可能与 Humble 的 colcon/ament 不兼容；只在构建命令
# 隔离 user site，运行时不能保留该变量，否则找不到 pip 安装的 onnxruntime。
PYTHONNOUSERSITE=1 colcon build --symlink-install \
  --packages-up-to nav_deploy rl_sar go2_description \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source install/setup.bash
ros2 run nav_deploy verify_onnx
ros2 launch nav_deploy sea_nav_sim.launch.py
```

启动文件会加载现有 `rl_sar` Gazebo 世界，在 `(-5, 7)` 生成 Go2，同时把
`/velodyne_points` 转成 41 维 `/sea_nav/scan`。5 秒后启动 `rl_sim`，自动起身、
打开导航模式，并向默认目标 `(0, 7)` 导航。RViz2 默认一同启动，Fixed Frame 为
`world`；点击工具栏的 **2D Goal Pose** 即可向 `/goal_pose` 发送新目标。也可以用
命令行发送：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 2.0, y: 7.0}, orientation: {w: 1.0}}}"
```

调试输出位于 `/sea_nav/raw_action` 和 `/sea_nav/filtered_action`。激光或状态超过
0.5 秒未更新、推理输出非有限数、或到达目标时，节点会立即发布零速度。

无图形环境下可关闭 RViz：

```bash
ros2 launch nav_deploy sea_nav_sim.launch.py use_rviz:=false
```

## 关键接口说明

- 41 维不能简单配置成 `angle_min=-120 deg`、`angle_max=120 deg`、
  `angle_increment=6 deg`。`pointcloud_to_laserscan` 使用 `ceil((max-min)/step)`
  分配数组，这样只会产生 40 维。配置给上界增加 `1e-6 rad`，实际第 41 个角度仍
  恰好是 `120 deg`。节点首次收到数据时会打印输入维数和策略维数。
- `pointcloud_to_laserscan` 使用传感器数据的 Best Effort QoS 发布 LaserScan；
  RViz 的 LaserScan Display 必须使用相同可靠性。若设为 Reliable，DDS 会报告
  `incompatible QoS`，RViz 将收不到 `/sea_nav/scan`，即使策略节点仍可正常推理。
- `/gazebo/model_states` 没有 TF。策略节点在仿真模式下广播动态
  `world -> base_link`，再与 `robot_state_publisher` 的关节 TF 相连，否则 RViz
  发出的目标无法可靠转换到策略使用的 world 坐标系。Go2 原有的
  `world_tf_publisher` 会另发 `map -> odom -> base_link`，造成 `base_link` 双父节点，
  因此已在 Xacro 中禁用，由部署节点独占根 TF。
- `gazebo_ros2_control` 会把生成后的整段 URDF 再作为 ROS 参数 YAML 解析。因此
  Xacro 中进入最终 URDF 的注释不能包含“冒号后紧跟空格”，否则控制插件会报
  `Couldn't parse parameter override rule`，随后所有 controller spawner 都会一直等待。
- Go2 原 VLP-16 配置会在 `0.9 m` 内丢弃点，近障碍信息无法由后处理恢复。
  `go2_description/xacro/robot.xacro` 已把原始点云量程改为模型对应的
  `0.1--5.0 m`；转换器还会按 `base_link` 高度 `[-0.15, 0.35] m` 去掉地面和
  过高点。
- `/cmd_vel` 只有在 `rl_sim` 已起身且 Navigation Mode 打开时才进入 HIMLoco
  观测。`rl_sar_autostart` 通过 action 顺序完成这两个状态切换，不能仅看到
  `/cmd_vel` 有数据就认为低层已经执行。

## 独立接入

已有 Gazebo/机器人栈时运行：

```bash
ros2 launch nav_deploy sea_nav.launch.py
```

该 launch 默认也从 `/velodyne_points` 启动点云转换。已有 LaserScan 时使用：

```bash
ros2 launch nav_deploy sea_nav.launch.py \
  use_pointcloud:=false scan_topic:=/your_scan
```

真实机器人或定位系统接入时，把配置中的 `state_source` 改为 `odom`，设置正确的
`odom_topic` 与 `state_frame`。目标不在状态坐标系时节点通过 TF2 转换；必须确保
对应 TF 可用。`odom_twist_in_body_frame` 用于声明里程计 twist 是否已经是机体系。

模型接口和导出数值误差记录在 `models/sea_nav_go2.json`。导出脚本内置了 Actor
和 CBF 推理结构，不依赖 SEA-Nav 训练源码：

```bash
python3 -m pip install -r nav_deploy/requirements-export.txt
python3 nav_deploy/tools/export_go2_onnx.py \
  --checkpoint nav_deploy/models/model_2000.pt \
  --output nav_deploy/models/sea_nav_go2.onnx
```

省略参数时，上述 checkpoint 和输出路径都是默认值，可直接运行脚本。脚本严格
恢复 checkpoint，仅导出导航 Actor 与 CBF 安全层，并自动执行 PyTorch 完整模型、
导出包装模型和 ONNX Runtime 三方数值对比。
