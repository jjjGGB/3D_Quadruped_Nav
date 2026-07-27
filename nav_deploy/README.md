# nav_deploy

`nav_deploy` 是 SEA-Nav 的 Go2 ROS2 Humble Gazebo 部署包。它运行导航 ONNX
策略并发布 `/cmd_vel`，现有 `rl_sar/rl_sim` 再用 Go2 HIMLoco 策略把速度命令转换为
12 个关节控制量。这里不包含 A2，也不会绕过真实的四足低层控制链路。

## 数据链路

```text
/sea_nav/scan + /gazebo/model_states + /goal_pose
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
python3 -m pip install -r nav_deploy/requirements.txt
colcon build --symlink-install --packages-up-to nav_deploy rl_sar go2_description
source install/setup.bash
ros2 run nav_deploy verify_onnx
ros2 launch nav_deploy sea_nav_sim.launch.py
```

启动文件会加载现有 `rl_sar` Gazebo 世界，在 `(-5, 7)` 生成 Go2，5 秒后启动
`rl_sim`，自动起身、打开导航模式，并向默认目标 `(0, 7)` 导航。可通过 RViz2 的
2D Goal Pose 或命令行更新目标：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 2.0, y: 7.0}, orientation: {w: 1.0}}}"
```

调试输出位于 `/sea_nav/raw_action` 和 `/sea_nav/filtered_action`。激光或状态超过
0.5 秒未更新、推理输出非有限数、或到达目标时，节点会立即发布零速度。

## 独立接入

已有 Gazebo/机器人栈时运行：

```bash
ros2 launch nav_deploy sea_nav.launch.py
```

真实机器人或定位系统接入时，把配置中的 `state_source` 改为 `odom`，设置正确的
`odom_topic` 与 `state_frame`。目标不在状态坐标系时节点通过 TF2 转换；必须确保
对应 TF 可用。`odom_twist_in_body_frame` 用于声明里程计 twist 是否已经是机体系。

模型接口和导出数值误差记录在 `models/sea_nav_go2.json`。重新导出仍应在原
SEA-Nav 训练仓库的 Python 环境中完成：

```bash
python3 -m pip install -r nav_deploy/requirements-export.txt
python3 nav_deploy/tools/export_go2_onnx.py \
  --sea-nav-root ../SEA-Nav-Code \
  --checkpoint ../SEA-Nav-Code/training/legged_gym/logs/Go2_pos_rough/05_19_10-41-47_/model_2000.pt
```

脚本从 checkpoint 恢复 `DifferentiableSafeActorCritic`，仅导出导航 actor 与 CBF
安全层，并自动执行 PyTorch 原模型、导出包装模型和 ONNX Runtime 三方数值对比。
