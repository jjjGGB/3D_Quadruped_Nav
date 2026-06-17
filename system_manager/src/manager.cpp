
#include "manager.h"
#include <tf2/utils.h>
#include "geometry_msgs/msg/quaternion.hpp"  
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>


manager_Interface::manager_Interface() 
    : Node("motion_plan"),
      has_valid_global_path_(false),
      has_obstacles_(false),
      should_plan_(false),
      needs_replan_(false),
      has_odom_(false),
      has_last_tf_pose_(false),
      last_v_(0.0F),
      last_w_(0.0F)
{
    loadParameters();
    createRosInterfaces();
    initPlannerBase();

    // 20Hz 主循环串起导航链路：
    // OctoPlanner 全局路径 -> octo_path_optimizer_node 三维路径平滑(/optimized_path)
    // -> manager 裁剪前方参考线 -> TEB 跟踪/动态避障 -> /cmd_vel。
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&manager_Interface::make_plan_and_control, this)
    );

    RCLCPP_INFO(this->get_logger(), "Ego Planner navigation bridge ready.");
    RCLCPP_INFO(this->get_logger(), "global_path_topic=%s, pose_source=%s, odom_topic=%s, tf=%s->%s, dynamic_obstacle_topic=%s, cmd_vel_topic=%s",
                global_path_topic_.c_str(), pose_source_.c_str(), odom_topic_.c_str(),
                global_frame_.c_str(), robot_frame_.c_str(), obstacle_cloud_topic_.c_str(),
                cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "auto_start_on_path=%s, use_obstacle_cloud=%s, enable_manual_debug_inputs=%s",
                auto_start_on_path_ ? "true" : "false",
                use_obstacle_cloud_ ? "true" : "false",
                enable_manual_debug_inputs_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "dynamic_obstacle_range=%.2fm, max_dynamic_obstacles=%d",
                dynamic_obstacle_range_, max_dynamic_obstacles_);
}

void manager_Interface::loadParameters()
{
    global_path_topic_ = this->declare_parameter<std::string>("global_path_topic", global_path_topic_);
    pose_source_ = this->declare_parameter<std::string>("pose_source", pose_source_);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", odom_topic_);
    obstacle_cloud_topic_ = this->declare_parameter<std::string>(
        "dynamic_obstacle_cloud_topic",
        this->declare_parameter<std::string>("obstacle_cloud_topic", obstacle_cloud_topic_));
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
    global_frame_ = this->declare_parameter<std::string>("global_frame", global_frame_);
    robot_frame_ = this->declare_parameter<std::string>("robot_frame", robot_frame_);
    tf_lookup_timeout_ = this->declare_parameter<double>("tf_lookup_timeout", tf_lookup_timeout_);
    dynamic_obstacle_range_ = this->declare_parameter<double>("dynamic_obstacle_range", dynamic_obstacle_range_);
    collision_check_radius_ = this->declare_parameter<double>("collision_check_radius", collision_check_radius_);
    max_dynamic_obstacles_ = this->declare_parameter<int>("max_dynamic_obstacles", max_dynamic_obstacles_);
    auto_start_on_path_ = this->declare_parameter<bool>("auto_start_on_path", auto_start_on_path_);
    use_obstacle_cloud_ = this->declare_parameter<bool>("use_obstacle_cloud", use_obstacle_cloud_);
    enable_manual_debug_inputs_ = this->declare_parameter<bool>(
        "enable_manual_debug_inputs", enable_manual_debug_inputs_);
}

void manager_Interface::createRosInterfaces()
{
    // 可视化输出统一使用 global_frame_。本项目默认 FASTLIO 和 OctoPlanner 都在 map 系下工作，
    // 因此这里不额外做 TF 转换，避免在链路里混入隐式坐标变换。
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_global_path", 10);
    teb_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_teb_trajectory", 10);
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_obstacles", 10);
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 默认订阅 /optimized_path：它由 ego_planner/octo_path_optimizer_node 根据 OctoPlanner 的
    // /planned_path 生成。manager 不再二次调用 Ego，只把这条已优化路径裁剪给 TEB 跟踪。
    rviz_global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic_, 10,
        std::bind(&manager_Interface::rviz_global_path_callback, this, std::placeholders::_1));

    // FASTLIO 常见运行方式只广播 map->base_link TF，不一定发布 /lio_odom。
    // 默认 pose_source=tf：主循环每帧查 TF 并差分估计 v/w；如果配置为 odom 或 both，
    // 才额外订阅 odom_topic_。
    if ((pose_source_ == "odom" || pose_source_ == "both") && !odom_topic_.empty())
    {
        currPose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&manager_Interface::pose_callback, this, std::placeholders::_1));
    }

    if (use_obstacle_cloud_)
    {
        // Ego 的障碍层只接局部动态点云。OctoMap 静态层已经通过 /planned_path 体现，
        // 不应把全局 OctoMap 点云灌进这里。
        rviz_obstacles_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            obstacle_cloud_topic_, 10,
            std::bind(&manager_Interface::rviz_obstacles_callback, this, std::placeholders::_1));
    }

    trigger_plan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_plan", 10,
        std::bind(&manager_Interface::trigger_plan_callback, this, std::placeholders::_1));

    if (enable_manual_debug_inputs_)
    {
        // 调试模式下才启用这些 RViz 输入。默认关闭，避免 /goal_pose 被 OctoPlanner 用作目标点时
        // 又被本节点错误地当成障碍物。
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&manager_Interface::goal_pose_callback, this, std::placeholders::_1));
        rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&manager_Interface::rviz_point_callback, this, std::placeholders::_1));
        pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&manager_Interface::pose_estimate_callback, this, std::placeholders::_1));
    }
}

void manager_Interface::pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    std::lock_guard<std::mutex> lock(data_mutex_);
    // 可选 odom 位姿源：如果外部节点提供 nav_msgs/Odometry，可用它更新规划闭环状态。
    // 当前 Gazebo + FASTLIO2 默认走 updatePoseFromTf()，直接从 map->base_link TF 获取位姿。
    // x/y/yaw 用于把全局路径裁剪到机器人前方，v/w 用于 TEB 热启动和速度约束。
    cur_pose_.x = msg->pose.pose.position.x;         

    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.theta = tf2::getYaw(q);
    cur_pose_.v     = msg->twist.twist.linear.x;
    cur_pose_.w     = msg->twist.twist.angular.z;
    has_odom_ = true;
}

bool manager_Interface::updatePoseFromTf()
{
    geometry_msgs::msg::TransformStamped tf_msg;
    try
    {
        tf_msg = tf_buffer_->lookupTransform(
            global_frame_, robot_frame_, tf2::TimePointZero,
            tf2::durationFromSec(tf_lookup_timeout_));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for FASTLIO TF %s -> %s: %s",
                             global_frame_.c_str(), robot_frame_.c_str(), ex.what());
        return false;
    }

    PathPoint tf_pose;
    tf_pose.x = tf_msg.transform.translation.x;
    tf_pose.y = tf_msg.transform.translation.y;
    tf_pose.z = 0.0F;

    tf2::Quaternion q(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w);
    tf_pose.theta = tf2::getYaw(q);

    const rclcpp::Time stamp(tf_msg.header.stamp);
    tf_pose.v = 0.0F;
    tf_pose.w = 0.0F;

    if (has_last_tf_pose_)
    {
        const double dt = (stamp - last_tf_stamp_).seconds();
        if (dt > 1.0e-3)
        {
            const double dx = tf_pose.x - last_tf_pose_.x;
            const double dy = tf_pose.y - last_tf_pose_.y;
            const double heading_x = std::cos(tf_pose.theta);
            const double heading_y = std::sin(tf_pose.theta);
            tf_pose.v = static_cast<float>((dx * heading_x + dy * heading_y) / dt);

            double dtheta = tf_pose.theta - last_tf_pose_.theta;
            while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
            tf_pose.w = static_cast<float>(dtheta / dt);
        }
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cur_pose_ = tf_pose;
        last_tf_pose_ = tf_pose;
        last_tf_stamp_ = stamp;
        has_last_tf_pose_ = true;
        has_odom_ = true;
    }

    return true;
}

bool manager_Interface::transformObstacleToGlobal(
    const geometry_msgs::msg::TransformStamped& tf_msg,
    float src_x,
    float src_y,
    float src_z,
    ObstacleInfo& obs) const
{
    tf2::Quaternion q(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w);
    tf2::Matrix3x3 rot(q);

    obs.x = static_cast<float>(
        rot[0][0] * src_x + rot[0][1] * src_y + rot[0][2] * src_z + tf_msg.transform.translation.x);
    obs.y = static_cast<float>(
        rot[1][0] * src_x + rot[1][1] * src_y + rot[1][2] * src_z + tf_msg.transform.translation.y);
    obs.z = static_cast<float>(
        rot[2][0] * src_x + rot[2][1] * src_y + rot[2][2] * src_z + tf_msg.transform.translation.z);
    return std::isfinite(obs.x) && std::isfinite(obs.y) && std::isfinite(obs.z);
}

// 初始化 TEB 跟踪器。Ego B-spline 优化由 octo_path_optimizer_node 独立完成。
void manager_Interface::initPlannerBase()
{
    teb_planner_ = std::make_shared<irpc::planning::TebPlannerInterface>();
    teb_planner_->initialize();
}

// 统一添加障碍物函数
void manager_Interface::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    ObstacleInfo obs;
    obs.x = x;
    obs.y = y;
    obs.z = 0.0;
    
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    
    // 如果有障碍物更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "障碍物更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "添加障碍物: (%.2f, %.2f), 总障碍物数量: %zu", 
                x, y, obstacles_.size());
}

void manager_Interface::rviz_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 动态避障入口：
    // 1. OctoMap 静态地图层只用于 OctoPlanner 生成 /planned_path，不再直接输入 Ego。
    // 2. Ego 这里只接局部动态障碍点云，例如感知节点从当前激光/深度点云中分割出的行人、
    //    车辆、临时物体等。点云可以在 map、base_link 或传感器坐标系，只要 TF 可查到 map。
    // 3. 每一帧点云都会整体替换上一帧 obstacles_，因此动态物体离开视野后不会残留在局部栅格。
    // 4. updateRobotState() 会把 obstacles_ 投影到以机器人当前位置为中心的 2D 局部栅格，
    //    Ego B-spline 优化器和碰撞检测都只使用这个动态障碍层。
    std::vector<ObstacleInfo> parsed_obstacles;
    const std::size_t raw_count = static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    const std::size_t max_keep = static_cast<std::size_t>(std::max(0, max_dynamic_obstacles_));
    parsed_obstacles.reserve(max_keep > 0 ? std::min(raw_count, max_keep) : raw_count);

    PathPoint robot_pose;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_pose = cur_pose_;
    }

    geometry_msgs::msg::TransformStamped obstacle_to_global;
    const bool already_global = msg->header.frame_id.empty() || msg->header.frame_id == global_frame_;
    if (!already_global)
    {
        try
        {
            obstacle_to_global = tf_buffer_->lookupTransform(
                global_frame_, msg->header.frame_id, tf2::TimePointZero,
                tf2::durationFromSec(tf_lookup_timeout_));
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Cannot transform dynamic obstacle cloud %s -> %s: %s",
                                 msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
            return;
        }
    }

    try
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y))
            {
                continue;
            }

            ObstacleInfo obs;
            const float src_z = std::isfinite(*iter_z) ? *iter_z : 0.0F;
            if (already_global)
            {
                obs.x = *iter_x;
                obs.y = *iter_y;
                obs.z = src_z;
            }
            else if (!transformObstacleToGlobal(obstacle_to_global, *iter_x, *iter_y, src_z, obs))
            {
                continue;
            }

            const double dx = static_cast<double>(obs.x) - robot_pose.x;
            const double dy = static_cast<double>(obs.y) - robot_pose.y;
            if (dynamic_obstacle_range_ > 0.0 &&
                dx * dx + dy * dy > dynamic_obstacle_range_ * dynamic_obstacle_range_)
            {
                continue;
            }

            parsed_obstacles.push_back(obs);
            if (max_keep > 0 && parsed_obstacles.size() >= max_keep)
            {
                break;
            }
        }
    }
    catch (const std::runtime_error& e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to parse obstacle cloud '%s': %s",
                    obstacle_cloud_topic_.c_str(), e.what());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 动态障碍采用整帧替换，不累加历史点；消失的动态物体会在下一帧自然清除。
        obstacles_ = parsed_obstacles;
        has_obstacles_ = !obstacles_.empty();
        // 只要已经有 OctoPlanner 全局路径，新的动态障碍就触发一次 Ego 局部重优化。
        // 这样全局静态可通行性仍由 OctoPlanner/OctoMap 保证，局部瞬时绕行由 Ego 负责。
        needs_replan_ = has_valid_global_path_;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dynamic obstacle cloud %s: raw=%zu, kept=%zu, range=%.2fm, frame=%s",
                         obstacle_cloud_topic_.c_str(), raw_count, parsed_obstacles.size(),
                         dynamic_obstacle_range_, msg->header.frame_id.c_str());
}

// 2D Pose Estimate回调函数
void manager_Interface::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    #if 0
    std::lock_guard<std::mutex> lock(data_mutex_);
   // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;
    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);
    std::cout << "pose_estimate_callback " << std::endl;
    // 2. 触发规划逻辑（新增）
    if (has_valid_global_path_) {  // 确保已有全局路径
        needs_replan_ = true;  // 标记需要重新规划
        if (should_plan_) {
            RCLCPP_INFO(this->get_logger(), "初始位置更新，触发重新规划！");
        } else {
            RCLCPP_WARN(this->get_logger(), "初始位置已更新，但规划未启动！请先发送 /trigger_plan true 启动规划");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "初始位置已更新，但无有效全局路径，无法规划！");
    }
    #endif
}

// 处理2D Nav Goal - 生成从起点到目标的直线路
void manager_Interface::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    add_obstacle_at_position(msg->pose.position.x, msg->pose.position.y);
}

void manager_Interface::trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    should_plan_ = msg->data;

    if (should_plan_)
    {
        // 手动恢复规划时，如果已经缓存了 OctoPlanner 路径，立即触发一次 Ego 重规划。
        needs_replan_ = has_valid_global_path_;
        RCLCPP_INFO(this->get_logger(), "Planning resumed by /trigger_plan.");
    }
    else
    {
        needs_replan_ = false;
        teb_planned_traj_.clear();
        globalPlan_.clear();
        RCLCPP_WARN(this->get_logger(), "Planning paused by /trigger_plan; cmd_vel will be held at zero.");
    }
}

// 生成直线路径
void manager_Interface::generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                                                           const geometry_msgs::msg::PoseStamped& goal)
{
    global_plan_traj_.clear();

    // 计算路径点数量（每0.1米一个点）
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    int num_points = std::max(2, static_cast<int>(distance / 0.1));

    // 生成直线路径点
    for (int i = 0; i < num_points; ++i) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        PathPoint point;
        point.x = start.pose.position.x + ratio * dx;
        point.y = start.pose.position.y + ratio * dy;
        point.z = 0.0;
        global_plan_traj_.push_back(point);
    }
}

// // 处理Publish Point点击 - 添加障碍物
void manager_Interface::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 仅调试模式使用：手动 Publish Point 按顺序拼成一条参考路径。
    PathPoint point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = 0; // 使用z作为theta角度
    global_plan_traj_.push_back(point);
    std::cout << "从RViz接收到全局路径点: (" << point.x << ", " << point.y << ")" << std::endl;
    has_valid_global_path_ = true;
    should_plan_ = auto_start_on_path_;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    // if (should_plan_) 
    {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
}

void manager_Interface::rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty())
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        RCLCPP_WARN(this->get_logger(), "Received empty global path from %s", global_path_topic_.c_str());
        global_plan_traj_.clear();
        has_valid_global_path_ = false;
        teb_planned_traj_.clear();
        globalPlan_.clear();
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    global_plan_traj_.clear();
    global_plan_traj_.reserve(msg->poses.size());
    for (const auto& pose_stamped : msg->poses)
    {
        PathPoint path_point;
        path_point.x = pose_stamped.pose.position.x;
        path_point.y = pose_stamped.pose.position.y;
        path_point.z = pose_stamped.pose.position.z;
        global_plan_traj_.push_back(path_point);
    }

    has_valid_global_path_ = global_plan_traj_.size() >= 2;
    should_plan_ = auto_start_on_path_ || should_plan_;
    needs_replan_ = has_valid_global_path_ && should_plan_;
    teb_planned_traj_.clear();
    globalPlan_.clear();
    
    RCLCPP_INFO(this->get_logger(), "Received optimized tracking path from %s, points=%zu, auto_start=%s",
                global_path_topic_.c_str(), global_plan_traj_.size(),
                should_plan_ ? "true" : "false");
}

void manager_Interface::updateRobotState()
{
    std::lock_guard<std::mutex> lock(data_mutex_); 
    local_pose_.x = cur_pose_.x;
    local_pose_.y = cur_pose_.y;
    local_pose_.z = 0;
    local_pose_.theta = cur_pose_.theta;

    irpc::planning::vehicleStateInfo curPose;
    curPose.x = cur_pose_.x;
    curPose.y = cur_pose_.y;
    curPose.theta =  cur_pose_.theta;
    curPose.v = cur_pose_.v;
    curPose.w = cur_pose_.w;
    teb_planner_->setVehicleState(curPose);

    std::vector<irpc::planning::obstacleInfo> teb_obstacles;
    teb_obstacles.reserve(obstacles_.size());
    for (const auto& obs : obstacles_)
    {
        irpc::planning::obstacleInfo teb_obs;
        teb_obs.x = obs.x;
        teb_obs.y = obs.y;
        teb_obstacles.push_back(teb_obs);
    }
    // 动态点云不再进入 Ego 栅格，而是直接作为 TEB PointObstacle。
    // 这样 OctoMap 静态层仍由 OctoPlanner/octo_path_optimizer 处理，
    // TEB 负责跟踪时的短时域动态避障。
    teb_planner_->setObstacleInfo(teb_obstacles);

}

bool manager_Interface::collisionDetection(std::vector<PathPoint>& planned_traj)
{
    if (planned_traj.empty() || obstacles_.empty())
    {
        return false;
    }

    const double radius_sq = collision_check_radius_ * collision_check_radius_;
    for (const auto& pt : planned_traj)
    {
        for (const auto& obs : obstacles_)
        {
            const double dx = static_cast<double>(pt.x) - obs.x;
            const double dy = static_cast<double>(pt.y) - obs.y;
            if (dx * dx + dy * dy <= radius_sq)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Tracking path intersects dynamic obstacle within %.2fm.",
                                     collision_check_radius_);
                return true;
            }
        }
    }

    return false; 

}

void manager_Interface::useGlobalPath()
{
    std::vector<PathPoint> global_plan_traj_temp;

    discretize_trajectory(global_plan_traj_, global_plan_traj_temp, 0.1);
             
    float mindist = 100000000;
    int minddex = 0;
    for(int i = 0; i < global_plan_traj_temp.size();i++)
    {
        double dist =  distance(global_plan_traj_temp[i], local_pose_); 
        if(dist < mindist)
        {
            mindist = dist;
            minddex = i;
        } 
    }
    std::vector<PathPoint> global_plan_traj_after;
    global_plan_traj_after.push_back(local_pose_);
    for(int i = minddex; i < global_plan_traj_temp.size();i++)
    {
        global_plan_traj_after.push_back(global_plan_traj_temp[i]);
    }
    discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, 0.1);

    
    globalPlan_.clear();
    for(int i = 0 ; i < global_plan_traj_temp.size() - 1;i++)
    {
        irpc::planning::trajPointInfo pt;
        pt.x = global_plan_traj_temp[i].x;
        pt.y = global_plan_traj_temp[i].y;
          
        pt.theta = atan2(global_plan_traj_temp[i+1].y - global_plan_traj_temp[i].y ,  global_plan_traj_temp[i+1].x -  global_plan_traj_temp[i].x);
        globalPlan_.push_back(pt);
    }
    
    if(globalPlan_.size() < 2)
    {
        std::cout << "[useGlobalPath()]global plan size less 2，maybe reached" << std::endl;
        return;
    } 
    teb_planner_->setReferencePath(globalPlan_);
}

// 核心逻辑：检查数据更新→触发规划→发布结果
void manager_Interface::make_plan_and_control()
{
    if ((pose_source_ == "tf" || pose_source_ == "both") && !updatePoseFromTf())
    {
        publish_global_path();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_valid_global_path_ || global_plan_traj_.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for global path on %s", global_path_topic_.c_str());
            publish_global_path();
            return;
        }

        if (global_plan_traj_.size() < 2)
        {
            publishZeroCmd("global path has fewer than 2 points");
            publish_global_path();
            return;
        }

        if (!has_odom_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for robot pose from %s", pose_source_.c_str());
            publish_global_path();
            return;
        }

        if (!should_plan_)
        {
            publishZeroCmd("planning paused");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Global path received, planning is paused. Publish /trigger_plan true to resume.");
            publish_global_path();
            return;
        }
    }

    updateRobotState();


    // 不再进行 Ego 二次优化。每个周期都依据当前位姿从 /optimized_path 裁剪前方参考线，
    // 交给 TEB 做短时域跟踪；如果局部轨迹碰到动态障碍，TEB 会结合障碍物约束重新求解。
    if (needs_replan_)
    {
        needs_replan_ = false;
    }

    useGlobalPath();

    if(globalPlan_.size() < 2) 
    {
        publishZeroCmd("TEB reference path has fewer than 2 points");
        return;
    }

    teb_planned_traj_.clear();
        
    std::vector<Eigen::Vector3f> traj;
    const bool teb_ok = teb_planner_->plan(cmd_,traj);
    if (!teb_ok)
    {
        publishZeroCmd("TEB planner failed");
    }
        
    for(int i = 0; i < traj.size();i++)
    {
        Eigen::Vector3f pathP = traj[i];
        PathPoint point;
        point.x = pathP[0];
        point.y = pathP[1];
        point.z = 0;
        teb_planned_traj_.push_back(point);
    }

    if (teb_ok)
    {
        publishCmd();
    }
    if (collisionDetection(teb_planned_traj_))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "TEB trajectory is still close to a dynamic obstacle; cmd_vel has already been constrained by TEB.");
    }
        
    if (has_obstacles_) 
    {
        RCLCPP_INFO(this->get_logger(), "规划完成! 包含障碍物避让. 路径点: %zu, 障碍物: %zu", 
                       global_plan_traj_.size(), obstacles_.size());
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "规划完成! 无障碍物. 路径点: %zu", 
                       global_plan_traj_.size());
    }
        
    // 发布所有可视化数据（无论是否更新，保持实时显示）
    publish_global_path();
    publish_teb_planned_trajectory();
    publish_obstacles();
}

void manager_Interface::publishCmd()
{
    // 发布给 rl_sar Gazebo 控制器的最终速度指令。斜率限制用于保护四足仿真控制，
    // 避免 TEB 输出突变导致 RL 行走策略收到过大的瞬时速度阶跃。
    float smoothed_v = limit_slope(cmd_.vx, last_v_, max_v_step_);
    float smoothed_w = limit_slope(cmd_.w, last_w_, max_w_step_);
    // 2. 更新状态，供下一帧参考
    last_v_ = smoothed_v;
    last_w_ = smoothed_w;

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = smoothed_v;
    msg.linear.y = 0;
    msg.angular.z = smoothed_w;
    pub_cmd_->publish(msg);
    std::cout << "publishCmd : v =" << smoothed_v << " w =" << smoothed_w << std::endl;
}

void manager_Interface::publishZeroCmd(const std::string& reason)
{
    cmd_.vx = 0.0F;
    cmd_.vy = 0.0F;
    cmd_.w = 0.0F;
    last_v_ = 0.0F;
    last_w_ = 0.0F;

    auto msg = geometry_msgs::msg::Twist();
    pub_cmd_->publish(msg);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing zero cmd_vel: %s", reason.c_str());
}

float manager_Interface::limit_slope(float target, float current, float step) 
{
    float diff = target - current;

    if (diff > step) {
        // 目标值比当前值大太多 (加速/正向增大)，向上限制
        return current + step;
    } else if (diff < -step) {
        // 目标值比当前值小太多 (减速/反向增大)，向下限制
        return current - step;
    }

    // 变化量在 [-0.02, 0.02] 之间，直接取目标值
    return target;
}

// 发布可视化全局路径
void manager_Interface::publish_global_path()
{
    // if (global_plan_traj_.empty()) return;

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = global_frame_;

    for (const auto& path_point : global_plan_traj_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

void manager_Interface::publish_teb_planned_trajectory()
{
    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = global_frame_;

    for (size_t i = 0; i < teb_planned_traj_.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = teb_planned_traj_[i].x;
        pose.pose.position.y = teb_planned_traj_[i].y;
        pose.pose.position.z = teb_planned_traj_[i].z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    teb_traj_pub_->publish(visual_traj);
}

// 发布可视化障碍物
void manager_Interface::publish_obstacles()
{
    // if (obstacles_.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = global_frame_;
    visual_obs.width = obstacles_.size();
    visual_obs.height = 1;
    visual_obs.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles_)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_pub_->publish(visual_obs);
}

// 计算两点之间的欧氏距离（单位：米）
double manager_Interface::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
 */
void manager_Interface::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                              std::vector<PathPoint>& discrete_trajectory,
                                              double interval) {
    if (original_trajectory.size() < 2) {
        if(!original_trajectory.empty()) {
             discrete_trajectory.push_back(original_trajectory[0]);
        }
        return;
    }

    discrete_trajectory.clear();
    // 1. 加入起点
    discrete_trajectory.push_back(original_trajectory[0]);

    double accumulated_dist = 0.0;
    double target_dist = interval; // 下一个目标采样点的距离

    // 2. 遍历原始轨迹
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);

        // 如果线段太短，直接跳过，累积误差极小
        if (seg_length < 1e-6) continue;

        // 当前线段覆盖了多少距离
        double current_seg_end_dist = accumulated_dist + seg_length;

        // 在当前线段上寻找所有符合步长的点
        while (target_dist <= current_seg_end_dist) {
            // 计算插值比例
            // distance_into_segment 是目标点在当前线段上的局部距离
            double distance_into_segment = target_dist - accumulated_dist;
            double ratio = distance_into_segment / seg_length;

            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            // Z轴或其他属性也可以插值
            p.z = 0; 

            discrete_trajectory.push_back(p);

            // 准备找下一个点
            target_dist += interval;
        }

        // 更新累积距离
        accumulated_dist += seg_length;
    }

    // 3. 强制加入终点（防止因最后一段不足 interval 而丢失终点）
    // 只有当最后一个采样点距离终点较远时才加，避免重复
    if (distance(discrete_trajectory.back(), original_trajectory.back()) > 1e-3) {
        discrete_trajectory.push_back(original_trajectory.back());
    }
}

// 主函数：启动节点
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<manager_Interface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
