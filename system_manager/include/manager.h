/**
 * @file      src/PNC/manager/include/manager.h
 * @brief     manager Interface
 * @author    juchunyu@qq.com
 * @date      2026-01-02  12:00:01
 * @copyright Copyright (c) 2025, Institute of Robotics Planning and Control (irpc). All rights reserved.
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/path.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <teb_local_planner_interface.h>

struct PathPoint
{
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
        float v = 0.0F;
        float w = 0.0F;
        float theta = 0.0F;
};

struct ObstacleInfo
{
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
};


class manager_Interface : public rclcpp::Node
{
public:
    manager_Interface();
    ~manager_Interface() = default;

private:
    void loadParameters();
    void createRosInterfaces();
    void initPlannerBase();
    void make_plan_and_control();
    void updateRobotState();
    bool updatePoseFromTf();
    bool transformObstacleToGlobal(
        const geometry_msgs::msg::TransformStamped& tf_msg,
        float src_x,
        float src_y,
        float src_z,
        ObstacleInfo& obs) const;
    bool collisionDetection(std::vector<PathPoint>& planned_traj);
    void useGlobalPath();
    
    // 回调函数
    void rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rviz_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishCmd();
    void publishZeroCmd(const std::string& reason);


    
    // 发布函数
    void publish_global_path();
    void publish_teb_planned_trajectory();
    void publish_obstacles();


    // 辅助函数
    void generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                               const geometry_msgs::msg::PoseStamped& goal);
    void add_obstacle_at_position(double x, double y);

    void discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                           std::vector<PathPoint>& discrete_trajectory,
                           double interval = 0.1);

    double distance(const PathPoint& p1, const PathPoint& p2);

    float limit_slope(float target, float current, float step);

    // ROS 2发布者/订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr teb_traj_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rviz_global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_obstacles_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_estimate_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr currPose_subscriber_;

    
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // TEB 跟踪控制器。Ego 全局/局部 B-spline 优化已经移动到 ego_planner/octo_path_optimizer_node，
    // manager 只消费 /optimized_path，不再持有 ego_planner::PlannerInterface。
    std::shared_ptr<irpc::planning::TebPlannerInterface> teb_planner_; 

    // 数据存储
    std::vector<PathPoint> global_plan_traj_;
    std::vector<ObstacleInfo> obstacles_;
    std::mutex data_mutex_;
    bool has_valid_global_path_;
    bool has_obstacles_;
    bool should_plan_;
    bool needs_replan_;  // 新增：是否需要重新规划的标志
    bool has_odom_;
    bool has_last_tf_pose_;
    bool flag_ = false;
    std::vector<PathPoint> teb_planned_traj_;
    std::vector<irpc::planning::trajPointInfo> globalPlan_;


    PathPoint  cur_pose_;
    PathPoint last_tf_pose_;
    irpc::planning::cmdInfo cmd_;
    PathPoint local_pose_;
    rclcpp::Time last_tf_stamp_;
    float last_v_;
    float last_w_;
 


    // 参数
    double max_vel_ = 1.0;
    double max_acc_ = 1.0;
    double max_jerk_ = 1.0;
    double map_resolution_ = 0.1;
    double map_x_size_ = 60.0;
    double map_y_size_ = 60.0;
    double map_z_size_ = 2.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    double map_inflate_value_ = 0.5;
    double theta_ = 0.0;
    float max_v_step_ = 0.02;
    float max_w_step_ = 0.02;

    // ROS接口参数。默认值对齐本工作空间中的 OctoPlanner、FASTLIO2 和 rl_sar Gazebo 仿真链路。
    std::string global_path_topic_ = "/optimized_path";
    std::string pose_source_ = "tf";
    std::string odom_topic_ = "";
    std::string obstacle_cloud_topic_ = "/dynamic_obstacles";
    std::string cmd_vel_topic_ = "/cmd_vel";
    std::string global_frame_ = "map";
    std::string robot_frame_ = "base_link";
    double tf_lookup_timeout_ = 0.02;
    double dynamic_obstacle_range_ = 8.0;
    double collision_check_radius_ = 0.6;
    int max_dynamic_obstacles_ = 5000;
    bool auto_start_on_path_ = true;
    bool use_obstacle_cloud_ = true;
    bool enable_manual_debug_inputs_ = false;

};
