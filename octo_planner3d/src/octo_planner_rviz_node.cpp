#include "global_planner.h"
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

constexpr double kZeroZThreshold = 1.0e-6;

std::string defaultInputPcd()
{
#ifdef OCTO_PLANNER3D_SOURCE_DIR
  return std::string(OCTO_PLANNER3D_SOURCE_DIR) + "/octomap/pcd_files/building2_9big.pcd";
#else
  return "../octomap/pcd_files/building2_9big.pcd";
#endif
}

std::string resolveInputPcd(const std::string & input_pcd)
{
  const std::filesystem::path pcd_path(input_pcd);
  if (pcd_path.is_absolute()) {
    return pcd_path.string();
  }

#ifdef OCTO_PLANNER3D_SOURCE_DIR
  return (std::filesystem::path(OCTO_PLANNER3D_SOURCE_DIR) / "octomap" / "pcd_files" / pcd_path)
    .string();
#else
  return (std::filesystem::path("../octomap/pcd_files") / pcd_path).string();
#endif
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

global_planner::PointPose toPlannerPoint(
  const geometry_msgs::msg::Pose & pose,
  double fallback_z)
{
  global_planner::PointPose point;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = std::abs(pose.position.z) > kZeroZThreshold ? pose.position.z : fallback_z;
  return point;
}

global_planner::PointPose toPlannerPoint(const geometry_msgs::msg::Point & point_msg)
{
  global_planner::PointPose point;
  point.x = point_msg.x;
  point.y = point_msg.y;
  point.z = point_msg.z;
  return point;
}

std::vector<std::string> splitCsv(const std::string & input)
{
  std::vector<std::string> result;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char c) {
      return !std::isspace(c);
    }));
    item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char c) {
      return !std::isspace(c);
    }).base(), item.end());
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  return result;
}

}  // namespace

class OctoPlannerRvizNode : public rclcpp::Node
{
public:
  OctoPlannerRvizNode()
  : Node("octo_planner_rviz_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    start_z_ = declare_parameter<double>("start_z", 0.3);
    goal_z_ = declare_parameter<double>("goal_z", 0.3);
    map_alpha_ = declare_parameter<double>("map_alpha", 0.82);
    map_color_mode_ = declare_parameter<std::string>("map_color_mode", "height");
    use_robot_pose_as_start_ = declare_parameter<bool>("use_robot_pose_as_start", true);
    robot_pose_lookup_timeout_ = declare_parameter<double>("robot_pose_lookup_timeout", 0.1);
    base_frame_candidates_ = splitCsv(
      declare_parameter<std::string>(
        "base_frame_candidates", "base_link,base_footprint,odin1_base_link"));
    if (base_frame_candidates_.empty()) {
      base_frame_candidates_.push_back("base_link");
    }
    const std::string clicked_point_topic =
      declare_parameter<std::string>("clicked_point_topic", "clicked_point");
    ego_obstacles_topic_ = declare_parameter<std::string>("ego_obstacles_topic", "/ego_obstacles");
    publish_ego_obstacles_ = declare_parameter<bool>("publish_ego_obstacles", true);
    ego_obstacles_max_points_ = declare_parameter<int>("ego_obstacles_max_points", 200000);
    const std::string input_pcd =
      resolveInputPcd(declare_parameter<std::string>("input_pcd", defaultInputPcd()));
    const std::string output_bt = declare_parameter<std::string>("output_bt", "result_cleaned.bt");
    const double map_publish_period =
      declare_parameter<double>("map_publish_period", 2.0);

    converter_ = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
    converter_->setInputPcdFile(input_pcd);
    converter_->setOutputBtFile(output_bt);
    planner_ = std::make_shared<global_planner::GlobalPlanner>();
    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    RCLCPP_INFO(get_logger(), "Building OctoMap from PCD file: %s", input_pcd.c_str());
    if (!converter_->convert()) {
      RCLCPP_ERROR(get_logger(), "Failed to build OctoMap. Node will stay alive for diagnostics.");
      return;
    }

    octree_ = converter_->getOctomap();
    planner_->setOctomap(octree_);

    const auto transient_qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("occupied_map", transient_qos);
    ego_obstacles_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(ego_obstacles_topic_, transient_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", transient_qos);
    path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", transient_qos);
    start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("start_marker", transient_qos);
    goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("goal_marker", transient_qos);

    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onStartPose, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onGoalPose, this, std::placeholders::_1));
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onClickedPoint, this, std::placeholders::_1));

    publishMap();
    map_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, map_publish_period))),
      std::bind(&OctoPlannerRvizNode::publishMap, this));

    RCLCPP_INFO(
      get_logger(),
      "Ready. Use goal_pose/clicked goal to plan. use_robot_pose_as_start=%s, clicked start is fallback. ego_obstacles=%s",
      use_robot_pose_as_start_ ? "true" : "false",
      publish_ego_obstacles_ ? ego_obstacles_topic_.c_str() : "disabled");
  }

private:
  void onStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_ = toPlannerPoint(msg->pose.pose, start_z_);
    has_start_ = true;
    publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Start set to [%.3f, %.3f, %.3f]",
      start_.x,
      start_.y,
      start_.z);
    planIfReady();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = toPlannerPoint(msg->pose, goal_z_);
    has_goal_ = true;
    publishPoseMarker(goal_, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Goal set to [%.3f, %.3f, %.3f]",
      goal_.x,
      goal_.y,
      goal_.z);
    planIfReady(true);
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (next_clicked_point_is_start_ && !use_robot_pose_as_start_) {
      start_ = toPlannerPoint(msg->point);
      has_start_ = true;
      has_goal_ = false;
      next_clicked_point_is_start_ = false;
      publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
      RCLCPP_INFO(
        get_logger(),
        "Start point set to [%.3f, %.3f, %.3f]. Publish the next point as goal.",
        start_.x,
        start_.y,
        start_.z);
      return;
    }

    goal_ = toPlannerPoint(msg->point);
    has_goal_ = true;
    next_clicked_point_is_start_ = !use_robot_pose_as_start_;
    publishPoseMarker(goal_, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Goal point set to [%.3f, %.3f, %.3f]. Planning with robot pose start when available.",
      goal_.x,
      goal_.y,
      goal_.z);
    planIfReady(true);
  }

  void planIfReady(bool refresh_robot_start = false)
  {
    if (!planner_ || !octree_ || !has_goal_) {
      return;
    }
    if (refresh_robot_start && use_robot_pose_as_start_) {
      if (updateStartFromRobotPose()) {
        RCLCPP_INFO(
          get_logger(),
          "Using robot current pose as start [%.3f, %.3f, %.3f]",
          start_.x,
          start_.y,
          start_.z);
      } else if (!has_start_) {
        RCLCPP_WARN(
          get_logger(),
          "Robot pose TF is unavailable and no manual fallback start exists. Skip planning.");
        return;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Robot pose TF is unavailable. Using manual fallback start [%.3f, %.3f, %.3f].",
          start_.x,
          start_.y,
          start_.z);
      }
    }
    if (!has_start_) {
      return;
    }

    planner_->makePlan(start_, goal_);

    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "Planner returned an empty path.");
      publishPath(path);
      return;
    }

    publishPath(path);
    RCLCPP_INFO(get_logger(), "Published planned path with %zu poses.", path.size());
  }

  bool updateStartFromRobotPose()
  {
    std::string last_error;
    for (const auto & base_frame : base_frame_candidates_) {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          frame_id_, base_frame, tf2::TimePointZero,
          tf2::durationFromSec(robot_pose_lookup_timeout_));
        start_.x = tf.transform.translation.x;
        start_.y = tf.transform.translation.y;
        start_.z = std::abs(tf.transform.translation.z) > kZeroZThreshold ?
          tf.transform.translation.z :
          start_z_;
        has_start_ = true;
        publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
        return true;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Could not lookup robot pose in %s. Last TF error: %s",
      frame_id_.c_str(), last_error.c_str());
    return false;
  }

  void publishMap()
  {
    if (!octree_ || !map_pub_) {
      return;
    }

    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const double z_range = std::max(1.0e-6, max_z - min_z);
    const float alpha = static_cast<float>(std::clamp(map_alpha_, 0.05, 1.0));

    std::unordered_map<double, visualization_msgs::msg::Marker> markers_by_size;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }

      const double size = it.getSize();
      auto marker_it = markers_by_size.find(size);
      if (marker_it == markers_by_size.end()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "occupied_voxels";
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        // 全部体素统一暗色
        // marker.color = makeColor(0.35F, 0.18F, 0.06F, alpha);
        marker.color = makeColor(0.45F, 0.22F, 0.06F, alpha);
        marker_it = markers_by_size.emplace(size, std::move(marker)).first;
      }

      marker_it->second.points.push_back(makePoint(it.getX(), it.getY(), it.getZ()));
    }

    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (auto & entry : markers_by_size) {
      auto & marker = entry.second;
      marker.header.stamp = now();
      marker.id = id++;
      array.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker cleanup;
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "occupied_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.insert(array.markers.begin(), cleanup);

    map_pub_->publish(array);
    publishEgoObstacles();
  }

  void publishEgoObstacles()
  {
    if (!publish_ego_obstacles_ || !octree_ || !ego_obstacles_pub_) {
      return;
    }

    std::vector<geometry_msgs::msg::Point> occupied_points;
    const std::size_t max_points =
      static_cast<std::size_t>(std::max(0, ego_obstacles_max_points_));

    occupied_points.reserve(max_points > 0U ? std::min<std::size_t>(max_points, 200000U) : 200000U);
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }
      occupied_points.push_back(makePoint(it.getX(), it.getY(), it.getZ()));
    }

    std::size_t output_count = occupied_points.size();
    std::size_t stride = 1U;
    if (max_points > 0U && occupied_points.size() > max_points) {
      output_count = max_points;
      stride = static_cast<std::size_t>(
        std::ceil(occupied_points.size() / static_cast<double>(max_points)));
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = now();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(output_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    std::size_t written = 0U;
    for (std::size_t i = 0; i < occupied_points.size() && written < output_count; i += stride) {
      *iter_x = static_cast<float>(occupied_points[i].x);
      *iter_y = static_cast<float>(occupied_points[i].y);
      *iter_z = static_cast<float>(occupied_points[i].z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++written;
    }

    ego_obstacles_pub_->publish(cloud);
  }

  std_msgs::msg::ColorRGBA heightColor(double t, float alpha) const
  {
    if (t < 0.33) {
      const float k = static_cast<float>(t / 0.33);
      return makeColor(0.10F, 0.45F + 0.35F * k, 0.95F - 0.25F * k, alpha);
    }
    if (t < 0.66) {
      const float k = static_cast<float>((t - 0.33) / 0.33);
      return makeColor(0.10F + 0.85F * k, 0.80F + 0.10F * k, 0.70F - 0.55F * k, alpha);
    }
    const float k = static_cast<float>((t - 0.66) / 0.34);
    return makeColor(0.95F, 0.90F - 0.45F * k, 0.15F + 0.05F * k, alpha);
  }

  void publishPath(const std::vector<global_planner::PointPose> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(point.x, point.y, point.z);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
    publishPathMarker(path);
  }

  void publishPathMarker(const std::vector<global_planner::PointPose> & path)
  {
    if (!path_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "planned_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = path.empty() ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    // 线宽：图里那种比较粗的紫色路径
    marker.scale.x = 0.18;   // 原来是 0.1，可再调成 0.15~0.25

    // 深紫色，接近你图里的颜色
    marker.color = makeColor(0.32F, 0.16F, 0.62F, 1.0F);

    marker.points.reserve(path.size());

    for (const auto & point : path) {
      marker.points.push_back(makePoint(point.x, point.y, point.z));
    }

    path_marker_pub_->publish(marker);
  }

  void publishPoseMarker(
    const global_planner::PointPose & pose,
    const std::string & ns,
    int id,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
  {
    if (!publisher) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(pose.x, pose.y, pose.z);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color = color;
    publisher->publish(marker);
  }

  std::string frame_id_;
  double start_z_ = 0.3;
  double goal_z_ = 0.3;
  double map_alpha_ = 0.82;
  std::string map_color_mode_ = "height";
  std::string ego_obstacles_topic_ = "/ego_obstacles";
  bool use_robot_pose_as_start_ = true;
  bool publish_ego_obstacles_ = true;
  double robot_pose_lookup_timeout_ = 0.1;
  int ego_obstacles_max_points_ = 200000;
  std::vector<std::string> base_frame_candidates_;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool next_clicked_point_is_start_ = true;

  global_planner::PointPose start_;
  global_planner::PointPose goal_;
  std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ego_obstacles_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlannerRvizNode>());
  rclcpp::shutdown();
  return 0;
}
