#include "planner_interface.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace
{

struct PathPoint3d
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

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

double distance2d(const PathPoint3d & a, const PathPoint3d & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double distance3d(const PathPoint3d & a, const PathPoint3d & b)
{
  return std::sqrt(
    (b.x - a.x) * (b.x - a.x) +
    (b.y - a.y) * (b.y - a.y) +
    (b.z - a.z) * (b.z - a.z));
}

double distanceSquared3d(const PathPoint3d & a, const PathPoint3d & b)
{
  return
    (b.x - a.x) * (b.x - a.x) +
    (b.y - a.y) * (b.y - a.y) +
    (b.z - a.z) * (b.z - a.z);
}

double distance2d(const ego_planner::PathPoint & a, const ego_planner::PathPoint & b)
{
  return std::hypot(static_cast<double>(b.x - a.x), static_cast<double>(b.y - a.y));
}

std::vector<double> cumulativeDistances3d(const std::vector<PathPoint3d> & path)
{
  std::vector<double> distances;
  distances.reserve(path.size());
  distances.push_back(0.0);

  for (std::size_t i = 1; i < path.size(); ++i) {
    distances.push_back(distances.back() + distance3d(path[i - 1], path[i]));
  }

  return distances;
}

std::vector<double> cumulativeDistances2d(const std::vector<ego_planner::PathPoint> & path)
{
  std::vector<double> distances;
  distances.reserve(path.size());
  distances.push_back(0.0);

  for (std::size_t i = 1; i < path.size(); ++i) {
    distances.push_back(distances.back() + distance2d(path[i - 1], path[i]));
  }

  return distances;
}

double interpolateOriginalZ(
  const std::vector<PathPoint3d> & original_path,
  const std::vector<double> & original_s,
  double target_s)
{
  if (original_path.empty()) {
    return 0.0;
  }
  if (original_path.size() == 1 || original_s.back() <= 1.0e-9) {
    return original_path.front().z;
  }
  if (target_s <= 0.0) {
    return original_path.front().z;
  }
  if (target_s >= original_s.back()) {
    return original_path.back().z;
  }

  const auto upper = std::upper_bound(original_s.begin(), original_s.end(), target_s);
  const std::size_t idx = static_cast<std::size_t>(std::distance(original_s.begin(), upper));
  const std::size_t prev = idx - 1;
  const double segment = std::max(1.0e-9, original_s[idx] - original_s[prev]);
  const double ratio = (target_s - original_s[prev]) / segment;
  return original_path[prev].z + ratio * (original_path[idx].z - original_path[prev].z);
}

std::vector<PathPoint3d> resamplePath2d(
  const std::vector<PathPoint3d> & path,
  double interval)
{
  if (path.size() < 2) {
    return path;
  }

  const double step = std::max(1.0e-3, interval);
  std::vector<PathPoint3d> result;
  result.reserve(path.size());
  result.push_back(path.front());

  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & start = path[i - 1];
    const auto & end = path[i];
    const double segment_length = distance2d(start, end);
    if (segment_length <= 1.0e-9) {
      continue;
    }

    const int inner_points = static_cast<int>(std::floor(segment_length / step));
    for (int j = 1; j <= inner_points; ++j) {
      const double ratio = std::min(1.0, (j * step) / segment_length);
      if (ratio >= 1.0) {
        break;
      }

      PathPoint3d point;
      point.x = start.x + ratio * (end.x - start.x);
      point.y = start.y + ratio * (end.y - start.y);
      point.z = start.z + ratio * (end.z - start.z);
      result.push_back(point);
    }

    result.push_back(end);
  }

  return result;
}

double estimateStartYaw(const std::vector<PathPoint3d> & path)
{
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path.front().x;
    const double dy = path[i].y - path.front().y;
    if (std::hypot(dx, dy) > 1.0e-6) {
      return std::atan2(dy, dx);
    }
  }

  return 0.0;
}

}  // namespace

class OctoPathOptimizerNode : public rclcpp::Node
{
public:
  OctoPathOptimizerNode()
  : Node("octo_path_optimizer_node")
  {
    input_path_topic_ = declare_parameter<std::string>("input_path_topic", "/planned_path");
    obstacle_cloud_topic_ =
      declare_parameter<std::string>("obstacle_cloud_topic", "/ego_obstacles");
    output_path_topic_ = declare_parameter<std::string>("output_path_topic", "/optimized_path");
    output_marker_topic_ =
      declare_parameter<std::string>("output_marker_topic", "/optimized_path_marker");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    path_resample_interval_ = declare_parameter<double>("path_resample_interval", 0.1);
    obstacle_z_min_ = declare_parameter<double>("obstacle_z_min", -0.2);
    obstacle_z_max_ = declare_parameter<double>("obstacle_z_max", 2.0);
    map_resolution_ = declare_parameter<double>("map_resolution", 0.1);
    map_x_size_ = declare_parameter<double>("map_x_size", 50.0);
    map_y_size_ = declare_parameter<double>("map_y_size", 50.0);
    map_z_size_ = declare_parameter<double>("map_z_size", 10.0);
    map_inflate_value_ = declare_parameter<double>("map_inflate_value", 1.0);
    max_vel_ = declare_parameter<double>("max_vel", 2.0);
    max_acc_ = declare_parameter<double>("max_acc", 3.0);
    max_jerk_ = declare_parameter<double>("max_jerk", 4.0);
    max_obstacle_points_ = declare_parameter<int>("max_obstacle_points", 5000);
    reoptimize_on_obstacle_update_ =
      declare_parameter<bool>("reoptimize_on_obstacle_update", true);
    min_obstacle_reoptimize_period_ =
      declare_parameter<double>("min_obstacle_reoptimize_period", 1.0);
    enable_geometric_smoothing_ =
      declare_parameter<bool>("enable_geometric_smoothing", true);
    smoothing_iterations_ =
      declare_parameter<int>("smoothing_iterations", 80);
    smoothing_data_weight_ =
      declare_parameter<double>("smoothing_data_weight", 0.18);
    smoothing_smooth_weight_ =
      declare_parameter<double>("smoothing_smooth_weight", 0.32);
    max_smoothing_deviation_ =
      declare_parameter<double>("max_smoothing_deviation", 0.8);
    smoothing_obstacle_clearance_ =
      declare_parameter<double>("smoothing_obstacle_clearance", map_inflate_value_);

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, latched_qos);
    optimized_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(output_marker_topic_, latched_qos);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_path_topic_,
      latched_qos,
      std::bind(&OctoPathOptimizerNode::onPath, this, std::placeholders::_1));

    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_cloud_topic_,
      latched_qos,
      std::bind(&OctoPathOptimizerNode::onObstacleCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Octo path optimizer ready. Subscribing path: %s, obstacles: %s, publishing: %s",
      input_path_topic_.c_str(),
      obstacle_cloud_topic_.c_str(),
      output_path_topic_.c_str());
  }

private:
  void onObstacleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<ego_planner::ObstacleInfo> obstacles;
    const std::size_t point_count =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    const std::size_t max_points =
      static_cast<std::size_t>(std::max(1, max_obstacle_points_));
    const std::size_t stride = point_count > max_points ?
      static_cast<std::size_t>(std::ceil(point_count / static_cast<double>(max_points))) :
      1U;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (std::size_t index = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index) {
        if (index % stride != 0U) {
          continue;
        }

        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z < obstacle_z_min_ || z > obstacle_z_max_) {
          continue;
        }

        ego_planner::ObstacleInfo obstacle;
        obstacle.x = x;
        obstacle.y = y;
        obstacle.z = z;
        obstacles.push_back(obstacle);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to read obstacle cloud xyz fields: %s", e.what());
      return;
    }

    const std::size_t cached_count = obstacles.size();
    {
      std::lock_guard<std::mutex> lock(obstacle_mutex_);
      latest_obstacles_ = std::move(obstacles);
      has_obstacle_cloud_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Cached %zu projected obstacle points from %zu cloud points.",
      cached_count,
      point_count);

    if (!reoptimize_on_obstacle_update_) {
      return;
    }

    std::vector<PathPoint3d> latest_path;
    std_msgs::msg::Header latest_header;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      if (!has_latest_path_) {
        return;
      }
      latest_path = latest_original_path_;
      latest_header = latest_path_header_;
    }

    const auto now_time = now();
    if (has_last_obstacle_reoptimize_) {
      const double elapsed = (now_time - last_obstacle_reoptimize_time_).seconds();
      if (elapsed < min_obstacle_reoptimize_period_) {
        return;
      }
    }

    last_obstacle_reoptimize_time_ = now_time;
    has_last_obstacle_reoptimize_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Obstacle cloud updated; re-optimizing latest path with %zu obstacle points.",
      cached_count);
    optimizeAndPublish(latest_path, latest_header);
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty input path. Publishing an empty optimized path.");
      publishPath({}, msg->header);
      publishMarker({}, msg->header);
      return;
    }

    const auto original_path = toPathPoints(*msg);
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      latest_original_path_ = original_path;
      latest_path_header_ = msg->header;
      has_latest_path_ = true;
    }

    optimizeAndPublish(original_path, msg->header);
  }

  void optimizeAndPublish(
    const std::vector<PathPoint3d> & original_path,
    const std_msgs::msg::Header & header)
  {
    if (original_path.size() < 2) {
      RCLCPP_WARN(get_logger(), "Input path has fewer than 2 poses. Skipping optimization.");
      publishPath(original_path, header);
      publishMarker(original_path, header);
      return;
    }

    std::vector<ego_planner::ObstacleInfo> obstacles;
    bool has_obstacle_cloud = false;
    {
      std::lock_guard<std::mutex> lock(obstacle_mutex_);
      obstacles = latest_obstacles_;
      has_obstacle_cloud = has_obstacle_cloud_;
    }

    if (!has_obstacle_cloud) {
      RCLCPP_WARN(
        get_logger(),
        "No obstacle cloud received yet. Optimizing as smoothing-only with an empty obstacle set.");
    }

    std::vector<PathPoint3d> optimized_path;
    const bool ego_ok = optimizePath(original_path, obstacles, optimized_path);
    if (!ego_ok) {
      RCLCPP_WARN(get_logger(), "Ego optimization failed. Publishing original path as fallback.");
      optimized_path = original_path;
    }

    if (enable_geometric_smoothing_) {
      const auto before_smoothing = optimized_path;
      optimized_path = smoothPath3d(optimized_path, obstacles);
      logPathDelta(before_smoothing, optimized_path, ego_ok);
    }

    publishPath(optimized_path, header);
    publishMarker(optimized_path, header);

    RCLCPP_INFO(
      get_logger(),
      "Published optimized path with %zu poses from input path with %zu poses.",
      optimized_path.size(),
      original_path.size());
  }

  std::vector<PathPoint3d> toPathPoints(const nav_msgs::msg::Path & msg) const
  {
    std::vector<PathPoint3d> path;
    path.reserve(msg.poses.size());
    for (const auto & pose_stamped : msg.poses) {
      PathPoint3d point;
      point.x = pose_stamped.pose.position.x;
      point.y = pose_stamped.pose.position.y;
      point.z = pose_stamped.pose.position.z;
      path.push_back(point);
    }
    return path;
  }

  bool optimizePath(
    const std::vector<PathPoint3d> & original_path,
    std::vector<ego_planner::ObstacleInfo> & obstacles,
    std::vector<PathPoint3d> & optimized_path)
  {
    const auto resampled_path = resamplePath2d(original_path, path_resample_interval_);
    if (resampled_path.size() < 2) {
      return false;
    }

    std::vector<ego_planner::PathPoint> ego_path;
    ego_path.reserve(resampled_path.size());
    for (const auto & point : resampled_path) {
      ego_planner::PathPoint ego_point;
      ego_point.x = static_cast<float>(point.x);
      ego_point.y = static_cast<float>(point.y);
      ego_point.z = 0.0F;
      ego_point.v = 0.0F;
      ego_path.push_back(ego_point);
    }

    ego_planner::PathPoint current_pose;
    const double start_yaw = estimateStartYaw(resampled_path);
    current_pose.x = static_cast<float>(resampled_path.front().x);
    current_pose.y = static_cast<float>(resampled_path.front().y);
    // PlannerInterface::makePlan() 使用 PathPoint::theta 生成起点速度/加速度方向。
    // 这里原来把 yaw 写进 z，单独启动 octo_path_optimizer_node 时 theta 未初始化，
    // 会导致初始导数方向随机，进而更容易出现 "Ego optimization failed"。
    current_pose.z = 0.0F;
    current_pose.theta = static_cast<float>(start_yaw);
    current_pose.v = 0.0F;
    current_pose.w = 0.0F;

    auto planner = std::make_shared<ego_planner::PlannerInterface>();
    planner->initParam(max_vel_, max_acc_, max_jerk_);
    planner->initEsdfMap(
      map_x_size_,
      map_y_size_,
      map_z_size_,
      map_resolution_,
      Eigen::Vector3d::Zero(),
      map_inflate_value_);
    planner->setGridMap(current_pose);
    planner->setCurrentVehiclePos(current_pose);
    planner->setObstacles(obstacles);
    planner->setPathPoint(ego_path);
    planner->makePlan();

    std::vector<ego_planner::PathPoint> ego_result;
    planner->getLocalPlanTrajResults(ego_result);
    if (ego_result.size() < 2) {
      return false;
    }

    optimized_path = restoreZ(original_path, ego_result);
    return optimized_path.size() >= 2;
  }

  bool candidateHitsObstacle(
    const PathPoint3d & point,
    const std::vector<ego_planner::ObstacleInfo> & obstacles) const
  {
    if (obstacles.empty() || smoothing_obstacle_clearance_ <= 0.0) {
      return false;
    }

    const double clearance_sq = smoothing_obstacle_clearance_ * smoothing_obstacle_clearance_;
    for (const auto & obstacle : obstacles) {
      const double dx = point.x - obstacle.x;
      const double dy = point.y - obstacle.y;
      const double dz = point.z - obstacle.z;
      if (dx * dx + dy * dy + dz * dz <= clearance_sq) {
        return true;
      }
    }

    return false;
  }

  PathPoint3d clampDeviation(const PathPoint3d & reference, const PathPoint3d & candidate) const
  {
    if (max_smoothing_deviation_ <= 0.0) {
      return reference;
    }

    const double dist = std::sqrt(distanceSquared3d(reference, candidate));
    if (dist <= max_smoothing_deviation_ || dist <= 1.0e-9) {
      return candidate;
    }

    const double ratio = max_smoothing_deviation_ / dist;
    PathPoint3d clamped;
    clamped.x = reference.x + ratio * (candidate.x - reference.x);
    clamped.y = reference.y + ratio * (candidate.y - reference.y);
    clamped.z = reference.z + ratio * (candidate.z - reference.z);
    return clamped;
  }

  std::vector<PathPoint3d> smoothPath3d(
    const std::vector<PathPoint3d> & input_path,
    const std::vector<ego_planner::ObstacleInfo> & obstacles) const
  {
    if (input_path.size() < 3 || smoothing_iterations_ <= 0) {
      return input_path;
    }

    std::vector<PathPoint3d> smoothed = input_path;
    const double data_weight = std::max(0.0, smoothing_data_weight_);
    const double smooth_weight = std::max(0.0, smoothing_smooth_weight_);

    for (int iter = 0; iter < smoothing_iterations_; ++iter) {
      std::vector<PathPoint3d> next = smoothed;

      for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
        // 3D elastic-band 平滑：
        // data 项把点拉回原始/避障优化后的参考路径，避免直接切穿 OctoPlanner 给出的可通行走廊；
        // smooth 项降低折线尖角，使 /optimized_path 和 /planned_path 在几何上产生可见差异。
        PathPoint3d candidate;
        candidate.x = smoothed[i].x +
          data_weight * (input_path[i].x - smoothed[i].x) +
          smooth_weight * (smoothed[i - 1].x + smoothed[i + 1].x - 2.0 * smoothed[i].x);
        candidate.y = smoothed[i].y +
          data_weight * (input_path[i].y - smoothed[i].y) +
          smooth_weight * (smoothed[i - 1].y + smoothed[i + 1].y - 2.0 * smoothed[i].y);
        candidate.z = smoothed[i].z +
          data_weight * (input_path[i].z - smoothed[i].z) +
          smooth_weight * (smoothed[i - 1].z + smoothed[i + 1].z - 2.0 * smoothed[i].z);

        candidate = clampDeviation(input_path[i], candidate);
        if (!candidateHitsObstacle(candidate, obstacles)) {
          next[i] = candidate;
        }
      }

      smoothed.swap(next);
    }

    smoothed.front() = input_path.front();
    smoothed.back() = input_path.back();
    return smoothed;
  }

  void logPathDelta(
    const std::vector<PathPoint3d> & before,
    const std::vector<PathPoint3d> & after,
    bool ego_ok) const
  {
    if (before.size() != after.size() || before.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Path smoothing result: ego_ok=%s, before_points=%zu, after_points=%zu",
        ego_ok ? "true" : "false",
        before.size(),
        after.size());
      return;
    }

    double sum_delta = 0.0;
    double max_delta = 0.0;
    for (std::size_t i = 0; i < before.size(); ++i) {
      const double delta = distance3d(before[i], after[i]);
      sum_delta += delta;
      max_delta = std::max(max_delta, delta);
    }

    RCLCPP_INFO(
      get_logger(),
      "Path smoothing result: ego_ok=%s, mean_delta=%.3fm, max_delta=%.3fm, points=%zu",
      ego_ok ? "true" : "false",
      sum_delta / static_cast<double>(before.size()),
      max_delta,
      after.size());
  }

  std::vector<PathPoint3d> restoreZ(
    const std::vector<PathPoint3d> & original_path,
    const std::vector<ego_planner::PathPoint> & optimized_xy) const
  {
    std::vector<PathPoint3d> output;
    output.reserve(optimized_xy.size());

    const auto original_s = cumulativeDistances3d(original_path);
    const auto optimized_s = cumulativeDistances2d(optimized_xy);
    const double original_total = original_s.empty() ? 0.0 : original_s.back();
    const double optimized_total = optimized_s.empty() ? 0.0 : optimized_s.back();

    for (std::size_t i = 0; i < optimized_xy.size(); ++i) {
      const double ratio =
        optimized_total > 1.0e-9 ? optimized_s[i] / optimized_total : 0.0;
      const double target_original_s = ratio * original_total;

      PathPoint3d point;
      point.x = optimized_xy[i].x;
      point.y = optimized_xy[i].y;
      point.z = interpolateOriginalZ(original_path, original_s, target_original_s);
      output.push_back(point);
    }

    if (!output.empty() && !original_path.empty()) {
      output.front().z = original_path.front().z;
      output.back().z = original_path.back().z;
    }

    return output;
  }

  void publishPath(
    const std::vector<PathPoint3d> & path,
    const std_msgs::msg::Header & input_header)
  {
    nav_msgs::msg::Path msg;
    msg.header = input_header;
    msg.header.stamp = now();
    if (msg.header.frame_id.empty()) {
      msg.header.frame_id = frame_id_;
    }
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(point.x, point.y, point.z);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    optimized_path_pub_->publish(msg);
  }

  void publishMarker(
    const std::vector<PathPoint3d> & path,
    const std_msgs::msg::Header & input_header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = input_header;
    marker.header.stamp = now();
    if (marker.header.frame_id.empty()) {
      marker.header.frame_id = frame_id_;
    }
    marker.ns = "optimized_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = path.empty() ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.16;
    marker.color = makeColor(0.0F, 0.95F, 0.65F, 1.0F);
    marker.points.reserve(path.size());

    for (const auto & point : path) {
      marker.points.push_back(makePoint(point.x, point.y, point.z));
    }

    optimized_marker_pub_->publish(marker);
  }

  std::string input_path_topic_;
  std::string obstacle_cloud_topic_;
  std::string output_path_topic_;
  std::string output_marker_topic_;
  std::string frame_id_;

  double path_resample_interval_ = 0.1;
  double obstacle_z_min_ = -0.2;
  double obstacle_z_max_ = 2.0;
  double map_resolution_ = 0.1;
  double map_x_size_ = 50.0;
  double map_y_size_ = 50.0;
  double map_z_size_ = 10.0;
  double map_inflate_value_ = 1.0;
  double max_vel_ = 2.0;
  double max_acc_ = 3.0;
  double max_jerk_ = 4.0;
  int max_obstacle_points_ = 5000;
  bool reoptimize_on_obstacle_update_ = true;
  double min_obstacle_reoptimize_period_ = 1.0;
  bool enable_geometric_smoothing_ = true;
  int smoothing_iterations_ = 80;
  double smoothing_data_weight_ = 0.18;
  double smoothing_smooth_weight_ = 0.32;
  double max_smoothing_deviation_ = 0.8;
  double smoothing_obstacle_clearance_ = 1.0;

  std::mutex obstacle_mutex_;
  bool has_obstacle_cloud_ = false;
  std::vector<ego_planner::ObstacleInfo> latest_obstacles_;

  std::mutex path_mutex_;
  bool has_latest_path_ = false;
  bool has_last_obstacle_reoptimize_ = false;
  rclcpp::Time last_obstacle_reoptimize_time_;
  std_msgs::msg::Header latest_path_header_;
  std::vector<PathPoint3d> latest_original_path_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPathOptimizerNode>());
  rclcpp::shutdown();
  return 0;
}
