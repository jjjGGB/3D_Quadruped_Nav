#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// localizer 的 ICP 只使用点的几何坐标，不使用 intensity。
// FASTLIO2 发布的 /fastlio2/body_cloud 是 PointXYZINormal，很多离线 PCD 地图也只有 xyz；
// 如果这里使用 PointXYZI，PCL 会在缺少 intensity 字段时打印
// "Failed to find match for field 'intensity'"。使用 PointXYZ 可以同时兼容
// xyz、xyzi、xyzinormal 等常见点云输入，避免字段不一致影响重定位调试。
using PointType = pcl::PointXYZ;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M4F = Eigen::Matrix4f;
using V4F = Eigen::Vector4f;
