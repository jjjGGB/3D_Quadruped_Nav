#include "utils.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <sensor_msgs/msg/point_field.hpp>
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::cloud2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    const int point_num = static_cast<int>(msg->width * msg->height);
    cloud->reserve(point_num / filter_num + 1);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    bool has_timestamp = false;
    std::string timestamp_field_name;
    sensor_msgs::msg::PointField timestamp_field;
    for (const auto &field : msg->fields)
    {
        if (field.name == "timestamp" || field.name == "time")
        {
            has_timestamp = true;
            timestamp_field_name = field.name;
            timestamp_field = field;
            break;
        }
    }
    std::vector<double> point_timestamps;
    if (has_timestamp)
    {
        point_timestamps.reserve(point_num);
        switch (timestamp_field.datatype)
        {
        case sensor_msgs::msg::PointField::FLOAT64:
        {
            sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp(*msg, timestamp_field_name);
            for (int i = 0; i < point_num; ++i, ++iter_timestamp)
                point_timestamps.push_back(*iter_timestamp);
            break;
        }
        case sensor_msgs::msg::PointField::FLOAT32:
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_timestamp(*msg, timestamp_field_name);
            for (int i = 0; i < point_num; ++i, ++iter_timestamp)
                point_timestamps.push_back(static_cast<double>(*iter_timestamp));
            break;
        }
        case sensor_msgs::msg::PointField::UINT32:
        {
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_timestamp(*msg, timestamp_field_name);
            for (int i = 0; i < point_num; ++i, ++iter_timestamp)
                point_timestamps.push_back(static_cast<double>(*iter_timestamp));
            break;
        }
        default:
            has_timestamp = false;
            break;
        }
    }
    double min_ts = std::numeric_limits<double>::max();
    double max_ts = std::numeric_limits<double>::lowest();
    if (has_timestamp && !point_timestamps.empty())
    {
        for (auto ts : point_timestamps)
        {
            min_ts = std::min(min_ts, ts);
            max_ts = std::max(max_ts, ts);
        }
    }
    const double ts_delta = max_ts - min_ts;
    double to_millisecond = 1000.0; // timestamp in second
    if (ts_delta > 1e7)
        to_millisecond = 1e-6; // timestamp in nanosecond
    else if (ts_delta > 1e4)
        to_millisecond = 1e-3; // timestamp in microsecond
    else if (ts_delta > 10.0)
        to_millisecond = 1.0; // timestamp in millisecond
    const bool has_valid_relative_time = has_timestamp && ts_delta > 1e-9;
    int idx = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++idx)
    {
        if (idx % filter_num != 0)
            continue;
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
            continue;
        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = *iter_intensity;
        if (has_valid_relative_time && idx < static_cast<int>(point_timestamps.size()) && min_ts != std::numeric_limits<double>::max())
            p.curvature = static_cast<float>((point_timestamps[idx] - min_ts) * to_millisecond);
        else if (has_timestamp)
            p.curvature = 0.0f;
        else
            p.curvature = point_num > 1 ? static_cast<float>(idx) * 100.0f / static_cast<float>(point_num - 1) : 0.0f;
        cloud->push_back(p);
    }

    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
