// Copyright 2026 Luis Alberto Munoz Marin
// Licensed under the Apache License, Version 2.0
// Experimental visualization of RAFT's visible moving points.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "thesis_perception/stationary_voxels.hpp"

class RaftSweep : public rclcpp::Node
{
public:
  RaftSweep() : Node("raft_sweep")
  {
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/thesis/perception/raft_swept_surface_cpp", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/thesis/perception/raft_sweep_input", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {publish(*cloud);});
  }
private:
  void publish(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = cloud.header;
    marker.ns = "raft_swept_surface_cpp";
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.025;
    marker.color.g = 0.85f; marker.color.b = 1.0f; marker.color.a = 0.32f;
    marker.lifetime.nanosec = 600000000;
    const std::array<const char *, 6> names{{"x", "y", "z", "vx", "vy", "vz"}};
    bool valid = cloud.height == 1 && !cloud.is_bigendian && cloud.point_step == 24 &&
      cloud.fields.size() == names.size() && cloud.row_step == cloud.width * 24ULL &&
      cloud.data.size() == cloud.row_step;
    for (std::size_t i = 0; valid && i < names.size(); ++i) {
      const auto & field = cloud.fields[i];
      valid = field.name == names[i] && field.offset == i * 4 && field.count == 1 &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    }
    if (!valid || cloud.width == 0) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      publisher_->publish(marker);
      return;
    }
    std::unordered_set<thesis_perception::VoxelKey, thesis_perception::VoxelHash> cells;
    cells.reserve(cloud.width * 3ULL);
    for (std::size_t i = 0; i < cloud.width; ++i) {
      std::array<float, 6> p;
      std::memcpy(p.data(), cloud.data.data() + i * 24, 24);
      bool finite = true;
      for (float v : p) {finite = finite && std::isfinite(v) && std::abs(v) < 1000.0f;}
      if (!finite) {continue;}
      for (int step = 0; step < 11; ++step) {
        // Float arithmetic mirrors the NumPy float32 reference implementation.
        const float t = static_cast<float>(step * 0.05);
        const auto key = thesis_perception::VoxelKey{
          static_cast<int>(std::floor((p[0] + p[3] * t) / 0.025f)),
          static_cast<int>(std::floor((p[1] + p[4] * t) / 0.025f)),
          static_cast<int>(std::floor((p[2] + p[5] * t) / 0.025f))};
        cells.insert(key);
      }
    }
    marker.action = cells.empty() ? visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.points.reserve(cells.size());
    for (const auto & cell : cells) {
      geometry_msgs::msg::Point point;
      point.x = (static_cast<double>(cell.x) + 0.5) * 0.025;
      point.y = (static_cast<double>(cell.y) + 0.5) * 0.025;
      point.z = (static_cast<double>(cell.z) + 0.5) * 0.025;
      marker.points.push_back(point);
    }
    publisher_->publish(marker);
  }
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaftSweep>());
  rclcpp::shutdown();
}
