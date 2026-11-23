// Copyright 2026 Luis Alberto Munoz Marin
// Licensed under the Apache License, Version 2.0
// Experimental visualization of point-wise RAFT trajectories.
#include <array>
#include <cmath>
#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

class RaftLines : public rclcpp::Node
{
public:
  RaftLines() : Node("raft_lines_experimental")
  {
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/thesis/perception/raft_swept_surface_experimental", qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/thesis/perception/raft_lines_input_experimental", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {publish(*cloud);});
  }

private:
  void publish(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = cloud.header;
    marker.ns = "raft_swept_surface_experimental";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.001;
    marker.color.g = 0.85f; marker.color.b = 1.0f; marker.color.a = 0.05f;
    marker.lifetime.nanosec = 600000000;
    const std::array<const char *, 6> names{{"x", "y", "z", "end_x", "end_y", "end_z"}};
    bool valid = cloud.height == 1 && !cloud.is_bigendian && cloud.point_step == 24 &&
      cloud.fields.size() == names.size() && cloud.row_step == cloud.width * 24ULL &&
      cloud.data.size() == cloud.row_step;
    for (std::size_t i = 0; valid && i < names.size(); ++i) {
      const auto & field = cloud.fields[i];
      valid = field.name == names[i] && field.offset == i * 4 && field.count == 1 &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    }
    const double age = (now() - rclcpp::Time(cloud.header.stamp)).seconds();
    if (valid && cloud.width > 0 && age >= -0.1 && age < 0.5) {
      marker.points.reserve(static_cast<std::size_t>(cloud.width) * 2);
      for (std::size_t i = 0; i < cloud.width; ++i) {
        std::array<float, 6> xyz;
        std::memcpy(xyz.data(), cloud.data.data() + i * 24, 24);
        bool finite = true;
        for (float value : xyz) {finite = finite && std::isfinite(value);}
        if (!finite) {continue;}
        geometry_msgs::msg::Point start, end;
        start.x = xyz[0]; start.y = xyz[1]; start.z = xyz[2];
        end.x = xyz[3]; end.y = xyz[4]; end.z = xyz[5];
        marker.points.push_back(start);
        marker.points.push_back(end);
      }
      if (!marker.points.empty()) {marker.action = visualization_msgs::msg::Marker::ADD;}
    }
    publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaftLines>());
  rclcpp::shutdown();
}
