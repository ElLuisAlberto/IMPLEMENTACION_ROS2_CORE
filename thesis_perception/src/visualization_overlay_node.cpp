// Copyright 2026 Luis Alberto Munoz Marin
// Licensed under the Apache License, Version 2.0.

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class Overlay final : public rclcpp::Node
{
public:
  Overlay()
  : Node("visualization_overlay")
  {
    const auto transform = declare_parameter<std::vector<double>>(
      "visualization_transform", std::vector<double>{});
    if (transform.size() != transform_.size()) {
      throw std::runtime_error("visualization_transform requires 12 values");
    }
    for (std::size_t i = 0; i < transform_.size(); ++i) {
      if (!std::isfinite(transform[i])) {
        throw std::runtime_error("nonfinite visualization transform");
      }
      transform_[i] = transform[i];
    }
    RCLCPP_WARN(get_logger(), "Experimental visualization only; transform is not a calibration");
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/thesis/visualization/points_experimental", qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/thesis/perception/points_filtered", qos,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {process(std::move(msg));});
  }

private:
  void process(sensor_msgs::msg::PointCloud2::UniquePtr cloud)
  {
    if (cloud->header.frame_id != "d435i_depth_optical_frame" || cloud->is_bigendian ||
      cloud->point_step == 0)
    {
      return;
    }
    std::array<int, 3> offsets{{-1, -1, -1}};
    for (const auto & field : cloud->fields) {
      for (int i = 0; i < 3; ++i) {
        if (field.name == std::string(1, "xyz"[i]) &&
          field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1)
        {
          offsets[i] = static_cast<int>(field.offset);
        }
      }
    }
    for (const int offset : offsets) {
      if (offset < 0 || static_cast<unsigned>(offset) + sizeof(float) > cloud->point_step) {
        return;
      }
    }
    if (cloud->row_step < cloud->width * cloud->point_step ||
      cloud->data.size() < static_cast<std::size_t>(cloud->row_step) * cloud->height)
    {
      return;
    }
    for (std::size_t row = 0; row < cloud->height; ++row) {
      for (std::size_t col = 0; col < cloud->width; ++col) {
        auto * point = cloud->data.data() + row * cloud->row_step + col * cloud->point_step;
        float input[3];
        float output[3];
        for (int i = 0; i < 3; ++i) {
          std::memcpy(&input[i], point + offsets[i], sizeof(float));
        }
        for (int i = 0; i < 3; ++i) {
          output[i] = static_cast<float>(
            transform_[i * 4] * input[0] + transform_[i * 4 + 1] * input[1] +
            transform_[i * 4 + 2] * input[2] + transform_[i * 4 + 3]);
        }
        for (int i = 0; i < 3; ++i) {
          std::memcpy(point + offsets[i], &output[i], sizeof(float));
        }
      }
    }
    cloud->header.frame_id = "j2n6s300_link_base";
    publisher_->publish(std::move(cloud));
  }

  std::array<double, 12> transform_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Overlay>());
  rclcpp::shutdown();
}
