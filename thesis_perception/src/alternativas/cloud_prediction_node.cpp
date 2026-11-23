// Copyright 2026 Luis Alberto Munoz Marin
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "thesis_perception/stationary_voxels.hpp"
#include "thesis_perception/motion_estimator.hpp"
#include "thesis_perception/swept_voxel_grid.hpp"

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
using Clock = std::chrono::steady_clock;

struct Track
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  thesis_perception::MotionEstimator motion;
  std::size_t size{};
  double stamp{};
  bool assigned{};
};

class CloudPrediction : public rclcpp::Node
{
public:
  CloudPrediction() : Node("cloud_prediction")
  {
    horizon_ = declare_parameter("horizon_s", 0.5);
    tolerance_ = declare_parameter("cluster_tolerance_m", 0.025);
    min_points_ = declare_parameter("min_cluster_points", 30);
    max_points_ = declare_parameter("max_cluster_points", 200000);
    gate_ = declare_parameter("association_distance_m", 0.055);
    timeout_ = declare_parameter("tracking_timeout_s", 0.5);
    max_speed_ = declare_parameter("max_speed_mps", 2.0);
    window_ = declare_parameter("velocity_window_s", 0.35);
    enter_speed_ = declare_parameter("moving_enter_speed_mps", 0.06);
    exit_speed_ = declare_parameter("moving_exit_speed_mps", 0.025);
    max_range_ = declare_parameter("max_valid_range_m", 1.0);
    edge_margin_ = declare_parameter("range_edge_margin_m", 0.02);
    stationary_enabled_ = declare_parameter("stationary_map_enabled", true);
    stationary_size_ = declare_parameter("stationary_voxel_m", 0.02);
    stationary_match_ = declare_parameter("stationary_match_m", 0.02);
    stationary_warmup_ = declare_parameter("stationary_warmup_s", 1.0);
    stationary_retention_ = declare_parameter("stationary_retention_s", 3.0);
    sweep_enabled_ = declare_parameter("sweep_enabled", true);
    sweep_voxel_m_ = declare_parameter("sweep_voxel_m", 0.02);
    sweep_max_cells_ = declare_parameter("sweep_max_cells", 40000);
    sweep_max_steps_ = declare_parameter("sweep_max_steps", 64);
    if (horizon_ <= 0 || tolerance_ <= 0 || min_points_ <= 0 ||
      max_points_ < min_points_ || gate_ <= 0 || timeout_ <= 0 || max_speed_ <= 0 ||
      window_ <= 0.12 || enter_speed_ <= exit_speed_ || exit_speed_ < 0 ||
      max_range_ <= 0 || edge_margin_ <= 0 || edge_margin_ >= max_range_ ||
      stationary_size_ <= 0 || stationary_match_ <= 0 || stationary_match_ > stationary_size_ ||
      stationary_warmup_ <= 0 || stationary_retention_ <= stationary_warmup_ ||
      !std::isfinite(sweep_voxel_m_) || sweep_voxel_m_ < 0.001 ||
      sweep_voxel_m_ > 1.0 || sweep_max_cells_ < 1 || sweep_max_steps_ < 1)
    {throw std::invalid_argument("Invalid cloud prediction parameters");}
    stationary_ = std::make_unique<thesis_perception::StationaryVoxels>(
      stationary_size_, stationary_match_, stationary_warmup_, stationary_retention_);
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    current_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/thesis/perception/tracked_points", qos);
    future_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/thesis/perception/predicted_points", qos);
    sweep_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/thesis/perception/swept_volume", qos);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/thesis/perception/prediction_diagnostics", 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("input_topic", "/thesis/perception/points_filtered"), qos,
      std::bind(&CloudPrediction::process, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (active_ && std::chrono::duration<double>(Clock::now() - last_received_).count() >
          timeout_) {reset("LOST: input timeout");}
        if (++ticks_ >= 5) {ticks_ = 0; publish_diagnostics();}
      });
  }
private:
  void reset(const std::string & reason)
  {
    lost_ += tracks_.size();
    tracks_.clear();
    stationary_->clear();
    active_ = false;
    moving_count_ = 0;
    unclustered_ = 0;
    stationary_suppressed_ = 0;
    clipped_ = false;
    edge_fraction_ = 0.0;
    status_ = reason;
    sensor_msgs::msg::PointCloud2 empty;
    empty.header = header_;
    current_->publish(empty);
    future_->publish(empty);
    publish_sweep(nullptr, header_);
    sweep_truncated_ = sweep_undersampled_ = false;
    sweep_cells_ = 0;
  }
  void publish_sweep(
    const thesis_perception::SweptVoxelGrid * grid, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "experimental_swept_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = sweep_voxel_m_;
    marker.color.r = 0.10f; marker.color.g = 0.70f;
    marker.color.b = 1.0f; marker.color.a = 0.25f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.action = grid && !grid->cells().empty() ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    if (grid) {
      marker.points.reserve(grid->cells().size());
      for (const auto & cell : grid->cells()) {
        const auto center = grid->center(cell);
        geometry_msgs::msg::Point point;
        point.x = center.x(); point.y = center.y(); point.z = center.z();
        marker.points.push_back(point);
      }
    }
    sweep_publisher_->publish(marker);
  }
  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Experimental cloud prediction";
    status.hardware_id = "RGB-D";
    status.level = (!active_ || clipped_ || sweep_truncated_ || sweep_undersampled_) ? 1 : 0;
    status.message = status_;
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key; item.value = value; status.values.push_back(item);
      };
    add("sweep_cells", std::to_string(sweep_cells_));
    add("sweep_truncated", sweep_truncated_ ? "true" : "false");
    add("sweep_undersampled", sweep_undersampled_ ? "true" : "false");
    add("tracks", std::to_string(tracks_.size()));
    add("moving_tracks", std::to_string(moving_count_));
    add("stationary_reference_points", std::to_string(stationary_suppressed_));
    add("stationary_map_voxels", std::to_string(stationary_->size()));
    add("unclustered_points", std::to_string(unclustered_));
    add("edge_fraction", std::to_string(edge_fraction_));
    add("range_edge_warning", clipped_ ? "true" : "false");
    add("tracks_started", std::to_string(started_));
    add("tracks_lost", std::to_string(lost_));
    array.status.push_back(status);
    diagnostics_->publish(array);
  }
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    const double age = now().seconds() - stamp;
    if (active_ && (msg->header.frame_id != header_.frame_id || stamp <= last_stamp_ ||
      stamp - last_stamp_ > timeout_)) {reset("LOST: timestamp or frame changed");}
    header_ = msg->header;
    if (stamp <= 0 || msg->header.frame_id.empty() || age > timeout_ || age < -0.1) {
      reset("LOST: invalid or stale input"); return;
    }
    auto raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *raw);
    auto cloud = std::make_shared<Cloud>();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*raw, *cloud, indices);
    active_ = true; last_stamp_ = stamp; last_received_ = Clock::now();
    if (cloud->empty()) {reset("WAITING: empty cloud"); return;}
    std::vector<bool> stationary_points(cloud->size(), false);
    stationary_suppressed_ = 0;
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      const auto & p = cloud->points[i];
      stationary_points[i] = stationary_enabled_ &&
        stationary_->contains(Eigen::Vector3f(p.x, p.y, p.z), stamp);
      if (stationary_points[i]) {++stationary_suppressed_;}
    }
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extraction;
    extraction.setClusterTolerance(tolerance_);
    extraction.setMinClusterSize(min_points_);
    extraction.setMaxClusterSize(max_points_);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    // A large mostly stationary component can contain a moving limb or object.
    // Split its nonstationary portion into local candidates rather than applying
    // one centroid to the entire body or discarding its motion outright.
    std::vector<pcl::PointIndices> candidates;
    for (const auto & component : clusters) {
      std::vector<int> changed;
      for (int index : component.indices) {
        if (!stationary_points[index]) {changed.push_back(index);}
      }
      if (!stationary_enabled_ || changed.size() * 2 > component.indices.size()) {
        candidates.push_back(component);
        continue;
      }
      if (changed.size() < static_cast<std::size_t>(min_points_)) {continue;}
      auto local_cloud = std::make_shared<Cloud>();
      local_cloud->reserve(changed.size());
      for (int index : changed) {local_cloud->push_back(cloud->points[index]);}
      auto local_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
      local_tree->setInputCloud(local_cloud);
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> local_extraction;
      local_extraction.setClusterTolerance(tolerance_);
      local_extraction.setMinClusterSize(min_points_);
      local_extraction.setMaxClusterSize(max_points_);
      local_extraction.setSearchMethod(local_tree);
      local_extraction.setInputCloud(local_cloud);
      std::vector<pcl::PointIndices> local_clusters;
      local_extraction.extract(local_clusters);
      for (auto & local : local_clusters) {
        for (int & index : local.indices) {index = changed[index];}
        candidates.push_back(std::move(local));
      }
    }
    clusters = std::move(candidates);
    std::vector<Eigen::Vector3d> centers;
    for (const auto & cluster : clusters) {
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (int index : cluster.indices) {
        const auto & p = cloud->points[index]; c += Eigen::Vector3d(p.x, p.y, p.z);
      }
      centers.push_back(c / static_cast<double>(cluster.indices.size()));
    }
    for (auto & track : tracks_) {track.assigned = false;}
    struct Match {double distance; std::size_t track; std::size_t cluster;};
    std::vector<Match> matches;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      const double dt = stamp - tracks_[i].stamp;
      if (dt <= 0 || dt > timeout_) {continue;}
      for (std::size_t j = 0; j < clusters.size(); ++j) {
        const double ratio = static_cast<double>(clusters[j].indices.size()) / tracks_[i].size;
        const double current_distance = (centers[j] - tracks_[i].center).norm();
        const double projected_distance = tracks_[i].motion.moving() ?
          (centers[j] - (tracks_[i].center + tracks_[i].motion.velocity() * dt)).norm() :
          current_distance;
        const double distance = std::min(current_distance, projected_distance);
        if (ratio >= 0.2 && ratio <= 5.0 &&
          distance <= gate_ + 0.35 * std::min(0.1, dt))
        {matches.push_back({distance, i, j});}
      }
    }
    std::sort(matches.begin(), matches.end(), [](const Match & a, const Match & b) {
        return a.distance < b.distance;
      });
    std::vector<int> assigned(clusters.size(), -1);
    for (const auto & match : matches) {
      if (tracks_[match.track].assigned || assigned[match.cluster] >= 0) {continue;}
      tracks_[match.track].assigned = true;
      assigned[match.cluster] = static_cast<int>(match.track);
    }
    thesis_perception::SweptVoxelGrid sweep(
      sweep_voxel_m_, static_cast<std::size_t>(sweep_max_cells_), sweep_max_steps_);
    Cloud tracked = *cloud, predicted = *cloud;
    for (auto & p : tracked.points) {p.r = 0; p.g = 255; p.b = 0;}
    for (auto & p : predicted.points) {p.r = 0; p.g = 255; p.b = 0;}
    moving_count_ = 0;
    unclustered_ = cloud->size();
    std::vector<bool> moving_point(cloud->size(), false);
    for (std::size_t j = 0; j < clusters.size(); ++j) {
      if (assigned[j] < 0) {
        tracks_.emplace_back(); assigned[j] = static_cast<int>(tracks_.size() - 1); ++started_;
      }
      auto & track = tracks_[assigned[j]];
      track.assigned = true;
      track.motion.update(stamp, centers[j], window_, enter_speed_, exit_speed_, max_speed_);
      track.center = centers[j]; track.size = clusters[j].indices.size(); track.stamp = stamp;
      unclustered_ -= cluster_size(clusters[j]);
      if (!track.motion.moving()) {continue;}
      std::size_t nonstationary_count = 0;
      for (int index : clusters[j].indices) {
        if (!stationary_points[index]) {++nonstationary_count;}
      }
      // A mostly stable component is likely background revealed by occlusion.
      if (stationary_enabled_ && nonstationary_count * 2 <= clusters[j].indices.size()) {
        continue;
      }
      ++moving_count_;
      for (int index : clusters[j].indices) {
        moving_point[index] = true;
        auto & p = predicted.points[index];
        if (sweep_enabled_) {
          sweep.add(Eigen::Vector3d(p.x, p.y, p.z), track.motion.velocity() * horizon_);
        }
        p.x += track.motion.velocity().x() * horizon_;
        p.y += track.motion.velocity().y() * horizon_;
        p.z += track.motion.velocity().z() * horizon_;
        p.r = 255; p.g = 100; p.b = 0;
      }
    }
    if (stationary_enabled_) {
      for (std::size_t i = 0; i < cloud->size(); ++i) {
        if (moving_point[i]) {continue;}
        const auto & p = cloud->points[i];
        stationary_->observe(Eigen::Vector3f(p.x, p.y, p.z), stamp);
      }
      if (++stationary_ticks_ >= 30) {stationary_->expire(stamp); stationary_ticks_ = 0;}
    }
    const auto old_size = tracks_.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [stamp, this](const Track & t) {
        return stamp - t.stamp > timeout_;
      }), tracks_.end());
    lost_ += old_size - tracks_.size();
    std::size_t near_edge = 0;
    for (const auto & p : cloud->points) {
      if (std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) >= max_range_ - edge_margin_) {++near_edge;}
    }
    edge_fraction_ = static_cast<double>(near_edge) / cloud->size();
    clipped_ = edge_fraction_ >= 0.03;
    sweep_cells_ = sweep.cells().size();
    sweep_truncated_ = sweep.truncated();
    sweep_undersampled_ = sweep.undersampled();
    status_ = sweep_truncated_ || sweep_undersampled_ ? "DEGRADED: sweep incomplete" :
      (clipped_ ? "DEGRADED: points near range boundary" : "ACTIVE: experimental preview");
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(tracked, output); output.header = msg->header; current_->publish(output);
    pcl::toROSMsg(predicted, output); output.header = msg->header; future_->publish(output);
    publish_sweep(sweep_enabled_ ? &sweep : nullptr, msg->header);
  }
  static std::size_t cluster_size(const pcl::PointIndices & c) {return c.indices.size();}
  double horizon_, tolerance_, gate_, timeout_, max_speed_, window_, enter_speed_, exit_speed_;
  double max_range_, edge_margin_, last_stamp_{}, edge_fraction_{};
  double stationary_size_, stationary_match_, stationary_warmup_, stationary_retention_;
  bool stationary_enabled_{};
  int stationary_ticks_{};
  std::size_t stationary_suppressed_{};
  std::unique_ptr<thesis_perception::StationaryVoxels> stationary_;
  int min_points_, max_points_, ticks_{};
  bool sweep_enabled_{}, sweep_truncated_{}, sweep_undersampled_{};
  double sweep_voxel_m_{};
  int sweep_max_cells_{}, sweep_max_steps_{};
  std::size_t sweep_cells_{};
  bool active_{}, clipped_{};
  std::size_t started_{}, lost_{}, moving_count_{}, unclustered_{};
  std::string status_{"WAITING: no cloud"};
  std::vector<Track> tracks_;
  std_msgs::msg::Header header_;
  Clock::time_point last_received_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_, future_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sweep_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudPrediction>());
  rclcpp::shutdown();
  return 0;
}
