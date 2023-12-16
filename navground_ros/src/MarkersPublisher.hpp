// RVIZ Drawing (Just for debugging)

#ifndef MARKERS_PUBLISHER_HPP_
#define MARKERS_PUBLISHER_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <valarray>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "navground/core/common.h"
#include "navground/core/controller_3d.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using navground::core::Cylinder;
using navground::core::Neighbor3;
using navground::core::orientation_of;
using navground::core::Pose3;
using navground::core::unit3;
using navground::core::Vector2;
using navground::core::Vector3;

inline geometry_msgs::msg::Point to_msg(const Vector3 &point) {
  geometry_msgs::msg::Point p;
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();
  return p;
}

struct MarkersPublisher {
  rclcpp::Node &node;
  std::string ns;
  int obstacleIndex;
  bool enabled;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub;

  MarkersPublisher(rclcpp::Node &node)
      : node(node),
        ns(node.get_effective_namespace()),
        obstacleIndex(0),
        enabled(false) {
    pub = node.create_publisher<visualization_msgs::msg::MarkerArray>("markers",
                                                                      10);
  }

  std::string add_ns(const std::string &value) {
    if (ns.length()) {
      return ns + "/" + value;
    }
    return value;
  }

  visualization_msgs::msg::Marker desired_velocity_marker(
      const Vector2 &relative_velocity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = add_ns("base_link");
    // marker.header.stamp = node.now();
    marker.id = 0;
    marker.ns = add_ns("hl_desired_velocity");
    marker.type = visualization_msgs::msg::Marker::ARROW;

    marker.scale.x = std::max<double>(0, relative_velocity.norm());
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, orientation_of(relative_velocity));
    tf2::convert(quat_tf, marker.pose.orientation);

    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    return marker;
  }

  void publish_desired_velocity(const Vector2 &relative_velocity) {
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(desired_velocity_marker(relative_velocity));
    pub->publish(msg);
  }

  visualization_msgs::msg::Marker hl_collisions_marker(
      const std::valarray<ng_float_t> &angles, const std::valarray<ng_float_t> &ranges,
      float margin = 0.0) {
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = add_ns("base_link");
    // lines.header.stamp = node.now();
    lines.id = 0;
    lines.ns = add_ns("hl_collisions");
    lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.05;
    lines.color.r = 0.0;
    lines.color.g = 1.0;
    lines.color.b = 0.0;
    lines.color.a = 1.0f;
    lines.lifetime = rclcpp::Duration(0, 0);
    for (int i = 0; i < angles.size(); ++i) {
      if (ranges[i] < 0) continue;
      lines.points.push_back(to_msg((margin + ranges[i]) * unit3(angles[i])));
    }
    return lines;
  }

  void publish_hl_collisions(const std::valarray<ng_float_t> &angles,
                             const std::valarray<ng_float_t> &ranges,
                             float margin = 0.0) {
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(hl_collisions_marker(angles, ranges, margin));
    pub->publish(msg);
  }

  visualization_msgs::msg::Marker point_marker(const Vector3 &point,
                                               const std::string &frame_id,
                                               float tolerance = 0.0f) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    // marker.header.stamp = node.now();
    marker.id = 0;
    marker.ns = add_ns("target");
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.pose.position = to_msg(point);
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0 * (tolerance ? tolerance : 0.1);
    marker.scale.y = 2.0 * (tolerance ? tolerance : 0.1);
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = tolerance ? 1.0 : 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
  }

  visualization_msgs::msg::Marker orientation_marker(
      const Pose3 &pose, const std::string &frame_id, float tolerance = 0.0f,
      float length = 0.2f) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    // marker.header.stamp = node.now();
    marker.id = 0;
    marker.ns = add_ns("target_orientation");
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.points.push_back(to_msg(pose.position));
    marker.points.push_back(
        to_msg(pose.position + length * unit3(pose.orientation)));
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.color.r = 1.0;
    marker.color.g = tolerance ? 1.0 : 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
  }

  void publish_target(const Vector3 &point, const std::string &frame_id,
                      float tolerance = 0.0f) {
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(point_marker(point, frame_id, tolerance));
    pub->publish(msg);
  }

  void publish_target(const Pose3 &pose, const std::string &frame_id,
                      float position_tolerance = 0.0f,
                      float orientation_tolerance = 0.0f) {
    // TODO(Jerome): Add circular sector
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(
        point_marker(pose.position, frame_id, position_tolerance));
    msg.markers.push_back(
        orientation_marker(pose, frame_id, orientation_tolerance));
    pub->publish(msg);
  }

  std_msgs::msg::ColorRGBA color_for_type([[maybe_unused]] unsigned type) {
    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    c.a = 0.8;
    return c;
  }

  visualization_msgs::msg::Marker obstacle_marker(
      const Cylinder &obstacle, const std::string &frame_id, unsigned id,
      const std::string &marker_ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    // marker.header.stamp = node.now();
    marker.id = id;
    marker.ns = add_ns(marker_ns);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.pose.position.x = obstacle.position.x();
    marker.pose.position.y = obstacle.position.y();
    marker.pose.position.z = obstacle.position.z() + obstacle.height * 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = obstacle.radius * 2;
    marker.scale.y = obstacle.radius * 2;
    marker.scale.z = std::max<double>(0.1, obstacle.height);

    // Set the color -- be sure to set alpha to something non-zero!

    marker.color = color_for_type(0);
    marker.lifetime = rclcpp::Duration(0, 0);
    // marker.lifetime = rclcpp::Duration::from_seconds(updatePeriod * 1.5);
    return marker;
  }

  visualization_msgs::msg::Marker clear_obstacles(
      const std::string &marker_ns) {
    visualization_msgs::msg::Marker marker;
    // marker.header.stamp = node.now();
    marker.ns = add_ns(marker_ns);
    marker.type = 3;
    return marker;
  }

  void publish_neighbors(const std::vector<Neighbor3> neighbors,
                         const std::string &frame_id) {
    publish_obstacles_(neighbors, frame_id, "neighbors");
  }

  void publish_obstacles(const std::vector<Cylinder> obstacles,
                         const std::string &frame_id) {
    publish_obstacles_(obstacles, frame_id, "obstacles");
  }

  template <typename T>
  void publish_obstacles_(const std::vector<T> obstacles,
                          const std::string &frame_id,
                          const std::string &marker_ns) {
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.reserve(obstacles.size() + 1);
    unsigned i = 0;
    msg.markers.push_back(clear_obstacles(marker_ns));
    for (const auto &obstacle : obstacles) {
      msg.markers.push_back(
          obstacle_marker(obstacle, frame_id, i++, marker_ns));
    }
    pub->publish(msg);
  }
};

#endif  // MARKERS_PUBLISHER_HPP_
