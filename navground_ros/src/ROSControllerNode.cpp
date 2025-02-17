/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

// TODO(Jerome): localization check

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#define _USE_MATH_DEFINES
#include <cctype>
#include <cmath>
#include <memory>
#include <type_traits>

#include "./MarkersPublisher.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navground/core/behavior.h"
#include "navground/core/behaviors/HL.h"
#include "navground/core/behaviors/HRVO.h"
#include "navground/core/behaviors/ORCA.h"
#include "navground/core/controller.h"
#include "navground/core/controller_3d.h"
#include "navground/core/plugins.h"
#include "navground/core/property.h"
#include "navground/core/types.h"
#include "navground_msgs/action/go_to_target.hpp"
#include "navground_msgs/msg/neighbors.hpp"
#include "navground_msgs/msg/obstacles.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using std::placeholders::_2;
using GoToTarget = navground_msgs::action::GoToTarget;
using GoalHandleGoToTarget = rclcpp_action::ServerGoalHandle<GoToTarget>;

namespace navground::core {

template <typename T> struct is_ros_param_type : std::false_type {};
template <> struct is_ros_param_type<bool> : std::true_type {};
template <> struct is_ros_param_type<int> : std::true_type {};
template <> struct is_ros_param_type<float> : std::true_type {};
template <> struct is_ros_param_type<std::string> : std::true_type {};
template <> struct is_ros_param_type<std::vector<bool>> : std::true_type {};
template <> struct is_ros_param_type<std::vector<int>> : std::true_type {};
template <> struct is_ros_param_type<std::vector<float>> : std::true_type {};
template <>
struct is_ros_param_type<std::vector<std::string>> : std::true_type {};

static std::shared_ptr<Kinematics>
make_kinematics(const std::string &name, ng_float_t max_speed,
                ng_float_t max_angular_speed, ng_float_t axis,
                ng_float_t max_acceleration,
                ng_float_t max_angular_acceleration) {
  auto kinematics = Kinematics::make_type(name);
  if (kinematics) {
    kinematics->set_max_speed(max_speed);
    kinematics->set_max_angular_speed(max_angular_speed);
    if (TwoWheelsDifferentialDriveKinematics *wk =
            dynamic_cast<TwoWheelsDifferentialDriveKinematics *>(
                kinematics.get())) {
      wk->set_wheel_axis(axis);
    }
    if (DynamicTwoWheelsDifferentialDriveKinematics *wk =
            dynamic_cast<DynamicTwoWheelsDifferentialDriveKinematics *>(
                kinematics.get())) {
      wk->set_max_acceleration(max_acceleration);
      wk->set_max_angular_acceleration(max_angular_acceleration);
    }
    return kinematics;
  }
  return std::make_shared<OmnidirectionalKinematics>(max_speed,
                                                     max_angular_speed);
}

static Behavior::Heading heading_from_string(const std::string &name) {
  if (name == "target_point") {
    return Behavior::Heading::target_point;
  }
  if (name == "target_angle") {
    return Behavior::Heading::target_angle;
  }
  if (name == "target_angular_speed") {
    return Behavior::Heading::target_angle;
  }
  if (name == "velocity") {
    return Behavior::Heading::velocity;
  }
  return Behavior::Heading::idle;
}

static Vector3 vector_from(const geometry_msgs::msg::Vector3 &v) {
  return {static_cast<ng_float_t>(v.x), static_cast<ng_float_t>(v.y),
          static_cast<ng_float_t>(v.z)};
}

static Vector3 point_from(const geometry_msgs::msg::Point &v) {
  return {static_cast<ng_float_t>(v.x), static_cast<ng_float_t>(v.y),
          static_cast<ng_float_t>(v.z)};
}

using Transform = Eigen::Transform<ng_float_t, 3, Eigen::Isometry>;

static Transform transform_from(const geometry_msgs::msg::Transform &t) {
  return Transform(
      Eigen::Translation<ng_float_t, 3>(
          static_cast<ng_float_t>(t.translation.x),
          static_cast<ng_float_t>(t.translation.y),
          static_cast<ng_float_t>(t.translation.z)) *
      Eigen::Quaternion<ng_float_t>(static_cast<ng_float_t>(t.rotation.w),
                                    static_cast<ng_float_t>(t.rotation.x),
                                    static_cast<ng_float_t>(t.rotation.y),
                                    static_cast<ng_float_t>(t.rotation.z)));
}

static Transform transform_from(const geometry_msgs::msg::Pose &t) {
  return Transform(
      Eigen::Translation<ng_float_t, 3>(static_cast<ng_float_t>(t.position.x),
                                        static_cast<ng_float_t>(t.position.y),
                                        static_cast<ng_float_t>(t.position.z)) *
      Eigen::Quaternion<ng_float_t>(static_cast<ng_float_t>(t.orientation.w),
                                    static_cast<ng_float_t>(t.orientation.x),
                                    static_cast<ng_float_t>(t.orientation.y),
                                    static_cast<ng_float_t>(t.orientation.z)));
}

static ng_float_t yaw_from(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z));
}

// static float yaw_from(const Eigen::Quaternionf &q) {
//   return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
//                     1 - 2 * (q.y() * q.y() + q.z() * q.z()));
// }

static ng_float_t yaw_from(const Transform &t) {
  const auto rpy = t.linear().eulerAngles(2, 1, 0);
  if (std::abs(rpy[1]) > HALF_PI && std::abs(rpy[2]) > HALF_PI) {
    return rpy[0] + M_PI;
  }
  return rpy[0];
}

static Pose3 pose_from(const Transform &value) {
  return {value.translation(), yaw_from(value)};
}

static Pose3 pose_from(const geometry_msgs::msg::Pose &pose) {
  return {point_from(pose.position), yaw_from(pose.orientation)};
}

static Twist3 twist_from(const geometry_msgs::msg::Twist &t) {
  return {vector_from(t.linear), static_cast<Radians>(t.angular.z),
          Frame::absolute};
}

static std::string tolower(const std::string &value) {
  std::string r;
  r.resize(value.size());
  transform(value.begin(), value.end(), r.begin(), ::tolower);
  return r;
}

// static std::optional<Property::Field> get_from_param(const Property
// &property,
//                                       const rclcpp::Parameter &param) {
//   return std::visit(
//       [&param](auto &&arg) -> std::optional<Property::Field> {
//         using T = std::decay_t<decltype(arg)>;
//         if constexpr (is_ros_param_type<T>) {
//           return static_cast<T>(param.get_value<T>());
//         } else {
//           return std::nullopt;
//         }
//       },
//       property.default_value);
// }

static std::optional<Property::Field>
get_from_param(const Property &property, const rclcpp::Parameter &param) {
  return std::visit(
      [&param](auto &&arg) -> std::optional<Property::Field> {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int> ||
                      std::is_same_v<T, float> ||
                      std::is_same_v<T, std::string> ||
                      std::is_same_v<T, std::vector<bool>> ||
                      std::is_same_v<T, std::vector<std::string>>) {
          return static_cast<T>(param.get_value<T>());
        } else if constexpr (std::is_same_v<T, std::vector<int>>) {
          const auto vs = param.as_integer_array();
          std::vector<int> rs;
          rs.reserve(vs.size());
          std::transform(vs.begin(), vs.end(), rs.begin(),
                         [](auto v) { return static_cast<int>(v); });
          return rs;
        } else if constexpr (std::is_same_v<T, std::vector<ng_float_t>>) {
          const auto vs = param.as_double_array();
          std::vector<ng_float_t> rs;
          rs.reserve(vs.size());
          std::transform(vs.begin(), vs.end(), rs.begin(),
                         [](auto v) { return static_cast<ng_float_t>(v); });
          return rs;
        } else {
          return std::nullopt;
        }
      },
      property.default_value);
}

class ROSControllerNode : public rclcpp::Node {
public:
  ROSControllerNode()
      : rclcpp::Node("controller"), controller(), markers_pub(*this),
        fixed_frame(""), goal_handle(nullptr) {
    const double rate = declare_parameter("rate", 10.0);
    update_period = 1.0 / rate;
    param_callback_handle = add_on_set_parameters_callback(
        std::bind(&ROSControllerNode::on_set_parameters, this, _1));
    init_params();
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    if (should_publish_cmd_stamped) {
      cmd_stamped_publisher =
          create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped",
                                                             1);
    } else {
      cmd_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }
    stop_subscriber = create_subscription<std_msgs::msg::Empty>(
        "stop", 1, std::bind(&ROSControllerNode::stop_cb, this, _1));
    target_pose_subscriber =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 1,
            std::bind(&ROSControllerNode::target_pose_cb, this, _1));
    // TODO(Jerome 2023): complete
    target_twist_subscriber =
        create_subscription<geometry_msgs::msg::TwistStamped>(
            "target_twist", 1,
            std::bind(&ROSControllerNode::target_twist_cb, this, _1));
    target_point_subscriber =
        create_subscription<geometry_msgs::msg::PointStamped>(
            "target_point", 1,
            std::bind(&ROSControllerNode::target_point_cb, this, _1));
    obstacles_subscriber = create_subscription<navground_msgs::msg::Obstacles>(
        "obstacles", 1, std::bind(&ROSControllerNode::obstacles_cb, this, _1));
    neighbors_subscriber = create_subscription<navground_msgs::msg::Neighbors>(
        "neighbors", 1, std::bind(&ROSControllerNode::neighbors_cb, this, _1));
    odometry_subscriber = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&ROSControllerNode::odom_cb, this, _1));
    action_server = rclcpp_action::create_server<GoToTarget>(
        this, "go_to", std::bind(&ROSControllerNode::handle_goal, this, _1, _2),
        std::bind(&ROSControllerNode::handle_cancel, this, _1),
        std::bind(&ROSControllerNode::handle_accepted, this, _1));
    controller.set_cmd_cb(std::bind(&ROSControllerNode::publish_cmd, this, _1));
    timer = create_wall_timer(
        std::chrono::milliseconds((unsigned)(1e3 * update_period)),
        std::bind(&ROSControllerNode::update, this));
    RCLCPP_INFO(get_logger(), "Ready");
  }

private:
  Controller3 controller;
  MarkersPublisher markers_pub;
  double update_period;
  bool should_publish_cmd_stamped;
  std::string fixed_frame;
  rclcpp::Time last_localization_stamp;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      target_point_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      target_twist_subscriber;
  rclcpp::Subscription<navground_msgs::msg::Obstacles>::SharedPtr
      obstacles_subscriber;
  rclcpp::Subscription<navground_msgs::msg::Neighbors>::SharedPtr
      neighbors_subscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_stamped_publisher;
  std::shared_ptr<GoalHandleGoToTarget> goal_handle;
  rclcpp_action::Server<navground_msgs::action::GoToTarget>::SharedPtr
      action_server;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  void update() {
    if (goal_handle && goal_handle->is_canceling()) {
      auto result = std::make_shared<GoToTarget::Result>();
      goal_handle->canceled(result);
      goal_handle = nullptr;
      controller.stop();
    }
    controller.update_3d(update_period);
  }

  void done(Action::State state) {
    if (goal_handle) {
      auto r = std::make_shared<GoToTarget::Result>();
      if (state == Action::State::success) {
        RCLCPP_INFO(get_logger(), "Set goal reached");
        goal_handle->succeed(r);
      } else {
        RCLCPP_INFO(get_logger(), "Aborted");
        goal_handle->abort(r);
      }
      goal_handle = nullptr;
    }
  }

  void running(ng_float_t time_remaining) {
    if (goal_handle) {
      auto f = std::make_shared<GoToTarget::Feedback>();
      // TODO(J): Modify interface
      f->distance = time_remaining;
      goal_handle->publish_feedback(f);
      if (markers_pub.enabled) {
        Behavior *behavior = controller.get_behavior().get();
        HLBehavior *hl = dynamic_cast<HLBehavior *>(behavior);
        if (hl) {
          // TODO(Jerome): should publish them only when moving, not turning
          markers_pub.publish_hl_collisions(hl->get_collision_angles(),
                                            hl->get_collision_distance());
        }
        if (behavior) {
          markers_pub.publish_desired_velocity(
              behavior->get_desired_velocity());
        }
      }
    }
  }

  std::optional<Transform> get_transform(const std::string &from,
                                         const std::string &to,
                                         const tf2::TimePoint &) {
    try {
      return transform_from(
          tf_buffer->lookupTransform(from, to, tf2::TimePointZero).transform);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "No transform from %s to %s: %s", from.c_str(),
                   to.c_str(), ex.what());
      return std::nullopt;
    }
  }

  std::optional<Twist3>
  twist_from_msg(const geometry_msgs::msg::TwistStamped &msg,
                 const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return twist_from(msg.twist);
    }
    const auto t = get_transform(frame_id, msg.header.frame_id,
                                 tf2_ros::fromMsg(msg.header.stamp));
    if (!t)
      return std::nullopt;
    const Vector3 linear = t->linear() * vector_from(msg.twist.linear);
    const Vector3 angular = t->linear() * vector_from(msg.twist.angular);
    return Twist3{linear, angular.z()};
  }

  std::optional<Vector3>
  vector_from_msg(const geometry_msgs::msg::Vector3Stamped &msg,
                  const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return vector_from(msg.vector);
    }
    const auto t = get_transform(frame_id, msg.header.frame_id,
                                 tf2_ros::fromMsg(msg.header.stamp));
    if (!t)
      return std::nullopt;
    return t->linear() * vector_from(msg.vector);
  }

  std::optional<Vector3>
  point_from_msg(const geometry_msgs::msg::PointStamped &msg,
                 const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return point_from(msg.point);
    }
    const auto t = get_transform(frame_id, msg.header.frame_id,
                                 tf2_ros::fromMsg(msg.header.stamp));
    if (!t)
      return std::nullopt;
    return *t * point_from(msg.point);
  }

  std::optional<Pose3> pose_from_msg(const geometry_msgs::msg::PoseStamped &msg,
                                     const std::string &frame_id) {
    if (frame_id == msg.header.frame_id) {
      return pose_from(msg.pose);
    }
    const auto t = get_transform(frame_id, msg.header.frame_id,
                                 tf2_ros::fromMsg(msg.header.stamp));
    if (!t)
      return std::nullopt;
    return pose_from(*t * transform_from(msg.pose));
  }

  std::optional<std::tuple<Pose3, Twist3>>
  odom_from_msg(const nav_msgs::msg::Odometry &msg,
                const std::string &frame_id) {
    Pose3 pose;
    Twist3 twist;
    std::optional<Transform> t;
    if (frame_id == msg.header.frame_id) {
      pose = pose_from(msg.pose.pose);
    } else {
      t = get_transform(frame_id, msg.header.frame_id,
                        tf2_ros::fromMsg(msg.header.stamp));
      if (!t)
        return std::nullopt;
      pose = pose_from(*t * transform_from(msg.pose.pose));
    }
    if (frame_id == msg.child_frame_id) {
      twist = twist_from(msg.twist.twist);
    } else {
      if (msg.child_frame_id != msg.header.frame_id) {
        t = get_transform(frame_id, msg.child_frame_id,
                          tf2_ros::fromMsg(msg.header.stamp));
        if (!t)
          return std::nullopt;
      }
      const auto linear = t->linear() * vector_from(msg.twist.twist.linear);
      const auto angular = t->linear() * vector_from(msg.twist.twist.angular);
      twist = {linear, angular.z()};
    }
    return std::make_tuple(pose, twist);
  }

  void odom_cb(const nav_msgs::msg::Odometry &msg) {
    last_localization_stamp = now();
    const auto odom = odom_from_msg(msg, fixed_frame);
    if (!odom)
      return;
    const auto &[pose, twist] = *odom;
    RCLCPP_INFO(get_logger(),
                "Odom -> %.2f %.2f %.2f %.2f-- %.2f %.2f %.2f %.2f",
                pose.position.x(), pose.position.y(), pose.position.z(),
                pose.orientation, twist.velocity.x(), twist.velocity.y(),
                twist.velocity.z(), twist.angular_speed);
    controller.set_twist(twist);
    controller.set_pose(pose);
  }

  void target_point_cb(const geometry_msgs::msg::PointStamped &msg) {
    if (goal_handle)
      return;
    const auto target = point_from_msg(msg, fixed_frame);
    if (target) {
      controller.follow_point(*target);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target, fixed_frame, 0.0);
      }
    }
  }

  void target_pose_cb(const geometry_msgs::msg::PoseStamped &msg) {
    if (goal_handle)
      return;
    const auto target = pose_from_msg(msg, fixed_frame);
    if (target) {
      controller.follow_pose(*target);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target, fixed_frame, 0.0);
      }
    }
  }

  void target_twist_cb(const geometry_msgs::msg::TwistStamped &msg) {
    if (goal_handle)
      return;
    const auto target = twist_from_msg(msg, fixed_frame);
    if (target) {
      controller.follow_twist(*target);
    }
  }

  void stop() {
    if (goal_handle) {
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->abort(r);
      goal_handle = nullptr;
    }
    controller.stop();
    publish_cmd({});
  }

  void stop_cb([[maybe_unused]] const std_msgs::msg::Empty &msg) { stop(); }

  rclcpp_action::GoalResponse
  handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
              [[maybe_unused]] std::shared_ptr<const GoToTarget::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    if (!goal_handle && controller.get_behavior())
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToTarget> _goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if (_goal_handle == goal_handle) {
      return rclcpp_action::CancelResponse::ACCEPT;
    } else {
      return rclcpp_action::CancelResponse::REJECT;
    }
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleGoToTarget> _goal_handle) {
    goal_handle = _goal_handle;
    auto goal = goal_handle->get_goal();
    std::optional<Vector3> target_point;
    std::optional<Pose3> target_pose;
    if (goal->target_pose.header.frame_id != "") {
      target_pose = pose_from_msg(goal->target_pose, fixed_frame);
    } else {
      target_point = point_from_msg(goal->target_point, fixed_frame);
    }
    if (!target_pose && !target_point) {
      RCLCPP_WARN(get_logger(), "Goal not valid");
      auto r = std::make_shared<GoToTarget::Result>();
      goal_handle->abort(r);
      goal_handle = nullptr;
    } else if (target_point) {
      RCLCPP_WARN(get_logger(), "go_to_position");
      auto action =
          controller.go_to_position(*target_point, goal->position_tolerance);
      action->running_cb = std::bind(&ROSControllerNode::running, this, _1);
      action->done_cb = std::bind(&ROSControllerNode::done, this, _1);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target_point, fixed_frame,
                                   goal->position_tolerance);
      }
    } else {
      RCLCPP_WARN(get_logger(), "go_to_pose");
      auto action = controller.go_to_pose(
          *target_pose, goal->position_tolerance, goal->orientation_tolerance);
      action->running_cb = std::bind(&ROSControllerNode::running, this, _1);
      action->done_cb = std::bind(&ROSControllerNode::done, this, _1);
      if (markers_pub.enabled) {
        markers_pub.publish_target(*target_pose, fixed_frame,
                                   goal->position_tolerance,
                                   goal->orientation_tolerance);
      }
    }
  }

  void publish_cmd(const Twist3 &twist) {
    if (should_publish_cmd_stamped) {
      // publish in odom frame
      geometry_msgs::msg::TwistStamped msg;
      msg.header.frame_id = fixed_frame;
      msg.header.stamp = now();
      if (twist.frame == Frame::relative) {
        const auto behavior = controller.get_behavior();
        if (!behavior)
          return;
        Twist2 a_twist = behavior->to_absolute(twist.project());
        msg.twist.linear.x = a_twist.velocity.x();
        msg.twist.linear.y = a_twist.velocity.y();
      } else {
        msg.twist.linear.x = twist.velocity.x();
        msg.twist.linear.y = twist.velocity.y();
      }
      msg.twist.linear.z = twist.velocity.z();
      msg.twist.angular.z = twist.angular_speed;
      cmd_stamped_publisher->publish(msg);
    } else {
      // publish in robot frame
      geometry_msgs::msg::Twist msg;
      if (twist.frame == Frame::absolute) {
        const auto behavior = controller.get_behavior();
        if (!behavior)
          return;
        Twist2 r_twist = behavior->to_relative(twist.project());
        msg.linear.x = r_twist.velocity.x();
        msg.linear.y = r_twist.velocity.y();
      } else {
        msg.linear.x = twist.velocity.x();
        msg.linear.y = twist.velocity.y();
      }
      msg.linear.z = twist.velocity.z();
      msg.angular.z = twist.angular_speed;
      cmd_publisher->publish(msg);
    }
  }

  void neighbors_cb(const navground_msgs::msg::Neighbors &msg) {
    std::vector<Neighbor3> neighbors;
    for (const auto &nmsg : msg.neighbors) {
      auto position = point_from_msg(nmsg.obstacle.top_position, fixed_frame);
      if (!position)
        return;
      const auto velocity = vector_from_msg(nmsg.velocity, fixed_frame);
      if (!velocity)
        return;
      *position -= Vector3(0, 0, nmsg.obstacle.height);
      neighbors.emplace_back(*position, nmsg.obstacle.radius,
                             nmsg.obstacle.height, velocity->head<2>(),
                             nmsg.id);
    }
    controller.set_neighbors(neighbors);
    if (markers_pub.enabled) {
      markers_pub.publish_neighbors(neighbors, fixed_frame);
    }
  }

  void obstacles_cb(const navground_msgs::msg::Obstacles &msg) {
    std::vector<Cylinder> obstacles;
    for (const auto &omsg : msg.obstacles) {
      auto position = point_from_msg(omsg.top_position, fixed_frame);
      if (!position)
        return;
      *position -= Vector3(0, 0, omsg.height);
      obstacles.emplace_back(*position, omsg.radius, omsg.height);
    }
    controller.set_static_obstacles(obstacles);
    if (markers_pub.enabled) {
      markers_pub.publish_obstacles(obstacles, fixed_frame);
    }
  }

  void init_params() {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;
    auto kinematics = make_kinematics(
        declare_parameter("kinematics.type", std::string("2WDiff"), param_desc),
        declare_parameter("kinematics.max_speed", 1.0, param_desc),
        declare_parameter("kinematics.max_angular_speed", 1.0, param_desc),
        declare_parameter("kinematics.wheel_axis", 1.0, param_desc),
        declare_parameter("kinematics.max_acceleration", 1.0, param_desc),
        declare_parameter("kinematics.max_angular_acceleration", 0.0,
                          param_desc));
    declare_parameter("radius", 0.0, param_desc);
    should_publish_cmd_stamped =
        declare_parameter("publish_cmd_stamped", false, param_desc);
    controller.set_cmd_frame(should_publish_cmd_stamped ? Frame::absolute
                                                        : Frame::relative);
    fixed_frame = declare_parameter("frame_id", "world", param_desc);
    declare_parameter("altitude.tau", 1.0);
    declare_parameter("altitude.optimal_speed", 0.1);
    declare_parameter("altitude.enabled", false);
    declare_parameter("speed_tolerance", 0.05);
    declare_parameter("optimal_speed", 0.3);
    declare_parameter("optimal_angular_speed", 0.3);
    declare_parameter("rotation_tau", 0.5);
    declare_parameter("horizon", 1.0);
    declare_parameter("safety_margin", 0.1);
    declare_parameter("heading", "idle");
    declare_parameter("drawing", false);
    for (const auto &[type, properties] : Behavior::type_properties()) {
      const std::string prefix = tolower(type) + ".";
      for (const auto &[name, property] : properties) {
        const std::string param_name = prefix + name;
        std::visit(
            [&param_name, this](auto &&arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr (is_ros_param_type<T>::value) {
                declare_parameter(param_name, arg);
              }
            },
            property.default_value);
      }
    }
    param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description =
        "The name of the obstacle avoidance behavior.\nOne of: ";
    for (const auto &type : Behavior::types()) {
      param_desc.description += type + ", ";
    }
    param_desc.description += ".";
    declare_parameter("behavior", "HL", param_desc);
  }

  void set_behavior(std::string type) {
    auto behavior = controller.get_behavior();
    if (behavior && behavior->get_type() == type)
      return;
    const auto &type_properties = Behavior::type_properties();
    if (!type_properties.count(type)) {
      RCLCPP_WARN(get_logger(), "Unknown behavior type %s", type.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Set behavior type to %s", type.c_str());

    // MAYBE: move this to the core lib
    bool effective_center = false;
    if (type == "ORCA-NH") {
      type = "ORCA";
      effective_center = true;
    }
    // -- MAYBE

    auto new_behavior = Behavior::make_type(type);

    // MAYBE: move this to the core lib
    if (effective_center) {
      dynamic_cast<ORCABehavior *>(new_behavior.get())
          ->should_use_effective_center(true);
    }
    // -- MAYBE

    const auto &properties = type_properties.at(type);
    const std::string prefix = tolower(type) + ".";
    for (const auto &[name, property] : properties) {
      const std::string param_name = prefix + name;
      const auto param = get_parameter(param_name);
      const auto value = get_from_param(property, param);
      if (value) {
        new_behavior->set(name, *value);
      }
    }

    if (behavior) {
      new_behavior->set_state_from(*behavior);
    } else {
      new_behavior->set_optimal_speed(
          get_parameter("optimal_speed").as_double());
      new_behavior->set_optimal_angular_speed(
          get_parameter("optimal_angular_speed").as_double());
      new_behavior->set_rotation_tau(get_parameter("rotation_tau").as_double());
      new_behavior->set_safety_margin(
          get_parameter("safety_margin").as_double());
      new_behavior->set_horizon(get_parameter("horizon").as_double());
      new_behavior->set_heading_behavior(
          heading_from_string(get_parameter("heading").as_string()));
    }
    controller.set_behavior(new_behavior);
  }

  rcl_interfaces::msg::SetParametersResult
  on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    Behavior *b = controller.get_behavior().get();
    const std::string prefix = b ? (tolower(b->get_type()) + ".") : "";
    const Properties &properties = b ? b->get_properties() : Properties{};
    for (const auto &param : parameters) {
      std::string name = param.get_name();
      // RCLCPP_INFO(get_logger(), "on_set_parameter %s", name.c_str());
      if (name == "behavior") {
        set_behavior(param.as_string());
      } else if (name == "altitude.enabled") {
        controller.should_be_limited_to_2d(!param.as_bool());
      } else if (name == "altitude.tau") {
        controller.set_altitude_tau(param.as_double());
      } else if (name == "altitude.optimal_speed") {
        controller.set_altitude_optimal_speed(param.as_double());
      } else if (name == "drawing") {
        markers_pub.enabled = param.as_bool();
      } else if (name == "speed_tolerance") {
        controller.set_speed_tolerance(param.as_double());
      }
      if (b) {
        if (name.find(prefix) != std::string::npos) {
          name = name.substr(prefix.size());
          if (properties.count(name)) {
            const auto value = get_from_param(properties.at(name), param);
            if (value) {
              b->set(name, *value);
            }
          }
        }
        if (name == "optimal_speed") {
          b->set_optimal_speed(param.as_double());
        } else if (name == "optimal_angular_speed") {
          b->set_optimal_angular_speed(param.as_double());
        } else if (name == "rotation_tau") {
          b->set_rotation_tau(param.as_double());
        } else if (name == "safety_margin") {
          b->set_safety_margin(param.as_double());
        } else if (name == "horizon") {
          b->set_horizon(param.as_double());
        } else if (name == "heading") {
          // TODO(Jerome): check value
          b->set_heading_behavior(heading_from_string(param.as_string()));
        }
      }
    }
    return result;
  }
};

} // namespace navground::core

int main(int argc, char *argv[]) {
  navground::core::load_plugins();
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navground::core::ROSControllerNode>();
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("navground"), "Shutting down");
  rclcpp::shutdown();
  return 0;
}
