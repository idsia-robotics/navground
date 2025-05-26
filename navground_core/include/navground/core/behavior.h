/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_H_
#define NAVGROUND_CORE_BEHAVIOR_H_

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "navground/core/behavior_modulation.h"
#include "navground/core/common.h"
#include "navground/core/export.h"
#include "navground/core/kinematics.h"
#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/social_margin.h"
#include "navground/core/state.h"
#include "navground/core/target.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief      This class describes a generic behavior to reach a target
 * position avoiding collision.
 *
 * Users should not instantiate this class, which will not make the agent move,
 * but one of its concrete sub-classes equipped with the desired navigation
 * algorithms, which are implemented by overriding any of the (virtual) methods
 * listed in \ref compute_cmd_internal.
 *
 * The following describes the typical usage of a behavior.
 *
 * *At initialization*
 *
 * 1. select the concrete behavior, the agent's size (agents are shaped like
 *    discs), and \ref Kinematics;
 *
 * 2. configure the generic parameters: \ref set_optimal_speed, \ref
 * set_horizon, \ref set_safety_margin \ref set_rotation_tau, and \ref
 * set_heading_behavior;
 *
 * 3. configure the specific parameters of the concrete behavior.
 *
 * 4. call \ref prepare to finalize the initialization
 *
 * *At regular time intervals*
 *
 * 1. update the agent's state with \ref set_pose and \ref set_twist (or other
 *    convenience methods)
 *
 * 2. update the target with \ref set_target
 *
 * 3. update the environment state \ref get_environment_state
 *
 * 4. ask for a control commands by calling \ref compute_cmd
 *
 * 5. actuate the control commands through user code
 *
 * *At the termination*
 *
 * 1. call \ref close
 */
class NAVGROUND_CORE_EXPORT Behavior : virtual public HasRegister<Behavior>,
                                       protected TrackChanges {
public:
  using HasRegister<Behavior>::C;

  static const std::string type;

  /**
   * @brief      Different behavior variants for the angular
   * motion when it is no constrained by the kinematics or already specified by
   * the current target.
   *
   * They ability to apply them depends on the kinematics and on the current
   * target.
   */
  enum class Heading {
    idle,                 /**< keep the same orientation */
    target_point,         /**< turn towards the target position (if any) */
    target_angle,         /**< turn towards the target orientation (if any) */
    target_angular_speed, /**< follow a target angular speed (if any) */
    velocity /**< turn towards the velocity direction. This is the only
                behavior available to constrained kinematics.*/
  };

  /**
   * @brief      Get a named \ref Heading mode
   *
   * @param      value  The name of the \ref Heading mode
   *
   * @return     The corresponding \ref Heading
   */
  static Heading heading_from_string(const std::string &value) {
    if (value == "target_point")
      return Heading::target_point;
    if (value == "target_angle")
      return Heading::target_angle;
    if (value == "target_angular_speed")
      return Heading::target_angular_speed;
    if (value == "velocity")
      return Heading::velocity;
    return Heading::idle;
  }

  /**
   * @brief      Get the name of an \ref Heading mode
   *
   * @param[in]  value  The \ref Heading mode
   *
   * @return     The name of the \ref Heading mode.
   */
  static std::string heading_to_string(const Heading &value) {
    if (value == Heading::target_point)
      return "target_point";
    if (value == Heading::target_angle)
      return "target_angle";
    if (value == Heading::target_angular_speed)
      return "target_angular_speed";
    if (value == Heading::velocity)
      return "velocity";
    return "idle";
  }

  /**
   * Default rotation tau
   */
  static constexpr ng_float_t default_rotation_tau = 0.5;
  /**
   * Default horizon
   */
  static constexpr ng_float_t default_horizon = 5;
  /**
   * Default safety margin
   */
  static constexpr ng_float_t default_safety_margin = 0;
  /**
   * Default safety margin
   */
  static constexpr ng_float_t default_path_tau = 0.5;
  /**
   * Default safety margin
   */
  static constexpr ng_float_t default_path_look_ahead = 1.0;
  /**
   * @brief      Constructs a new instance.
   *
   * @param  kinematics  The kinematics of the agent.
   * @param  radius      The radius of the agent.
   */
  Behavior(std::shared_ptr<Kinematics> kinematics = nullptr,
           ng_float_t radius = 0)
      : TrackChanges(), social_margin(), kinematics(kinematics), radius(radius),
        pose(), twist(), horizon(default_horizon),
        safety_margin(default_safety_margin),
        optimal_speed(kinematics ? kinematics->get_max_speed() : 0),
        optimal_angular_speed(kinematics ? kinematics->get_max_angular_speed()
                                         : 0),
        rotation_tau(default_rotation_tau), path_tau(default_path_tau),
        path_look_ahead(default_path_look_ahead),
        heading_behavior(Heading::idle), assume_cmd_is_actuated(true), target(),
        modulations() {}

  virtual ~Behavior() = default;
  /**
   * @brief  Finalizes the behavior initialization.
   *
   * Call it after configuring the behavior parameters, before starting using
   * it.
   *
   * The base class implementation does nothing.
   * Override it to add any logic that to finalizes the concrete behavior before
   * using it. For instance, it can be used to load resources or configure the
   * coordination with other behaviors.
   */
  virtual void prepare() {};
  /**
   * @brief Clean-up the behavior before termination.
   *
   * Call it once the behavior is not used anymore.
   *
   * The base class implementation does nothing.
   * Override it to add any logic to clean-up steps performed in \prepare.
   */
  virtual void close() {};

  //------------ AGENT PARAMETERS

  /**
   * @brief      Gets the kinematics.
   *
   * @return     The agent's kinematics.
   */
  std::shared_ptr<Kinematics> get_kinematics() const { return kinematics; }
  /**
   * @brief      Sets the kinematics.
   *
   * @param[in]  value The desired kinematics.
   */
  void set_kinematics(std::shared_ptr<Kinematics> value) {
    if (value) {
      if (!kinematics) {
        if (optimal_speed == 0) {
          optimal_speed = value->get_max_speed();
        }
        if (optimal_angular_speed == 0) {
          optimal_angular_speed = value->get_max_angular_speed();
        }
      }
      kinematics = value;
    }
  }

  /**
   * @brief      Gets the radius of the agent.
   *
   * @return     The agent's radius.
   */
  ng_float_t get_radius() const { return radius; }
  /**
   * @brief      Sets the radius of the agent.
   *
   * @param      value  A positive value.
   */
  void set_radius(ng_float_t value) {
    radius = std::max<ng_float_t>(0, value);
    change(RADIUS);
  }
  /**
   * @brief      Gets the maximal speed.
   *
   * @return     The maximal speed returned from \ref Kinematics::get_max_speed
   */
  ng_float_t get_max_speed() const {
    return kinematics ? kinematics->get_max_speed() : 0;
  }
  /**
   * @brief      Sets the maximal speed.
   *
   * @param[in]  value  The value to pass to \ref Kinematics::set_max_speed
   */
  void set_max_speed(ng_float_t value) {
    if (kinematics)
      kinematics->set_max_speed(value);
  }

  /**
   * @brief      Gets the maximal angular speed speed.
   *
   * @return     The maximal angular speed from \ref
   * Kinematics::get_max_angular_speed
   */
  Radians get_max_angular_speed() const {
    return kinematics ? kinematics->get_max_angular_speed() : 0;
  }
  /**
   * @brief      Sets the maximal angular speed speed.
   *
   * @param[in]  value  The value to pass to \ref
   * Kinematics::set_max_angular_speed.
   */
  void set_max_angular_speed(Radians value) {
    if (kinematics)
      kinematics->set_max_angular_speed(value);
  }

  // ---------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the desired optimal speed.
   *
   * If not configured through \ref set_optimal_speed,
   * it return \ref get_max_speed.
   *
   * @return     The desired optimal speed.
   */
  ng_float_t get_optimal_speed() const { return optimal_speed; }
  /**
   * @brief      Sets the desired optimal speed.
   *
   * @param[in]  value A positive linear speed.
   */
  void set_optimal_speed(ng_float_t value) {
    optimal_speed = std::max<ng_float_t>(value, 0);
    change(OPTIMAL_SPEED);
  }

  /**
   * @brief      Clamp a speed in the range of feasible values given by the
   * kinematics.
   *
   * @param[in]  value  The desired value
   *
   * @return the nearest feasible value
   */
  ng_float_t feasible_speed(ng_float_t value) const {
    const auto max_value = get_max_speed();
    return std::clamp<ng_float_t>(value, -max_value, max_value);
  }

  /**
   * @brief      Clamp an angular speed in the range of feasible values given by
   * the kinematics.
   *
   * @param[in]  value  The desired value
   *
   * @return the nearest feasible value
   */
  ng_float_t feasible_angular_speed(ng_float_t value) const {
    const auto max_value = get_max_angular_speed();
    return std::clamp<ng_float_t>(value, -max_value, max_value);
  }

  /**
   * @brief      Clamp an twist in the range of feasible values given by the
   * kinematics.
   *
   * @param[in]  value  The desired value
   *
   * @return the nearest feasible value
   */
  Twist2 feasible_twist(const Twist2 &value) const;

  /**
   * @brief      Computes the nearest feasible twist given the
   * kinematics and the current twist over a time step
   *
   * @param[in]  value  The desired value
   * @param[in]  time_step  The time step
   *
   * @return the nearest feasible value
   */
  Twist2 feasible_twist_from_current(const Twist2 &value,
                                     ng_float_t time_step) const;

  /**
   * @brief      Gets the desired optimal angular speed.
   *
   * If not configured through \ref set_optimal_angular_speed,
   * it will return set to \ref get_max_angular_speed.
   *
   * @return     The desired optimal angular speed.
   */
  Radians get_optimal_angular_speed() const { return optimal_angular_speed; }
  /**
   * @brief      Sets the optimal angular speed.
   *
   * @param[in]  value  A positive angular speed in radians/time unit.
   */
  void set_optimal_angular_speed(Radians value) {
    optimal_angular_speed = std::max<ng_float_t>(value, 0);
  }
  /**
   * @brief      Gets the relaxation time to rotate towards a desired
   * orientation.
   *
   * The default behaviors applies a P control to rotations, e.g.,
   * \f$\omega = \frac{\delta \theta}{\tau_\textrm{rot}}\f$
   *
   * @return     The rotation relaxation time.
   */
  ng_float_t get_rotation_tau() const { return rotation_tau; }
  /**
   * @brief      Sets the relaxation time to rotate towards a desired
   * orientation. See \ref get_rotation_tau.
   *
   * @param[in]  value  The rotation relaxation time. Set it to a zero or
   * negative to rotate as fast as possible.
   */
  void set_rotation_tau(ng_float_t value) { rotation_tau = value; }

  /**
   * @brief      Gets the minimal safety margin to keep away from obstacles
   *
   * @return     The safety margin.
   */
  ng_float_t get_safety_margin() const { return safety_margin; }
  /**
   * @brief      Sets the safety margin to keep away from obstacles.
   *
   * @param[in]  value  A positive value.
   */
  void set_safety_margin(ng_float_t value) {
    safety_margin = std::max<ng_float_t>(0, value);
    change(SAFETY_MARGIN);
  }
  /**
   * @brief      Gets the horizon: the size of the portion of environment
   *             around the agent considered when computing possible collisions.
   *             Larger values generally lead to a more computationally
   * expensive \ref compute_cmd but fewer deadlocks and less jamming.
   *
   * @return     The horizon.
   */
  ng_float_t get_horizon() const { return horizon; }
  /**
   * @brief      Sets the horizon, see \ref get_horizon.
   *
   * @param[in]  value  A positive value.
   */
  void set_horizon(ng_float_t value) {
    horizon = std::max<ng_float_t>(0, value);
    change(HORIZON);
  }

  // Sub-classes may not override this method but the specialized methods
  // \ref cmd_twist_towards_target, \ref cmd_twist_towards_target_orientation,
  // and \ref cmd_cmd_twist_towards_stopping.

  /**
   * @brief      Gets whether to assume that the compute command will be
   * actuated as it is.
   *
   * If set, \ref Behavior will assume that the control command
   * computed by \ref compute_cmd be actuated, therefore setting \ref
   * set_actuated_twist to that value. If not set, the user should set \ref
   * Behavior::set_actuated_twist before querying for a new control commands:
   * some behavior use the old actuated control command to compute smoother
   * controls.
   *
   * @return     True if it assumes that the command will be actuated as it is.
   */
  bool get_assume_cmd_is_actuated() const { return assume_cmd_is_actuated; }
  /**
   * @brief      Sets whether to assume that the compute command will be
   * actuated as it is.
   * @see \ref get_assume_cmd_is_actuated.
   *
   * @param[in]  value  The desired value.
   */
  void set_assume_cmd_is_actuated(bool value) {
    assume_cmd_is_actuated = value;
  }

  /**
   * @brief      Gets the path relaxation time
   *
   * @return     The time.
   */
  ng_float_t get_path_tau() const { return path_tau; }
  /**
   * @brief      Sets the path relaxation time, see \ref get_path_tau.
   *
   * @param[in]  value  A positive value.
   */
  void set_path_tau(ng_float_t value) {
    path_tau = std::max<ng_float_t>(0, value);
  }

  /**
   * @brief      Gets the path look-ahead distance
   *
   * @return     The distance.
   */
  ng_float_t get_path_look_ahead() const { return path_look_ahead; }
  /**
   * @brief      Sets the path look-ahead distance, see \ref
   * get_path_look_ahead.
   *
   * @param[in]  value  A positive value.
   */
  void set_path_look_ahead(ng_float_t value) {
    path_look_ahead = std::max<ng_float_t>(0, value);
  }

  //----------- AGENT STATE

  /**
   * @brief      Gets the current pose in the world-fixed frame.
   *
   * @return     The current pose.
   */
  Pose2 get_pose() const { return pose; }
#if 0
  /**
   * @brief      Gets a reference to the pose in the world-fixed frame.
   *
   * @return     The pose.
   */
  Pose2 &get_pose_ref() { return pose; }
#endif
  /**
   * @brief      Sets the current pose in the world-fixed frame.
   *
   * @param[in]  value The desired value
   */
  void set_pose(const Pose2 &value) {
    pose = value;
    change(POSITION | ORIENTATION);
  }
  /**
   * @brief      Convenience method to get the current position in the
   * world-fixed frame. See \ref get_pose.
   *
   * @return     The position.
   */
  Vector2 get_position() const { return pose.position; }
  /**
   * @brief      Convenience method to set the current position in the
   * world-fixed frame. See \ref set_pose.
   *
   * @param[in]  value The desired value
   */
  void set_position(const Vector2 &value) {
    pose.position = value;
    change(POSITION);
  }
  /**
   * @brief      Convenience method to get the current orientation. See \ref
   * get_pose.
   *
   * @return     The current orientation.
   */
  Radians get_orientation() const { return pose.orientation; }
  /**
   * @brief      Convenience method to set the current orientation. See \ref
   * set_pose.
   *
   * @param[in]  value  The desired value
   */
  void set_orientation(Radians value) {
    pose.orientation = value;
    change(ORIENTATION);
  }
  /**
   * @brief      Gets the current twist.
   *
   * @param[in]  frame  The desired frame of reference.
   *
   * @return     The current twist.
   */
  Twist2 get_twist(Frame frame = Frame::absolute) const {
    return to_frame(twist, frame);
  }
#if 0
  /**
   * @brief      Gets a reference to the twist.
   *
   * @return     The twist.
   */
  Twist2 &get_twist_ref() { return twist; }
#endif
  /**
   * @brief      Sets the current twist.
   *
   * @param[in]  value The desired value
   */
  void set_twist(const Twist2 &value) {
    twist = value;
    change(VELOCITY | ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to get the current velocity. See \ref
   * get_twist
   *
   * @param[in]  frame  The desired frame of reference.
   *
   * @return     The velocity.
   */
  Vector2 get_velocity(Frame frame = Frame::absolute) const {
    return to_frame(twist, frame).velocity;
  }
  /**
   * @brief      Convenience method to set the current velocity. See \ref
   * set_twist
   *
   * @param[in]  value  The velocity
   * @param[in]  frame  The desired frame of reference.
   */
  void set_velocity(const Vector2 &value, Frame frame = Frame::absolute) {
    twist.velocity = value;
    twist.frame = frame;
    change(VELOCITY);
  }
  /**
   * @brief      Convenience method to get the current speed. See \ref get_twist
   *
   * @return     The current speed.
   */
  ng_float_t get_speed() const { return twist.velocity.norm(); }
  /**
   * @brief      Convenience method to get the current the angular speed.
   *
   * @return     The current angular speed.
   */
  Radians get_angular_speed() const { return twist.angular_speed; }
  void set_angular_speed(Radians value) {
    twist.angular_speed = value;
    change(ANGULAR_SPEED);
  }
  /**
   * @brief      Convenience method to get the current wheel speeds. See \ref
   * get_twist
   *
   * @return     The wheel speeds.
   */
  WheelSpeeds get_wheel_speeds() const {
    return wheel_speeds_from_twist(twist);
  }
  /**
   * @brief      Convenience method to set the current wheel speeds. See \ref
   * set_twist
   *
   * @param[in]  value  The wheel speeds
   */
  void set_wheel_speeds(const WheelSpeeds &value) {
    if (kinematics && kinematics->is_wheeled()) {
      WheeledKinematics *wk =
          dynamic_cast<WheeledKinematics *>(kinematics.get());
      set_twist(wk->twist(value));
    }
  }
  /**
   * @brief      Gets the last actuated twist.
   *
   * @param[in]  frame  The desired frame of reference.
   *
   * @return     The actuated twist.
   */
  Twist2 get_actuated_twist(Frame frame = Frame::absolute) const {
    return to_frame(actuated_twist, frame);
  }
#if 0
  /**
   * @brief      Gets a reference to the actuated twist.
   *
   * @return     The actuated twist.
   */
  Twist2 &get_actuated_twist_ref() { return actuated_twist; }
#endif
  /**
   * @brief      Sets the last actuated twist.
   *
   * @param[in]  value The desired value
   */
  void set_actuated_twist(const Twist2 &value) { actuated_twist = value; }
  /**
   * @brief      Convenience method to get the last actuated wheel speeds from
   * \ref actuated_twist.
   *
   * If the agent is not wheeled (\ref Kinematics::is_wheeled), an empty vector
   * is returned.
   *
   * @return     The actuated wheel speeds.
   */
  WheelSpeeds get_actuated_wheel_speeds() const {
    return wheel_speeds_from_twist(actuated_twist);
  }
  /**
   * @brief      Actuate a twist command, integrating using \ref
   * Pose2::integrate
   *
   * @param[in]  twist_cmd  The twist
   * @param[in]  time_step  The time step
   * @param[in]  enforce_feasibility  Whether to enforce that the command is
   * kinematically feasible
   */
  void actuate(const Twist2 &twist_cmd, ng_float_t time_step,
               bool enforce_feasibility = false);
  /**
   * @brief      Convenience method to actuate the stored actuated twist
   * command,
   *
   * @param[in]  time_step  The time step
   */
  void actuate(ng_float_t time_step) { actuate(actuated_twist, time_step); }

  /**
   * @brief      Clone the state of this behavior from another behavior
   *
   * @param[in]  other  The other behavior
   */
  void set_state_from(const Behavior &other);

  //----------- CONTROL

  /**
   * @brief      Gets the heading behavior:
   *
   * @return     The heading behavior.
   */
  Heading get_heading_behavior() const {
    if (kinematics && kinematics->dof() == 3) {
      return heading_behavior;
    } else {
      return Heading::velocity;
    }
  }
  /**
   * @brief      Sets the heading behavior, see \ref get_heading_behavior.
   *             When the kinematics has only 2 degrees of freedom (see \ref
   * Kinematics::dof()), the only available behavior is \ref Heading::velocity.
   *
   * @param[in]  value The desired value
   */
  void set_heading_behavior(Heading value) { heading_behavior = value; }
  /**
   * @brief      Gets the target.
   *
   * @return     The target.
   */
  Target get_target() const { return target; }
  /**
   * @brief      Gets a reference to the target.
   *
   * @return     The target.
   */
  Target &get_target_ref() { return target; }
  /**
   * @brief      Sets the target.
   *
   * @param[in]  value  The desired value
   */
  void set_target(const Target &value) {
    target = value;
    change(TARGET);
  }

  /**
   * @brief      Returns whether the current target is valid.
   */
  bool has_target() const { return target.valid(); }

  /**
   * @brief      Query the behavior to get a control command
   *
   * Before calling this method, update the state using methods such as
   * \ref set_pose, and \ref set_twist and set the target \ref set_target.
   *
   * Behaviors may use caching to speed up the next queries if the state does
   * not change.
   *
   * Modulations are applied as wrappers/context modifiers:
   * right before evaluating the behavior in \ref compute_cmd_internal,
   * it will call \ref BehaviorModulation::pre for any modulation
   * in \ref get_modulations,
   * and right after \ref BehaviorModulation::post but in reverse order.
   *
   * @param[in]  time_step   The control time step. Not all behavior use it
   * but some may use it, for example, to limit accelerations.
   * @param[in]  frame  An optional desired frame of reference for the command.
   * @param[in]  enforce_feasibility  Whether to enforce that the command is
   * kinematically feasible
   *
   * @return  The control command as a twist in the specified frame.
   */

  Twist2 compute_cmd(ng_float_t time_step,
                     std::optional<Frame> frame = std::nullopt,
                     bool enforce_feasibility = false);

#if 0
  /**
   * @brief      The most natural frame for the current kinematics:
   * \ref Frame::relative in case the agent is wheeled, else \ref
   * Frame::absolute.
   *
   * @return     The frame
   */
  Frame default_cmd_frame() {
    return kinematics ? kinematics->cmd_frame() : Frame::absolute;
  }
#endif

  /**
   * @brief      Gets the last computed desired velocity.
   *
   * @return     The desired velocity (only valid if computed) in \ref
   * Frame::absolute
   */
  Vector2 get_desired_velocity() const { return desired_velocity; }

  //----------- TRANSFORMATIONS

  /**
   * @brief      Transform a twist to \ref Frame::absolute.
   *
   * @param[in]  value  The original twist
   *
   * @return     The same twist in \ref Frame::absolute
   *             (unchanged if already in \ref Frame::absolute)
   */
  Twist2 to_absolute(const Twist2 &value) const { return value.absolute(pose); }

  /**
   * @brief      Transform a twist to \ref Frame::relative.
   *
   * @param[in]  value  The original twist
   *
   * @return     The same twist in \ref Frame::relative
   *             (unchanged if already in \ref Frame::relative)
   */
  Twist2 to_relative(const Twist2 &value) const { return value.relative(pose); }

  /**
   * @brief      Transform a vector (e.g., a velocity)
   *             from \ref Frame::absolute to \ref Frame::relative.
   *
   * @param[in]  value  The vector in \ref Frame::absolute
   *
   * @return     The vector in \ref Frame::relative
   */
  Vector2 to_relative(const Vector2 &value) const {
    return rotate(value, -pose.orientation);
  }

  /**
   * @brief      Convert a twist to a reference frame.
   *
   * @param[in]  value     The original twist.
   * @param[in]  frame  The desired frame of reference
   *
   * @return     The twist in the desired frame of reference.
   */
  Twist2 to_frame(const Twist2 &value, Frame frame) const {
    return frame == Frame::relative ? to_relative(value) : to_absolute(value);
  }

  /**
   * @brief      Convenience method to transform from twist to wheel speeds.
   *
   * If the agent is not wheeled (\ref Kinematics::is_wheeled), an empty vector
   * is returned.
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  WheelSpeeds wheel_speeds_from_twist(const Twist2 &value) const {
    if (kinematics && kinematics->is_wheeled()) {
      WheeledKinematics *wk =
          dynamic_cast<WheeledKinematics *>(kinematics.get());
      return wk->wheel_speeds(
          value.frame == Frame::relative ? value : to_relative(value));
    }
    return {};
  }
  /**
   * @brief       Convenience method to transform from wheel speeds to twist.
   *
   * If the agent is not wheeled (\ref Kinematics::is_wheeled), an zero twist is
   * returned.
   *
   * @param[in]  value  The wheel speeds
   *
   * @return     The corresponding twist.
   */
  Twist2 twist_from_wheel_speeds(const WheelSpeeds &value) const {
    if (kinematics && kinematics->is_wheeled()) {
      WheeledKinematics *wk =
          dynamic_cast<WheeledKinematics *>(kinematics.get());
      return wk->twist(value);
    }
    return {};
  }

  /**
   * The behavior social margin modulation
   */
  SocialMargin social_margin;

  /**
   * @brief      Gets the environment state.
   *
   * @return     The environment state.
   */
  virtual EnvironmentState *get_environment_state() { return nullptr; }

  /**
   * @brief      Check if the current target has been satisfied
   *
   * @return     True if the current target has been satisfied.
   */
  bool check_if_target_satisfied() const;

  /**
   * @brief      Estimate how much time before the target is satisfied
   *
   * @return     A positive value if the target is not yet satisfied, else 0.
   */
  ng_float_t estimate_time_until_target_satisfied() const;

  /**
   * @brief      Determines if the agent is stuck: if should move but it is
   * still.
   *
   * @return     True if stuck, False otherwise.
   */
  bool is_stuck() const;

  /**
   * @brief      Gets the efficacy: the projection of the current velocity on
   * the ideal velocity (ignoring obstacles) towards the target.
   *
   * A value of 1.0 denotes ideal efficacy, value of 0.0 that the agent is
   * stuck.
   *
   * @return     The efficacy.
   */
  ng_float_t get_efficacy() const;

  /**
   * @brief      Gets the modulations applied to this behavior
   *
   * @return     The modulations.
   */
  std::vector<std::shared_ptr<BehaviorModulation>> &get_modulations() {
    return modulations;
  }
  /**
   * @brief      Gets the modulations applied to this behavior
   *
   * @return     The modulations.
   */
  const std::vector<std::shared_ptr<BehaviorModulation>> &
  get_modulations() const {
    return modulations;
  }
  /**
   * @brief      Adds a modulation.
   *
   * @param[in]  value  The modulation to add
   */
  void add_modulation(const std::shared_ptr<BehaviorModulation> &value) {
    modulations.push_back(value);
  }

  /**
   * @brief      Removes a modulation.
   *
   * @param[in]  value  The modulation to remove
   */
  void remove_modulation(const std::shared_ptr<BehaviorModulation> &value) {
    modulations.erase(
        std::remove(modulations.begin(), modulations.end(), value),
        modulations.end());
  }

  /**
   * @brief      Removes all modulations
   */
  void clear_modulations() { modulations.clear(); }

  /**
   * @brief      Returns the target's pose composed by
   *             position \ref get_target_position
   *             and orientation \ref get_target_orientation,
   *             if both are defined, else null.
   *
   * @param[in]  frame  The desired frame
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The pose in the desired frame or null.
   */
  std::optional<Pose2> get_target_pose(Frame frame,
                                       bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the target's position if not satisfied,
   *             else null.
   *
   * @param[in]  frame  The desired frame
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The vector in the desired frame
   */
  std::optional<Vector2>
  get_target_position(Frame frame, bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the target's orientation if not satisfied,
   *             else null.
   *
   * @param[in]  frame  The desired frame
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The orientation in the desired frame
   */
  std::optional<ng_float_t>
  get_target_orientation(Frame frame, bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the direction towards
   *             the target's position if not satisfied,
   *             or the target's direction if defined, else null.
   *
   * @param[in]  frame  The desired frame
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The normalized direction in the desired frame
   */
  std::optional<Vector2>
  get_target_direction(Frame frame, bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the distance to the target point, if valid,
   *             else 0.
   *
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The distance.
   */
  ng_float_t get_target_distance(bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the distance to the target point, if valid,
   *             else null.
   *
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The distance.
   */
  std::optional<int>
  get_target_angular_direction(bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the distance to the target orientation, if valid,
   *             else 0.
   *
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The angular distance.
   */
  ng_float_t get_target_angular_distance(bool ignore_tolerance = false) const;

  /**
   * @brief      Gets the current target twist.
   *
   *             Return a twist with linear velocity \ref get_target_velocity
   *             and angular velocity \ref get_angular_target_velocity
   *
   * @param[in]  frame  The desired frame
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The twist in the desired frame.
   */
  Twist2 get_target_twist(Frame frame, bool ignore_tolerance = false) const;

  /**
   * @brief      Gets the current target velocity.
   *
   *             Multiplication of \ref get_target_direction
   *             by \ref get_target_speed.
   *             Returns a zero vector if the target direction is undefined.
   *
   * @param[in]  frame  The desired frame
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The velocity in the desired frame.
   */
  Vector2 get_target_velocity(Frame frame, bool ignore_tolerance = false) const;

  /**
   * @brief      Returns the nearest feasible speed
   *             to target's speed, if defined,
   *             else to the behavior own optimal speed.
   *
   * @return     The speed
   */
  ng_float_t get_target_speed() const;

  /**
   * @brief      Returns the nearest feasible angular speed
   *             to target's angular speed, if defined,
   *             else to the behavior own optimal angular speed.
   *
   * @return     The angular speed.
   */
  ng_float_t get_target_angular_speed() const;

  /**
   * @brief      Gets the current target angular velocity.
   *
   *             Multiplication of \ref get_target_angular_direction
   *             by \ref get_target_angular_speed.
   *             Returns zero if the target angular direction is undefined.
   *
   *             Note that this returns a signed value, while
   *             \ref get_target_angular_speed, returns the absolute value
   * instead. Moreover considers (angular) tolerances, while \ref
   * get_target_angular_speed not, just like
   *             \ref get_target_velocity vs \ref get_target_speed.
   *
   * @param[in]  ignore_tolerance  Whether to ignore the target tolerance
   *
   * @return     The angular velocity z-component.
   */
  ng_float_t get_target_angular_velocity(bool ignore_tolerance = false) const;

  /**
   * @brief      Determines if the behavior twist is small enough.
   *
   * @param[in]  epsilon_speed          The maximal speed to be at stop
   * @param[in]  epsilon_angular_speed  The maximal angular speed to be at stop
   *
   * @return     True if stopped, False otherwise.
   */
  bool is_stopped(ng_float_t epsilon_speed = 1e-6,
                  ng_float_t epsilon_angular_speed = 1e-6) const;

protected:
  enum {
    POSITION = 1 << 0,
    ORIENTATION = 1 << 1,
    VELOCITY = 1 << 2,
    ANGULAR_SPEED = 1 << 3,
    HORIZON = 1 << 4,
    OPTIMAL_SPEED = 1 << 5,
    SAFETY_MARGIN = 1 << 6,
    RADIUS = 1 << 7,
    TARGET = 1 << 8,
  };

  std::shared_ptr<Kinematics> kinematics;
  ng_float_t radius;
  Pose2 pose;
  Twist2 twist;
  Twist2 actuated_twist;
  ng_float_t horizon;
  ng_float_t safety_margin;
  ng_float_t optimal_speed;
  Radians optimal_angular_speed;
  ng_float_t rotation_tau;
  ng_float_t path_tau;
  ng_float_t path_look_ahead;
  Heading heading_behavior;
  bool assume_cmd_is_actuated;
  // Last computed desired velocity in \ref Frame::absolute
  Vector2 desired_velocity;
  Target target;
  std::vector<std::shared_ptr<BehaviorModulation>> modulations;

  bool should_stop() const;

  /**
   * @brief      Computes the control command.
   *
   *
   * This is a virtual protected that sub-classes may override
   * to define a behavior and is not part of the public API.
   * Users should call \ref compute_cmd instead.
   *
   * The base implementation checks for a valid target:
   *
   * 1. position along a path => calls \ref cmd_twist_along_path
   *
   * 2. pose          => calls \ref cmd_twist_towards_pose
   *
   * 3. position      => calls \ref cmd_twist_towards_point
   *
   * 4. orientation   => calls \ref cmd_twist_towards_orientation
   *
   * 5. velocity      => calls \ref cmd_twist_towards_velocity
   *
   * 6. angular speed => calls \ref cmd_twist_towards_angular_speed
   *
   * 7. else          => calls \ref cmd_twist_towards_stopping.
   *
   * To specialize a \ref Behavior, users may override this or
   * any of the methods listed above. They may also override the following
   * internal methods:
   *
   * - \ref desired_velocity_towards_point
   *
   * - \ref desired_velocity_towards_velocity
   *
   * - \ref twist_towards_velocity
   *
   * The command should be kinematically feasible.
   * The base implementation does not guaranteed it,
   * delegating the check to the methods listed above.
   *
   * @param[in]  time_step  The time step
   *
   * @return     The command in the desired frame.
   */
  virtual Twist2 compute_cmd_internal(ng_float_t time_step);

  /**
   * @brief      Compute a command to follow a path
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation uses a carrot planner to computes a target
   * velocity, and then calls \ref cmd_twist_towards_velocity.
   *
   * @param      path       The desired path in world-fixed frame
   * @param[in]  speed      The desired speed
   * @param[in]  time_step  The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_along_path(Path &path, ng_float_t speed,
                                      ng_float_t time_step);

  /**
   * @brief      Computes a command to move towards a desired pose
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation ignores the orientation, calling
   * call \ref cmd_twist_towards_point, to first reach the desired position
   * and then rotate in place to reach the desired orientation.
   *
   * @param[in]  pose           The desired pose in world-fixed frame
   * @param[in]  speed          The desired speed
   * @param[in]  angular_speed  The desired angular speed
   * @param[in]  time_step      The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_towards_pose(const Pose2 &pose, ng_float_t speed,
                                        Radians angular_speed,
                                        ng_float_t time_step);
  /**
   * @brief      Computes a command to move towards a desired position
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation:
   *
   * 1. calls \ref desired_velocity_towards_point to compute a desired velocity
   *
   * 2. calls \ref twist_towards_velocity to compute a desired twist
   *
   * 3. returns the nearest feasible twist.
   *
   * @param[in]  point      The desired position in world-fixed frame
   * @param[in]  speed      The desired speed
   * @param[in]  time_step  The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_towards_point(const Vector2 &point, ng_float_t speed,
                                         ng_float_t time_step);
  /**
   * @brief      Computes a command to turn towards a desired orientation
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation:
   *
   * 1. calls \ref desired_velocity_towards_velocity to compute a desired
   * velocity
   *
   * 2. calls \ref twist_towards_velocity to compute a desired twist
   *
   * 3. returns the nearest feasible twist.
   *
   * @param[in]  velocity   The velocity in world-fixed frame
   * @param[in]  time_step  The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_towards_velocity(const Vector2 &velocity,
                                            ng_float_t time_step);
  /**
   * @brief      Computes a command to turn towards a desired orientation
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation:
   *
   * 1. computes a desired angular speed to rotate in \ref get_rotation_tau time
   *    towards the desired orientation
   *
   * 2. calls \ref cmd_twist_towards_angular_speed
   *
   * @param[in]  orientation    The desired orientation in world-fixed frame
   * @param[in]  angular_speed  The desired angular speed
   * @param[in]  time_step      The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_towards_orientation(Radians orientation,
                                               ng_float_t angular_speed,
                                               ng_float_t time_step);
  /**
   * @brief      Computes a command to turn at a desired angular speed
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation returns the nearest feasible twist to a twist
   * with null velocity and desired angular speed.
   *
   * @param[in]  angular_speed  The desired angular speed
   * @param[in]  time_step      The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_towards_angular_speed(ng_float_t angular_speed,
                                                 ng_float_t time_step);
  /**
   * @brief      Computes a command to stop
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation returns the null twist.
   *
   * @param[in]  time_step  The time step
   *
   * @return     The command twist.
   */
  virtual Twist2 cmd_twist_towards_stopping(ng_float_t time_step);

  /**
   * @brief      Computes a control velocity towards a desired position
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation returns a null vector.
   *
   * @param[in]  point      The desired point in world-fixed frame
   * @param[in]  speed      The desired speed
   * @param[in]  time_step  The time step
   *
   * @return     A velocity in world-fixed frame
   */
  virtual Vector2 desired_velocity_towards_point(const Vector2 &point,
                                                 ng_float_t speed,
                                                 ng_float_t time_step);
  /**
   * @brief      Computes a control velocity towards a desired velocity
   *
   * Users may override it to specialize a \ref Behavior.
   *
   * The base implementation returns a null vector.
   *
   * @param[in]  velocity   The desired velocity in world-fixed frame
   * @param[in]  time_step  The time step
   *
   * @return     A velocity in world-fixed frame
   */
  virtual Vector2 desired_velocity_towards_velocity(const Vector2 &velocity,
                                                    ng_float_t time_step);
  /**
   * @brief      Computes a twist to reach a control velocity
   *
   * This method assumes that the control velocity is safe.
   * Use it to convert a velocity to a twist once you have computed (using,
   * e.g.,
   * \ref desired_velocity_towards_point or \ref
   * desired_velocity_towards_velocity) a safe control velocity.
   *
   * @param[in]  velocity  The control velocity in world-fixed frame
   *
   * @return     A command twist.
   */
  virtual Twist2 twist_towards_velocity(const Vector2 &velocity);
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_H_
