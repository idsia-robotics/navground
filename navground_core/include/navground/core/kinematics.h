#ifndef NAVGROUND_CORE_KINEMATICS_H_
#define NAVGROUND_CORE_KINEMATICS_H_

#include <algorithm>
#include <limits>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/export.h"
#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief      Abstract Kinematics type.
 *
 * A kinematics is used to
 *
 * - validated twist in the agent's frame as feasible
 *
 * - convert between wheel speeds and body twist
 *
 * - returns maximal linear and angular speed
 *
 * - returns the number of degrees of freedom
 *
 * Negative speed means unconstrained.
 */
class NAVGROUND_CORE_EXPORT Kinematics
    : virtual public HasRegister<Kinematics> {
public:
  using HasRegister<Kinematics>::C;

  /**
   * We use infinite (or negative) to mark unconstrained values.
   */
  static constexpr ng_float_t inf = std::numeric_limits<ng_float_t>::infinity();

  Kinematics(ng_float_t max_speed = Kinematics::inf,
             float max_angular_speed = Kinematics::inf)
      : _max_speed(max_speed < 0 ? Kinematics::inf : max_speed),
        _max_angular_speed(max_angular_speed < 0 ? Kinematics::inf
                                                 : max_angular_speed) {}

  virtual ~Kinematics() = default;

  /**
   * @brief      The most natural frame for this kinematics:
   * \ref Frame::relative in case the agent is wheeled, else \ref
   * Frame::absolute.
   *
   * @return     The frame
   */
  // Frame cmd_frame() const {
  //   if (is_wheeled() || dof() < 3) {
  //     return Frame::relative;
  //   }
  //   return Frame::absolute;
  // }

  /**
   * @brief      Computes the nearest feasible twist to a desired twist.
   *
   * @param[in]  twist  The desired twist
   *
   * @return     The same desired twist if feasible else the nearest feasible
   * value. How this is defined depends on the concrete sub-class.
   */
  virtual Twist2 feasible(const Twist2 &twist) const = 0;

  /**
   * @brief      Computes the nearest feasible twist to a desired twist,
   *             taking into account dynamic constraints
   *
   * @param[in]  twist     The desired twist
   * @param[in]  current   The current twist
   * @param[in]  time_step The time step to reach the desired twist
   *
   * @return     The same desired twist if feasible else the nearest feasible
   * value. How this is defined depends on the concrete sub-class.
   */
  virtual Twist2
  feasible_from_current(const Twist2 &twist,
                        [[maybe_unused]] const Twist2 &current,
                        [[maybe_unused]] ng_float_t time_step) const {
    return feasible(twist);
  }

  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     True if wheeled, False otherwise.
   */
  virtual bool is_wheeled() const { return false; }
  /**
   * @brief      Returns the degrees of freedom (between 0 and 3 for planar
   * rigid body kinematics)
   *
   * @return     The number of degrees of freedom.
   */
  virtual unsigned dof() const = 0;
  /**
   * @brief      Gets the maximal speed.
   *
   * @return     The maximal speed.
   */
  virtual ng_float_t get_max_speed() const { return _max_speed; }
  /**
   * @brief      Sets the maximum speed.
   *
   * @param[in]  value  The desired value.
   *                    A negative number is interpreted as +infinite.
   */
  void set_max_speed(ng_float_t value) {
    if (value < 0) {
      _max_speed = Kinematics::inf;
    } else {
      _max_speed = value;
    }
  }
  /**
   * @brief      Gets the maximal angular speed.
   *
   * @return     The maximal angular speed.
   */
  virtual ng_float_t get_max_angular_speed() const {
    return _max_angular_speed;
  }
  /**
   * @brief      Sets the maximum angular speed.
   *
   * @param[in]  value  The desired value.
   *                    A negative number is interpreted as +infinite.
   */
  void set_max_angular_speed(ng_float_t value) {
    if (value < 0) {
      _max_angular_speed = Kinematics::inf;
    } else {
      _max_angular_speed = value;
    }
  }

private:
  /**
   * The maximal speed
   */
  ng_float_t _max_speed;
  /**
   * The maximal angular speed
   */
  ng_float_t _max_angular_speed;
};

/**
 * @brief      Unconstrained kinematics (e.g., quad-copters)
 */
class NAVGROUND_CORE_EXPORT OmnidirectionalKinematics : public Kinematics {
public:
  static const std::string type;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  explicit OmnidirectionalKinematics(
      ng_float_t max_speed = Kinematics::inf,
      ng_float_t max_angular_speed = Kinematics::inf)
      : Kinematics(max_speed, max_angular_speed) {}

  /**
   * @private
   */
  Twist2 feasible(const Twist2 &twist) const override;
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     3
   */
  unsigned dof() const override { return 3; }
};

/**
 * @brief      Kinematics for non-wheeled agents that head towards where they
 * move (e.g., people)
 */
class NAVGROUND_CORE_EXPORT AheadKinematics : public Kinematics {
public:
  static const std::string type;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  explicit AheadKinematics(ng_float_t max_speed = Kinematics::inf,
                           ng_float_t max_angular_speed = Kinematics::inf)
      : Kinematics(max_speed, max_angular_speed) {}

  /**
   * @private
   */
  Twist2 feasible(const Twist2 &twist) const override;
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     2
   */
  unsigned dof() const override { return 2; }
};

/**
 * @brief      Abstract class for kinematics that have wheels 
 * \a and are completely determined when the speed of each wheel 
 * is known. 
 *
 */
class NAVGROUND_CORE_EXPORT WheeledKinematics : virtual public Kinematics {
public:
  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     True
   */
  bool is_wheeled() const override { return true; }
  /**
   * @brief      Convert wheel speeds to a twist
   *
   * @param[in]  value  The wheel speeds
   *
   * @return     The corresponding twist
   */
  virtual Twist2 twist(const WheelSpeeds &value) const = 0;
  /**
   * @brief      Convert a twist to wheel speeds
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  virtual WheelSpeeds wheel_speeds(const Twist2 &value) const = 0;
  /**
   * @brief      Convert a twist to feasible wheel speeds
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  virtual WheelSpeeds feasible_wheel_speeds(const Twist2 &value) const {
    return wheel_speeds(feasible(value));
  }
};

/**
 * @brief   Two differential drive wheels (left, right) (e.g., a wheelchair)
 * with a physical maximal wheel speed, and software-limited
forward/backwards speeds.
 *
 * It implements two types of kinematics constrains:
 *
 * - a maximal wheel speed that adds a linear coupling between linear and
 * angular speed (e.g., the robot can move at maximal linear speed only
 * straights) (same as \ref TwoWheelsDifferentialDriveKinematics)
 * - a controller that clamps linear and angular speed inside a box: these
 * constrains when the related bounds are set to negative values.
 *
 * *Registered properties*:
 *
 *   - `max_forward_speed` (float, \ref get_max_forward_speed)
 *
 *   - `max_backward_speed` (float, \ref get_max_backward_speed)
 *
 *   - `wheel_axis` (float, \ref get_wheel_axis)
 */
class NAVGROUND_CORE_EXPORT TwoWheelsDifferentialDriveKinematics
    : virtual public Kinematics,
      public WheeledKinematics {
public:
  static const std::string type;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal wheel speed
   * @param[in]  wheel_axis               The wheel axis (i.e., the distance
   * between the wheels)
   * @param[in]  max_angular_speed  The maximal angular speed, that is also
   * limited by the 2 * max_speed / axis, if the axis is positive.
   * @param[in]  max_forward_speed  The maximal linear speed when moving
   * forwards (set to negative or infinite to leave unconstrained)
   * @param[in]  max_backward_speed The maximal linear speed when moving
   * backwards (set to negative or infinite to leave unconstrained)
   */
  explicit TwoWheelsDifferentialDriveKinematics(
      ng_float_t max_speed = Kinematics::inf, ng_float_t wheel_axis = 1,
      ng_float_t max_angular_speed = Kinematics::inf,
      ng_float_t max_forward_speed = Kinematics::inf,
      ng_float_t max_backward_speed = 0)
      : Kinematics(max_speed, max_angular_speed), _axis(wheel_axis),
        _max_forward_speed(max_forward_speed),
        _max_backward_speed(max_backward_speed) {}

  static constexpr unsigned DOF = 2;

  /**
   * @brief      Gets the wheel axis.
   *
   * @return     The axis.
   */
  ng_float_t get_wheel_axis() const { return _axis; }
  /**
   * @brief      Sets the wheel axis.
   *
   * @param[in]  value  A positive value
   */
  void set_wheel_axis(ng_float_t value) {
    if (value > 0)
      _axis = value;
  }
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     2
   */
  unsigned dof() const override { return DOF; }

  /**
   * @private
   */
  ng_float_t get_max_angular_speed() const override {
    ng_float_t value = Kinematics::inf;
    if (get_wheel_axis() > 0) {
      value = 2 * get_max_speed() / get_wheel_axis();
    }
    return std::min(value, Kinematics::get_max_angular_speed());
  }

  /**
   * @private
   */
  ng_float_t get_max_speed() const override {
    return std::min(std::max(_max_backward_speed, _max_forward_speed),
                    Kinematics::get_max_speed());
  }

  /**
   * @brief      Whether the agent can move backwards.
   *
   * @return     True if moving backwards is possible.
   */
  bool can_move_backwards() const {
    return Kinematics::get_max_speed() > 0 && _max_backward_speed > 0;
  }
  /**
   * @brief      Whether the agent can move forwards.
   *
   * @return     True if moving forwards is possible.
   */
  bool can_move_forwards() const {
    return Kinematics::get_max_speed() > 0 && _max_forward_speed > 0;
  }

  /**
   * @brief      Gets the maximal linear speed when moving forwards.
   *
   * @return     The maximal speed.
   */
  ng_float_t get_max_forward_speed() const { return _max_forward_speed; }
  /**
   * @brief      Sets the maximal speed when moving forwards.
   *
   * @param[in]  value  The desired value.
   *                    A negative number is interpreted as +infinite.
   */
  void set_max_forward_speed(ng_float_t value) {
    if (value < 0) {
      _max_forward_speed = Kinematics::inf;
    } else {
      _max_forward_speed = value;
    }
  }

  /**
   * @brief      Gets the maximal linear speed when moving backwards.
   *
   * @return     The maximal speed.
   */
  ng_float_t get_max_backward_speed() const { return _max_backward_speed; }
  /**
   * @brief      Sets the maximal speed when moving backwards.
   *
   * @param[in]  value  The desired value.
   *                    A negative number is interpreted as +infinite.
   */
  void set_max_backward_speed(ng_float_t value) {
    if (value < 0) {
      _max_backward_speed = Kinematics::inf;
    } else {
      _max_backward_speed = value;
    }
  }

  /**
   * @brief      See \ref WheeledKinematics::twist.
   *
   * @param[in]  speeds  The wheel speeds in the order {left, right}
   *
   * @return     The corresponding twist
   */
  Twist2 twist(const WheelSpeeds &speeds) const override;

  /**
   * @brief      See \ref WheeledKinematics::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {left, right}
   */
  WheelSpeeds wheel_speeds(const Twist2 &twist) const override;

  /** @private
   */
  Twist2 feasible(const Twist2 &twist) const override;

private:
  ng_float_t _axis;
  ng_float_t _max_forward_speed;
  ng_float_t _max_backward_speed;
};

/**
 * @brief   Two differential drive wheels (left, right) (e.g., a wheelchair)
 *          with acceleration limits due to limited motor torque.
 *
 * The two motors have the same maximal torque.
 *
 * *Registered properties*:
 *
 *   - `wheel_axis` (float, \ref
 * TwoWheelsDifferentialDriveKinematics::get_wheel_axis)
 *
 *   - `max_forward_speed` (float, \ref
 * TwoWheelsDifferentialDriveKinematics::get_max_forward_speed)
 *
 *   - `max_backward_speed` (float, \ref
 * TwoWheelsDifferentialDriveKinematics::get_max_backward_speed)
 *
 *   - `max_acceleration` (float, \ref get_max_acceleration)
 *
 *   - `moi` (float, \ref get_moi, 1.0 by default)
 */
class NAVGROUND_CORE_EXPORT DynamicTwoWheelsDifferentialDriveKinematics
    : public TwoWheelsDifferentialDriveKinematics {
public:
  static const std::string type;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed  The maximal wheel speed
   * @param[in]  wheel_axis  The wheel axis (i.e., the distance between the
   * wheels)
   * @param[in]  max_angular_speed  The maximal angular speed, that is also
   * limited by the 2 * max_speed / axis, if the axis is positive.
   * @param[in]  max_forward_speed  The maximal linear speed when moving
   * forwards (set to negative or infinite to leave unconstrained)
   * @param[in]  max_backward_speed The maximal linear speed when moving
   * backwards (set to negative or infinite to leave unconstrained)
   * @param[in]  max_acceleration The maximal linear body acceleration
   * @param[in]  moi The scaled moment of inertial (``moi = I / (mass * axis^2 /
   * 8)``) Equal to 1 for an homogeneous disc of diameter ``wheel_axis``.
   */
  DynamicTwoWheelsDifferentialDriveKinematics(
      ng_float_t max_speed = Kinematics::inf, ng_float_t wheel_axis = 1,
      ng_float_t max_angular_speed = Kinematics::inf,
      ng_float_t max_forward_speed = Kinematics::inf,
      ng_float_t max_backward_speed = 0,
      ng_float_t max_acceleration = Kinematics::inf, ng_float_t moi = 1)
      : TwoWheelsDifferentialDriveKinematics(
            max_speed, wheel_axis, max_angular_speed, max_forward_speed,
            max_backward_speed),
        max_acceleration(max_acceleration), moi(moi) {}

  using TwoWheelsDifferentialDriveKinematics::feasible;

  /**
   * @private
   */
  Twist2 feasible_from_current(const Twist2 &twist, const Twist2 &current,
                               ng_float_t time_step) const override;

  /**
   * @brief      Gets the scaled moment of inertial
   *
   * It is equal to 1 for an homogeneous of disc of diameter \ref
   * TwoWheelsDifferentialDriveKinematics::get_wheel_axis.
   * Lower for when weight is shifted towards the center.
   *
   * @return     A positive value
   */
  ng_float_t get_moi() const { return moi; }
  /**
   * @brief      Sets the scaled moment of inertial
   *
   * It is equal to for an homogeneous of disc of diameter
   * \ref TwoWheelsDifferentialDriveKinematics::get_wheel_axis.
   * Lower for when weight is shifted towards the center.
   *
   * @param[in]  value  A positive value
   */
  void set_moi(ng_float_t value) {
    if (value > 0)
      moi = value;
  }

  /**
   * @brief      Gets the maximal (body) acceleration.
   *
   * @return     The acceleration.
   */
  ng_float_t get_max_acceleration() const { return max_acceleration; }
  /**
   * @brief      Sets the maximal (body) acceleration.
   *
   * @param[in]  value  A positive value
   */
  void set_max_acceleration(ng_float_t value) {
    if (value > 0)
      max_acceleration = value;
  }

  /**
   * @brief      Gets the maximal (body) angular acceleration.
   *
   * @return     The angular acceleration.
   */
  ng_float_t get_max_angular_acceleration() const;
  /**
   * @brief      Sets the maximal (body) angular acceleration.
   *
   * @param[in]  value  A positive value
   */
  void set_max_angular_acceleration(ng_float_t value);

  /**
   * @brief      Computes the wheel torques required to accelerate over a time
   * step
   *
   * @param[in]  value      The target value
   * @param[in]  current    The current value
   * @param[in]  time_step  The time step
   *
   * @return     {left, right} wheel torques. May not be feasible
   */
  std::vector<ng_float_t> wheel_torques(const Twist2 &value,
                                        const Twist2 &current,
                                        ng_float_t time_step) const;

  /**
   * @brief      Applies wheel torques to accelerate a twist over a time step
   *
   * \warning Does not check whether motor torques are feasible.
   *
   * @param[in]  values     The motor torques
   * @param[in]  current    The current twist
   * @param[in]  time_step  The time step
   *
   * @return     The accelerated twist.
   */
  Twist2 twist_from_wheel_torques(const std::vector<ng_float_t> &values,
                                  const Twist2 &current,
                                  ng_float_t time_step) const;

  /**
   * @brief      Gets the maximal [scaled] wheel torque.
   *
   * @return     The maximal wheel torque (in acceleration units)
   */
  ng_float_t get_max_wheel_torque() const { return max_acceleration; }

private:
  ng_float_t max_acceleration;
  ng_float_t moi;
};

// TODO(Jerome): make it general

/**
 * @brief      Four Omni-differential wheels (e.g., a Robomaster)
 *
 * \warning We assume that the distance between front and back wheel centers is
 * the same as the lateral distance.
 */
class NAVGROUND_CORE_EXPORT FourWheelsOmniDriveKinematics
    : virtual public Kinematics,
      public WheeledKinematics {
public:
  static const std::string type;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed   The maximal wheel speed
   * @param[in]  wheel_axis  The wheel axis (i.e., the distance between the
   * wheels)
   */
  explicit FourWheelsOmniDriveKinematics(ng_float_t max_speed = Kinematics::inf,
                                         ng_float_t wheel_axis = 0)
      : Kinematics(max_speed, Kinematics::inf), _axis(wheel_axis) {}

  /**
   * @brief      Gets the wheel axis.
   *
   * @return     The axis.
   */
  ng_float_t get_wheel_axis() const { return _axis; }
  /**
   * @brief      Sets the wheel axis.
   *
   * @param[in]  value  A positive value
   */
  void set_wheel_axis(ng_float_t value) {
    if (value > 0)
      _axis = value;
  }

  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     3
   */
  unsigned dof() const override { return 3; }

  /**
   * @private
   */
  ng_float_t get_max_angular_speed() const override {
    if (get_wheel_axis() > 0) {
      return get_max_speed() / get_wheel_axis();
    }
    return 0;
  }

  /**
   * @brief      See \ref WheeledKinematics::twist.
   *
   * @param[in]  speeds  The wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   *
   * @return     The corresponding twist
   */
  Twist2 twist(const WheelSpeeds &speeds) const override;

  /**
   * @brief      See \ref WheeledKinematics::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   */
  WheelSpeeds wheel_speeds(const Twist2 &twist) const override;

  /**
   * @brief      See \ref WheeledKinematics::feasible_wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   */
  WheelSpeeds feasible_wheel_speeds(const Twist2 &twist) const override;

  /** @private
   */
  Twist2 feasible(const Twist2 &twist) const override;

private:
  ng_float_t _axis;
};

/**
 * @brief      Bicycle kinematics.
 *
 * Given linear and angular desired speeds
 * \f$(v_{\mathrm{des}}, \omega_{\mathrm{des}})\f$,
 * it computes linear and angular speeds that respect
 * the upper bounds of the steering angle (\f$|\alpha| \le \alpha_{\max}\f$)
 * so that \f$\omega = v \frac{\tan \alpha}{l}\f$.
 *
 * When \f$(v_{\mathrm{des}}, \omega_{\mathrm{des}})\f$ lie outside of the
 * feasible set, i.e.
 * \f$|\omega_{\mathrm{des}}| > |v_{\mathrm{des}}| \frac{\tan \alpha_{\max}}{l}\f$,
 * there are two simple ways to compute a feasible pair:
 *
 * 1. reduce \f$|\omega_{\mathrm{des}}|\f$, while keeping the same sign for
 * \f$\omega_{\mathrm{des}}\f$ and the same value for \f$v_{\mathrm{des}}\f$,
 *
 * 2. reduce \f$|v_{\mathrm{des}}|\f$, while keeping the same sign for
 * \f$v_{\mathrm{des}}\f$  and the same value for \f$\omega_{\mathrm{des}}\f$.
 *
 * Parameter \ref get_k control the computation:
 *
 * - ``k==0``: uses #1,
 * 
 * - ``k==1``: uses #2,
 * 
 * - ``0 < k < 1``: interpolates between the two methods.
 *
 * Moreover, parameter \ref get_use_velocity_norm can be used to ignore the
 * direction of the desired velocity but just consider its norm as desired
 * linear speed.
 *
 * \note This is \a not a subclass of \ref WheeledKinematics. Although it has
 * wheels, it also has a steering, therefore it is not completely determined by
 * wheel speeds.
 *
 * *Registered properties*:
 *
 *   - `max_backward_speed` (float, \ref get_max_backward_speed)
 *
 *   - `axis` (float, \ref get_axis)
 *
 *   - `max_steering_angle` (float, \ref get_max_steering_angle)
 *
 *   - `k` (float, \ref get_k)
 *
 *   - `use_velocity_norm` (float, \ref get_use_velocity_norm)
 *   
 */
class NAVGROUND_CORE_EXPORT BicycleKinematics : virtual public Kinematics {
public:
  static const std::string type;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed           The maximal forwards speed
   * @param[in]  max_backward_speed  The maximal backward speed
   * @param[in]  axis                The distance between front and back wheels.
   * @param[in]  max_steering_angle  Maximal steering angle.
   * @param[in]  k                   k.
   * @param[in]  use_velocity_norm   Wheter to use the velocity norm instead
   * of the projection on the x-axis.
   */
  explicit BicycleKinematics(ng_float_t max_speed = Kinematics::inf,
                             ng_float_t max_backward_speed = Kinematics::inf,
                             ng_float_t axis = 1,
                             ng_float_t max_steering_angle = 1,
                             ng_float_t k = 0, bool use_velocity_norm = false)
      : Kinematics(max_speed, Kinematics::inf),
        _max_backward_speed(max_backward_speed), _axis(axis),
        _max_steering_angle(max_steering_angle), _k(k),
        _use_velocity_norm(use_velocity_norm) {}
  /**
   * @brief      Gets distance between front and back wheels.
   *
   * @return     The axis.
   */
  ng_float_t get_axis() const { return _axis; }
  /**
   * @brief      Sets distance between front and back wheels.
   *
   * @param[in]  value  A positive value
   */
  void set_axis(ng_float_t value) {
    if (value > 0)
      _axis = value;
  }
  /**
   * @brief      Gets the maximal backward speed.
   *
   * @return     The axis.
   */
  ng_float_t get_max_backward_speed() const { 
    return std::min(_max_backward_speed, get_max_speed()); 
  }
  /**
   * @brief      Sets the maximal backward speed.
   *
   * @param[in]  value  A positive value
   */
  void set_max_backward_speed(ng_float_t value) {
    if (value < 0) {
      _max_backward_speed = Kinematics::inf;
    } else {
      _max_backward_speed = value;
    }
  }
  /**
   * @brief      Gets the maximal steering angle.
   *
   * @return     The angle in radians.
   */
  ng_float_t get_max_steering_angle() const { return _max_steering_angle; }

  /**
   * @brief      Sets the maximal steering angle.
   *
   * @param[in]  value  A positive value in radians
   */
  void set_max_steering_angle(ng_float_t value) {
    if (value > 0)
      _max_steering_angle = value;
  }
  /**
   * @brief      Gets k.
   *
   * @return     k.
   */
  ng_float_t get_k() const { return _k; }

  /**
   * @brief      Gets whether to use the velocity norm instead
   * of the projection on the x-axis.
   *
   * @return     True if is uses the velocity norm, False if it uses the
   * projection.
   */
  bool get_use_velocity_norm() const { return _use_velocity_norm; }
  /**
   * @brief      Sets whether to use the velocity norm instead
   * of the projection on the x-axis.
   *
   * @param[in]  value  The desired value
   */
  void set_use_velocity_norm(bool value) { _use_velocity_norm = value; }

  /**
   * @brief      Sets k.
   *
   * @param[in]  value  A value in [0, 1]
   */
  void set_k(ng_float_t value) { _k = std::clamp<ng_float_t>(value, 0, 1); }
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     2
   */
  unsigned dof() const override { return 2; }
  /**
   * @private
   */
  ng_float_t get_max_angular_speed() const override {
    return get_max_speed() / get_min_steering_radius();
  }
  /**
   * @brief      Returns the nearest feasible steering angle
   *
   * @param[in]  twist  The twist
   *
   * @return     The angle in radians.
   */
  ng_float_t feasible_steering_angle(const Twist2 &twist) const;
  /** @private
   */
  Twist2 feasible(const Twist2 &twist) const override;
  /**
   * @brief      Gets the minimal steering radius.
   *
   * @return     The minimal steering radius.
   */
  ng_float_t get_min_steering_radius() const;

  /**
   * @brief      Computes a twist from linear speed and steering angle.
   *
   * Linear speed and steering angle are clipped in the feasible range.
   *
   * @param[in]  linear_speed    The linear speed
   * @param[in]  steering_angle  The steering angle
   *
   * @return     The twist in relative frame.
   */
  Twist2 twist_from_steering(ng_float_t linear_speed,
                             ng_float_t steering_angle) const;

private:
  ng_float_t _max_backward_speed;
  ng_float_t _axis;
  ng_float_t _max_steering_angle;
  ng_float_t _k;
  ng_float_t _use_velocity_norm;
};

} // namespace navground::core

#endif /* end of include guard: NAVGROUND_CORE_KINEMATICS_H_ */
