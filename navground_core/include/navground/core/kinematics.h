#ifndef NAVGROUND_CORE_KINEMATICS_H_
#define NAVGROUND_CORE_KINEMATICS_H_

#include <assert.h>

#include <algorithm>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/types.h"
#include "navground_core_export.h"

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
 * - store maximal linear and angular speed
 *
 * - store the number of degrees of freedom
 */
class NAVGROUND_CORE_EXPORT Kinematics
    : virtual public HasProperties,
      virtual public HasRegister<Kinematics> {
 public:
  using HasRegister<Kinematics>::C;

  Kinematics(ng_float_t max_speed, float max_angular_speed = 0)
      : max_speed(max_speed), max_angular_speed(max_angular_speed) {}

  virtual ~Kinematics() = default;

  /**
   * @brief      The most natural frame for this kinematics:
   * \ref Frame::relative in case the agent is wheeled, else \ref
   * Frame::absolute.
   *
   * @return     The frame
   */
  Frame cmd_frame() const {
    if (is_wheeled() || dof() < 3) {
      return Frame::relative;
    }
    return Frame::absolute;
  }

  /**
   * @brief      Computes the nearest feasible twist to a desired twist.
   *
   * @param[in]  twist  The desired twist
   *
   * @return     The same desired twist if feasible else the nearest feasible
   * value. How this is defined depends on the concrete sub-class.
   */
  virtual Twist2 feasible(const Twist2& twist) const = 0;

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
  virtual Twist2 feasible(const Twist2& twist,
                          [[maybe_unused]] const Twist2& current,
                          [[maybe_unused]] ng_float_t time_step) const {
    return feasible(twist);
  }

  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     True if wheeled, False otherwise.
   */
  virtual bool is_wheeled() const = 0;
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
  ng_float_t get_max_speed() const { return max_speed; }
  /**
   * @brief      Sets the maximum speed.
   *
   * @param[in]  value  A positive value.
   */
  void set_max_speed(ng_float_t value) {
    max_speed = std::max<ng_float_t>(0, value);
  }
  /**
   * @brief      Gets the maximal angular speed.
   *
   * @return     The maximal angular speed.
   */
  virtual ng_float_t get_max_angular_speed() const { return max_angular_speed; }
  /**
   * @brief      Sets the maximum angular speed.
   *
   * @param[in]  value  A positive value.
   */
  void set_max_angular_speed(ng_float_t value) {
    max_angular_speed = std::max<ng_float_t>(0, value);
  }

 private:
  /**
   * The maximal speed
   */
  ng_float_t max_speed;
  /**
   * The maximal angular speed
   */
  ng_float_t max_angular_speed;
};

/**
 * @brief      Unconstrained kinematics (e.g., quad-copters)
 */
class NAVGROUND_CORE_EXPORT OmnidirectionalKinematics : public Kinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  OmnidirectionalKinematics(ng_float_t max_speed = 0,
                            ng_float_t max_angular_speed = 0)
      : Kinematics(max_speed, max_angular_speed) {}

  /**
   * @private
   */
  Twist2 feasible(const Twist2& twist) const override;
  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     False
   */
  bool is_wheeled() const override { return false; }
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     3
   */
  unsigned dof() const override { return 3; }

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  const static std::string type;
};

/**
 * @brief      Kinematics for non-wheeled agents that head towards where they
 * move (e.g., people)
 */
class NAVGROUND_CORE_EXPORT AheadKinematics : public Kinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal speed
   * @param[in]  max_angular_speed  The maximal angular speed
   */
  AheadKinematics(ng_float_t max_speed = 0, ng_float_t max_angular_speed = 0)
      : Kinematics(max_speed, max_angular_speed) {}

  /**
   * @private
   */
  Twist2 feasible(const Twist2& twist) const override;
  /**
   * @brief      Returns whether the kinematics has wheels.
   *
   * @return     False
   */
  bool is_wheeled() const override { return false; }
  /**
   * @brief      Returns the degrees of freedom
   *
   * @return     2
   */
  unsigned dof() const override { return 2; }

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  static const std::string type;
};

/**
 * @brief      Abstract wheeled kinematics
 *
 * *Registered properties*:
 *
 *   - `wheel_axis` (float, \ref get_axis)
 */
class NAVGROUND_CORE_EXPORT WheeledKinematics : public Kinematics {
 public:
  WheeledKinematics(ng_float_t max_speed, ng_float_t max_angular_speed,
                    ng_float_t axis)
      : Kinematics(max_speed, max_angular_speed), axis(axis) {}

  virtual ~WheeledKinematics() = default;

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
  virtual Twist2 twist(const WheelSpeeds& value) const = 0;
  /**
   * @brief      Convert a twist to wheel speeds
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  virtual WheelSpeeds wheel_speeds(const Twist2& value) const = 0;

  /**
   * @brief      Convert a twist to feasible wheel speeds
   *
   * @param[in]  value  The twist
   *
   * @return     The corresponding wheel speeds.
   */
  virtual WheelSpeeds feasible_wheel_speeds(const Twist2& value) const = 0;

  /**
   * @private
   */
  Twist2 feasible(const Twist2& value) const override;

  /**
   * @brief      Gets the wheel axis.
   *
   * @return     The axis.
   */
  ng_float_t get_axis() const { return axis; }
  /**
   * @brief      Sets the wheel axis.
   *
   * @param[in]  value  A positive value
   */
  void set_axis(ng_float_t value) {
    if (value > 0) axis = value;
  }

  /**
   * @private
   */
  virtual const Properties& get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static const std::map<std::string, Property> properties;

 protected:
  ng_float_t axis;
};

/**
 * @brief   Two differential drive wheels (left, right) (e.g., a wheelchair)
 */
class NAVGROUND_CORE_EXPORT TwoWheelsDifferentialDriveKinematics
    : public WheeledKinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal wheel speed
   * @param[in]  axis               The wheel axis (i.e., the distance between
   * the wheels)
   */
  TwoWheelsDifferentialDriveKinematics(ng_float_t max_speed = 0,
                                       ng_float_t axis = 0)
      : WheeledKinematics(max_speed, (axis > 0) ? 2 * max_speed / axis : 0,
                          axis) {}

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
    if (get_axis() > 0) {
      return 2 * get_max_speed() / get_axis();
    }
    return 0;
  }

  /**
   * @brief      See \ref WheeledKinematics::twist.
   *
   * @param[in]  speeds  The wheel speeds in the order {left, right}
   *
   * @return     The corresponding twist
   */
  Twist2 twist(const WheelSpeeds& speeds) const override;

  /**
   * @brief      See \ref WheeledKinematics::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {left, right}
   */
  WheelSpeeds wheel_speeds(const Twist2& twist) const override;

  /**
   * @brief      See \ref WheeledKinematics::feasible_wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {left, right}
   */
  WheelSpeeds feasible_wheel_speeds(const Twist2& twist) const override;

  /** @private
   */
  Twist2 feasible(const Twist2& twist) const override;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  const static std::string type;
};

// TODO(Jerome): make it general

/**
 * @brief      Four Omni-differential wheels (e.g., a Robomaster)
 *
 * \warning We assume that the distance between front and back wheel centers is
 * the same as the lateral distance.
 */
class NAVGROUND_CORE_EXPORT FourWheelsOmniDriveKinematics
    : public WheeledKinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed          The maximal wheel speed
   * @param[in]  axis               The wheel axis (i.e., the distance between
   * the wheels)
   */
  FourWheelsOmniDriveKinematics(ng_float_t max_speed = 0, ng_float_t axis = 0)
      : WheeledKinematics(max_speed, axis > 0 ? max_speed / axis : 0, axis) {}

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
    if (get_axis() > 0) {
      return get_max_speed() / get_axis();
    }
    return 0;
  }

  /**
   * @brief      See \ref WheeledKinematics::twist.
   *
   * @param[in]  speeds  The wheel speeds in the order {front left, rear left,
   * rear right, rear left}
   *
   * @return     The corresponding twist
   */
  Twist2 twist(const WheelSpeeds& speeds) const override;

  /**
   * @brief      See \ref WheeledKinematics::wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   */
  WheelSpeeds wheel_speeds(const Twist2& twist) const override;

  /**
   * @brief      See \ref WheeledKinematics::feasible_wheel_speeds.
   *
   * @param[in]  twist  The twist
   *
   * @return     The corresponding wheel speeds in the order {front left, rear
   * left, rear right, rear left}
   */
  WheelSpeeds feasible_wheel_speeds(const Twist2& twist) const override;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  const static std::string type;
};

/**
 * @brief   Two differential drive wheels (left, right) (e.g., a wheelchair)
 *          with acceleration limits due to limited motor torque.
 *
 * The two motors have the same maximal torque.
 *
 * *Registered properties*:
 *
 *   - `wheel_axis` (float, \ref WheeledKinematics::get_axis)
 *
 *   - `max_acceleration` (float, \ref get_max_acceleration)
 *
 *   - `moi` (float, \ref get_moi, 1.0 by default)
 *
 *   - `reduce_torques` (bool, \ref get_reduce_torques, false by default)
 */
class NAVGROUND_CORE_EXPORT DynamicTwoWheelsDifferentialDriveKinematics
    : public TwoWheelsDifferentialDriveKinematics {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  max_speed  The maximal wheel speed
   * @param[in]  axis  The wheel axis (i.e., the distance between the wheels)
   * @param[in]  max_acceleration The maximal linear body acceleration
   * @param[in]  moi The scaled moment of inertial (``moi = I / (mass * axis^2 /
   * 8)``) Equal to one for an homogeneous disc of diameter ``axis``.
   */
  DynamicTwoWheelsDifferentialDriveKinematics(ng_float_t max_speed = 0,
                                              ng_float_t axis = 0,
                                              ng_float_t max_acceleration = 0,
                                              ng_float_t moi = 1,
                                              bool reduce_torques = false)
      : TwoWheelsDifferentialDriveKinematics(max_speed, axis),
        max_acceleration(max_acceleration),
        moi(moi),
        reduce_torques(reduce_torques) {}

  using TwoWheelsDifferentialDriveKinematics::feasible;

  /**
   * @private
   */
  Twist2 feasible(const Twist2& twist, const Twist2& current,
                  ng_float_t time_step) const override;

  /**
   * @brief      Gets the scaled moment of inertial
   *
   * Equal to 1.0 for an homogeneous of disc of diameter \ref
   * get_axis. Lower for when weight is shifted towards the center.
   *
   * @return     A positive value
   */
  ng_float_t get_moi() const { return moi; }
  /**
   * @brief      Sets the scaled moment of inertial
   *
   * Equal to 1.0 for an homogeneous of disc of diameter \ref
   * get_axis. Lower for when weight is shifted towards the center.
   *
   * @param[in]  value  A positive value
   */
  void set_moi(ng_float_t value) {
    if (value > 0) moi = value;
  }

  /**
   * @brief      Returns whether to reduce the torques proportionally
   *             instead of clipping them independently.
   *
   * @return     True if will reduce torques proportionally
   */
  bool get_reduce_torques() const { return reduce_torques; }
  /**
   * @brief      Sets whether to reduce the torques proportionally
   *             instead of clipping them independently.
   *
   * @param[in]  value  The desired value
   */
  void set_reduce_torques(bool value) { reduce_torques = value; }

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
    if (value > 0) max_acceleration = value;
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
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  const Properties& get_properties() const override { return properties; };

  /**
   * @private
   */
  static const std::map<std::string, Property> properties;

 private:
  const static std::string type;
  ng_float_t max_acceleration;
  ng_float_t moi;
  bool reduce_torques;
};

}  // namespace navground::core

#endif /* end of include guard: navground_KINEMATICS_H_ */
