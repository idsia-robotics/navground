#ifndef NAVGROUND_CORE_KINEMATICS_H_
#define NAVGROUND_CORE_KINEMATICS_H_

#include <assert.h>

#include <algorithm>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/core/register.h"
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
class NAVGROUND_CORE_EXPORT Kinematics : virtual public HasProperties,
                                        virtual public HasRegister<Kinematics> {
 public:
  using HasRegister<Kinematics>::C;

  Kinematics(float max_speed, float max_angular_speed = 0.0)
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
  float get_max_speed() const { return max_speed; }
  /**
   * @brief      Sets the maximum speed.
   *
   * @param[in]  value  A positive value.
   */
  void set_max_speed(float value) { max_speed = std::max(0.0f, value); }
  /**
   * @brief      Gets the maximal angular speed.
   *
   * @return     The maximal angular speed.
   */
  virtual float get_max_angular_speed() const { return max_angular_speed; }
  /**
   * @brief      Sets the maximum angular speed.
   *
   * @param[in]  value  A positive value.
   */
  void set_max_angular_speed(float value) {
    max_angular_speed = std::max(0.0f, value);
  }

 private:
  /**
   * The maximal speed
   */
  float max_speed;
  /**
   * The maximal angular speed
   */
  float max_angular_speed;
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
  OmnidirectionalKinematics(float max_speed = 0.0f,
                            float max_angular_speed = 0.0f)
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
  inline static std::string type =
      register_type<OmnidirectionalKinematics>("Omni");
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
  AheadKinematics(float max_speed = 0.0f, float max_angular_speed = 0.0f)
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
  inline static std::string type = register_type<AheadKinematics>("Ahead");
};

/**
 * @brief      Abstract wheeled kinematics
 *
 * *Properties*: wheel_axis (float)
 */
class NAVGROUND_CORE_EXPORT WheeledKinematics : public Kinematics {
 public:
  WheeledKinematics(float max_speed, float max_angular_speed, float axis)
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
   * @private
   */
  Twist2 feasible(const Twist2& value) const override;

  /**
   * @brief      Gets the wheel axis.
   *
   * @return     The axis.
   */
  float get_axis() const { return axis; }
  /**
   * @brief      Sets the wheel axis.
   *
   * @param[in]  value  A positive value
   */
  void set_axis(float value) {
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
  static inline std::map<std::string, Property> properties = Properties{
      {"wheel_axis", make_property<float, WheeledKinematics>(
                         &WheeledKinematics::get_axis,
                         &WheeledKinematics::set_axis, 0.0f, "Wheel Axis")},
  };

 protected:
  float axis;
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
  TwoWheelsDifferentialDriveKinematics(float max_speed = 0.0f,
                                       float axis = 0.0f)
      : WheeledKinematics(max_speed, (axis > 0) ? 2 * max_speed / axis : 0.0f,
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
  float get_max_angular_speed() const override {
    if (get_axis() > 0) {
      return 2 * get_max_speed() / get_axis();
    }
    return 0.0f;
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
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  inline static std::string type =
      register_type<TwoWheelsDifferentialDriveKinematics>("2WDiff");
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
  FourWheelsOmniDriveKinematics(float max_speed = 0.0f, float axis = 0.0f)
      : WheeledKinematics(max_speed, axis > 0 ? max_speed / axis : 0.0f, axis) {
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
  float get_max_angular_speed() const override {
    if (get_axis() > 0) {
      return get_max_speed() / get_axis();
    }
    return 0.0f;
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
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  inline static std::string name =
      register_type<FourWheelsOmniDriveKinematics>("4WOmni");
};

}  // namespace navground::core

#endif /* end of include guard: navground_KINEMATICS_H_ */
