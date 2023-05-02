/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_CONTROLLER_3D_H_
#define NAVGROUND_CORE_CONTROLLER_3D_H_

#include <Eigen/Core>
#include <algorithm>
#include <memory>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/controller.h"
#include "navground/core/states/geometric.h"
#include "navground_core_export.h"

namespace navground::core {

/**
 * A three-dimensional vector, see <a
 * href="https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html">Eigen</a>
 */
using Vector3 = Eigen::Vector3f;

/**
 * @brief      Three dimensional unit vector.
 *
 * @param[in]  angle The desired orientation
 *
 * @return     Vector of norm one and orientation ``angle``
 */
inline Vector3 unit3(float angle) { return {cosf(angle), sinf(angle), 0.0f}; }

// TODO: check velocity 2D vs 3D

/**
 * @brief      Three dimensional obstacles with a vertical cylindrical shape.
 */
struct Cylinder {
  /**
   * Center of the *lower* face.
   */
  Vector3 position;
  /**
   * Radius of the cylinder
   */
  float radius;
  /**
   * Height of the cylinder
   */
  float height;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  position  Center of the *lower* face
   * @param[in]  radius    Radius of the cylinder
   * @param[in]  height    Height of the cylinder
   */
  Cylinder(const Vector3 position, float radius, float height = -1.0)
      : position(position), radius(radius), height(height) {}

  /**
   * @brief      Project the cylinder to the two dimensional plane
   *
   * @return     The projected disc.
   */
  Disc disc() const { return {position.head<2>(), radius}; }
};

/**
 * @brief      Three dimensional neighbor of a vertical cylindrical shape.
 */
struct Neighbor3 : Cylinder {
  /**
   * The horizontal velocity
   */
  Vector2 velocity;
  /**
   * See \ref Neighbor::id
   */
  unsigned id;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  position       Center of the lower face
   * @param[in]  radius         Radius of the cylinder
   * @param[in]  height         Height of the cylinder
   * @param[in]  velocity       Planar velocity
   * @param[in]  id             Neighbor identifier
   */
  Neighbor3(const Vector3 position, float radius, float height = -1.0,
            Vector2 velocity = Vector2::Zero(), unsigned id = 0.0)
      : Cylinder(position, radius, height), velocity(velocity), id(id) {}

  /**
   * @brief      Project to the two dimensional plane.
   *
   * @return     The projected neighbor.
   */
  Neighbor neighbor() const { return Neighbor{disc(), velocity.head<2>(), id}; }
};

// TODO(old) complete with obstacle avoidance

struct SimpleControl {
  enum class Mode {
    idle,
    value,
    speed,
  };

  float value;
  float speed;
  float target;
  float tau;
  float optimal_speed;
  float target_speed;
  bool value_set;
  bool target_speed_set;
  bool target_set;
  Mode mode;

  SimpleControl()
      : value(0.0f),
        speed(0.0f),
        target(0.0f),
        tau(0.125f),
        optimal_speed(1.0f),
        target_speed(0.0f),
        value_set(false),
        target_speed_set(false),
        target_set(false),
        mode(Mode::idle) {}

  float update(float dt) {
    if (mode == Mode::value && target_set && value_set) {
      float desired_speed =
          std::clamp((target - value) / tau, -optimal_speed, optimal_speed);
      return desired_speed + dt * (speed - desired_speed) / tau;
    }
    if (mode == Mode::speed && target_speed_set) {
      target_speed = std::clamp(target_speed, -optimal_speed, optimal_speed);
      return target_speed + (target_speed - speed) / tau;
    }
    return 0.0f;
  }

  bool enabled() const {
    return ((mode == Mode::value && target_set && value_set) ||
            (mode == Mode::speed && target_speed_set));
  }
};

/**
 * @brief      Three-dimensional twist composed of  velocity and angular speed.
 *
 * Twist3 coordinates may be in a fixed frame or in the agent's own frame, as
 * specified by \ref frame.
 */
struct Twist3 {
  /**
   * Velocity
   */
  Vector3 velocity;
  /**
   * Angular speed
   */
  Radians angular_speed;
  /**
   * The frame of reference.
   */
  Frame frame;

  Twist3(const Vector3& velocity, Radians angular_speed = 0.0,
         Frame frame = Frame::absolute)
      : velocity(velocity), angular_speed(angular_speed), frame(frame) {}
  Twist3() : Twist3(Vector3{}) {}
  Twist3(const Twist2& twist, float vz)
      : velocity{twist.velocity.x(), twist.velocity.x(), vz},
        angular_speed(twist.angular_speed),
        frame(twist.frame) {}
  /**
   * @brief      Project to the two dimensional plane.
   *
   * @return     The projected twist
   */
  Twist2 project() const { return {velocity.head<2>(), angular_speed, frame}; }
};

/**
 * @brief      Three-dimensional pose composed of  position and orientation.
 *
 * Poses are assumed to be a world fixed frame.
 */
struct Pose3 {
  /**
   * Position in world frame
   */
  Vector3 position;
  /**
   * Orientation in world frame
   */
  Radians orientation;

  Pose3(const Vector3& position, Radians orientation = 0.0)
      : position(position), orientation(orientation) {}
  Pose3() : Pose3(Vector3{}) {}
  Pose3(const Pose2& pose, float z)
      : position{pose.position.x(), pose.position.x(), z},
        orientation(pose.orientation) {}
  /**
   * @brief      Integrate a pose
   *
   * @param[in]  twist  The twist
   * @param[in]  dt     The time step
   *
   * @return     ``pose + dt * twist``
   */
  Pose3 integrate(const Twist3& twist, float dt) {
    return {position + dt * twist.velocity,
            orientation + dt * twist.angular_speed};
  }
  /**
   * @brief      Project to the two dimensional plane.
   *
   * @return     The projected pose
   */
  Pose2 project() const { return {position.head<2>(), orientation}; }
};

/**
 * @brief  Simplistic extension of \ref Controller to 3D.
 *
 * Assumes that the agent has planar attitude and motion in 3D space.
 *
 * Relies on the 2D obstacle avoidance \ref Behavior,
 * when provided with all relevant state (i.e.,
 * vertical position, speed, and target position), it 1) considers only
 * obstacles that may intersect in three dimensions 2) controls vertical
 * speed with a proportional controller.
 *
 * Note that this is at best a 2.5D controller, making no attempt at 3D
 * collision avoidance. It is best suited for planar motion at arbitrary
 * altitudes, like when a drone flies above ground robots, effectively ignoring
 * them.
 *
 * *Typical usage*
 *
 * 1. Pick, configure a \ref Behavior, set it to \ref behavior
 *
 * 2. Initialize and configure the controller
 *
 * 3. At regular intervals update the state, using the \ref Controller3 API,
 * and call \ref update_3d.
 *
 * 4. Trigger one of the actions.
 *
 * 5. Either manually check the action's \ref get_state or set callbacks to be
 * notified when the action terminates or updates.
 *
 * 6. Actuate the target command by using the return value of \ref update_3d
 *    or by setting a callback \ref set_cmd_cb
 */
class NAVGROUND_CORE_EXPORT Controller3 : public Controller {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  behavior                   The navigation behavior
   * @param[in]  limit_to_2d                Whether to limit the control to 2D
   */
  Controller3(std::shared_ptr<Behavior> behavior = nullptr,
              bool limit_to_2d = false)
      : Controller(behavior), altitude(), limit_to_2d{limit_to_2d} {}

  /**
   * @brief      Sets the neighbors.
   *
   * Filter relevant neighbors, project them to 2D and pass them to \ref
   * GeometricState::set_neighbors
   *
   * @param[in]  neighbors  The neighbors
   */
  void set_neighbors(const std::vector<Neighbor3>& neighbors);
  /**
   * @brief      Sets the static obstacles.
   *
   * Filter relevant obstacles, project them to 2D and pass them to \ref
   * GeometricState::set_static_obstacles
   *
   * @param[in]  obstacles  The obstacles
   */
  void set_static_obstacles(const std::vector<Cylinder>& obstacles);
  /**
   * @brief      Gets the pose.
   *
   * @return     The pose.
   */
  Pose3 get_pose() const {
    if (behavior) {
      return Pose3(behavior->get_pose(), altitude.value);
    }
    return {};
  }
  /**
   * @brief      Sets the pose.
   *
   * Automatically also set the behavior 2D pose.
   *
   * @param[in]  pose  The pose
   */
  void set_pose(const Pose3& pose) {
    altitude.value = pose.position[2];
    altitude.value_set = true;
    if (behavior) {
      behavior->set_pose(pose.project());
    }
  }
  /**
   * @brief      Gets the twist.
   *
   * @return     The twist.
   */
  Twist3 get_twist() const {
    if (behavior) {
      return Twist3(behavior->get_twist(), altitude.speed);
    }
    return {};
  }
  /**
   * @brief      Sets the twist.
   *
   * Automatically set the behavior 2D twist.
   *
   * @param[in]  twist  The twist
   */
  void set_twist(const Twist3& twist) {
    altitude.speed = twist.velocity[2];
    if (behavior) {
      behavior->set_twist(twist.project());
    }
  }
  /**
   * @brief      Actuate a twist command, integrating using \ref
   * Pose3::integrate
   *
   * @param[in]  twist  The twist
   * @param[in]  time_step     the time step
   */
  void actuate(const Twist3& twist, float time_step) {
    altitude.speed = twist.velocity[2];
    altitude.value += altitude.speed * time_step;
    if (behavior) {
      behavior->actuate(twist.project(), time_step);
    }
  }
  /**
   * @brief      Gets the altitude optimal speed.
   *
   * @return     The altitude optimal speed.
   */
  float get_altitude_optimal_speed() const { return altitude.optimal_speed; }
  /**
   * @brief      Sets the altitude optimal speed.
   *
   * @param[in]  value  The desired value
   */
  void set_altitude_optimal_speed(float value) {
    altitude.optimal_speed = value;
  }
  /**
   * @brief      Gets the altitude relaxation time.
   *
   * @return     The altitude relaxation time.
   */
  float get_altitude_tau() const { return altitude.tau; }
  /**
   * @brief      Sets the altitude relaxation time.
   *
   * @param[in]  value  The desired value
   */
  void set_altitude_tau(float value) { altitude.tau = value; }
  /**
   * @brief      Starts an action to go to a 3D point.
   *
   * The action succeed once the agent arrives within a tolerance
   * from the target point and comes to a stop.
   *
   * If an action is already running, the controller aborts it.
   *
   * @param[in]  point              The target point
   * @param[in]  tolerance  The spatial tolerance
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> go_to_position(const Vector3& point, float tolerance);
  /**
   * @brief      Starts an action to go to a 3D pose.
   *
   * The action succeed once the agent arrives within a tolerances
   * from the target pose and comes to a stop.
   *
   * If an action is already running, the controller aborts it.
   *
   * @param[in]  pose              The target pose
   * @param[in]  position_tolerance  The spatial tolerance
   * @param[in]  orientation_tolerance  The spatial tolerance
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> go_to_pose(const Pose3& pose,
                                     float position_tolerance,
                                     float orientation_tolerance);
  /**
   * @brief      Starts an action to follow a 3D point.
   *
   * The action keeps running even after the agent arrive at the target.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a point/pose, in which case it just updates the target.
   *
   * @param[in]  point              The target point
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_point(const Vector3& point);
  /**
   * @brief      Starts an action to follow a 3D pose.
   *
   * The action keeps running even after the agent arrive at the target.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a point/pose, in which case it just updates the target.
   *
   * @param[in]  pose              The target pose
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_pose(const Pose3& pose);
  /**
   * @brief      Starts an action to follow a 3D velocity.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a velocity/twist, in which case it just updates the target.
   *
   * @param[in]  velocity              The target velocity
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_velocity(const Vector3& velocity);
  /**
   * @brief      Starts an action to follow a 3D twist.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a velocity/twist, in which case it just updates the target.
   *
   * @param[in]  twist              The target twist
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_twist(const Twist3& twist);

  /**
   * @private
   */
  float estimate_time_until_target_satisfied() const override {
    // Expect[tolerance >= 0];
    const float t = Controller::estimate_time_until_target_satisfied();
    if (limit_to_2d) {
      return t;
    }
    return std::max(
        t, abs(altitude.target - altitude.value) / altitude.optimal_speed);
  }

  /**
   * @private
   */
  bool is_still() const override {
    return Controller::is_still() &&
           (limit_to_2d || abs(altitude.speed) < speed_tolerance);
  }

  /**
   * @brief      Updates the control for time step, computing a 3D command.
   *
   * Internally calls \ref Behavior::compute_cmd for collision avoidance.
   *
   * @param[in]  time_step  The time step
   *
   * @return     The command twist to execute the current action
   */
  Twist3 update_3d(float time_step);

  /**
   * @brief      Sets the callback called each time a command is computed for an
   * active action.
   *
   * @param[in]  value  The callback that takes the 3D command twist as
   * argument.
   */
  void set_cmd_cb(std::function<void(const Twist3&)> value) {
    cmd_cb_3 = value;
  }

  /**
   * @brief      Determines if currently limited to 2D control.
   *
   * @return     True if limited to 2D, False otherwise.
   */
  bool is_limited_to_2d() const { return limit_to_2d; }
  /**
   * @brief      Set if the control should ignore 3D information
   *
   * @param[in]  value The desired value
   */
  void should_be_limited_to_2d(bool value) { limit_to_2d = value; }

 private:
  SimpleControl altitude;
  bool limit_to_2d;
  std::optional<std::function<void(const Twist3&)>> cmd_cb_3;
};

}  // namespace navground::core

#endif /* navground_CONTROLLER_3D_H_ */
