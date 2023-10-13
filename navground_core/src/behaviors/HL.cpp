/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/common.h"


#include <assert.h>
#include <iostream>

#include <algorithm>


#include "navground/core/behaviors/HL.h"

#include "navground/core/collision_computation.h"

static float relax(float x0, float x1, float tau, float dt) {
  if (tau == 0) return x1;
  // float dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}

static std::vector<float> relax(const std::vector<float> &v0,
                                const std::vector<float> &v1, float tau,
                                float dt) {
  if (tau == 0) return v1;
  auto v2 = std::vector<float>(v0.size());
  for (size_t i = 0; i < v0.size(); i++) {
    v2[i] = relax(v0[i], v1[i], tau, dt);
  }
  return v2;
}

static navground::core::Twist2 relax(const navground::core::Twist2 &v0,
                                   const navground::core::Twist2 &v1, float tau,
                                   float dt) {
  assert(v1.frame == v0.frame);
  if (tau == 0) {
    return v1;
  }
  return {{relax(v0.velocity[0], v1.velocity[0], tau, dt),
           relax(v0.velocity[1], v1.velocity[1], tau, dt)},
          relax(v0.angular_speed, v1.angular_speed, tau, dt),
          v1.frame};
}

namespace navground::core {

DiscCache HLBehavior::make_neighbor_cache(const Neighbor &neighbor) {
  Vector2 delta = neighbor.position - pose.position;
  float margin = radius + safety_margin + neighbor.radius;
  float distance = delta.norm() - margin;
  if (epsilon > 0 && distance < epsilon) {
    delta = delta / delta.norm() * (margin + epsilon);
    distance = epsilon;
  }
  margin += social_margin.get(0, distance);
  return {delta, margin, neighbor.velocity, barrier_angle};
}

DiscCache HLBehavior::make_obstacle_cache(const Disc &obstacle) {
  Vector2 delta = obstacle.position - pose.position;
  const float margin = radius + safety_margin + obstacle.radius;
  const float distance = delta.norm() - margin;
  if (epsilon > 0 && distance < epsilon) {
    delta = delta / delta.norm() * (margin + epsilon);
  }
  return {delta, margin, Vector2::Zero(), barrier_angle};
}

// HLBehavior::~HLBehavior() = default;

CollisionComputation::CollisionMap HLBehavior::get_collision_distance(
    bool assuming_static, std::optional<float> speed) {
  float target_speed = speed.value_or(cached_target_speed);
  prepare(target_speed);
  return collision_computation.get_free_distance_for_sector(
      pose.orientation - aperture, 2 * aperture, resolution, effective_horizon,
      !assuming_static, target_speed);
}

// distance from agent center to target
static inline float distance_from_target(Radians angle, float free_distance,
                                         float horizon) {
  if (cos(angle) * horizon < free_distance) {
    return fabs(sin(angle) * horizon);
  }
  return sqrt(horizon * horizon + free_distance * free_distance -
              2 * free_distance * horizon * cos(angle));
}

// TODO(J:revision2023): check why we need effective_horizon
// output is in absolute frame
Vector2 HLBehavior::desired_velocity_towards_point(const Vector2 &point,
                                                   float target_speed,
                                                   [[maybe_unused]] float dt) {
  prepare(target_speed);
  const Vector2 delta_target = point - pose.position;
  const Radians start_angle = orientation_of(delta_target);
  const Radians relative_start_angle = start_angle - pose.orientation;
  const Radians da = get_angular_resolution();
  // float max_distance = agentToTarget.norm();
  // effective_horizon = horizon;
  // effective_horizon=fmin(horizon,D);
  // Vector2 effectiveTarget = agentToTarget / D * effective_horizon;
  const float max_distance = effective_horizon;  // - radius;
  const Radians max_angle{1.6f};                 // Radians::PI_OVER_TW0;
  Radians angle = 0.0f;
  // relative to target direction
  Radians optimal_angle = 0.0f;
  float optimal_distance_from_target = max_distance;
  int out[2] = {0, 0};
  bool found = false;
  while (angle < max_angle && !(out[0] == 2 && out[1] == 2)) {
    float distance_from_target_lower_bound = fabs(sin(angle) * max_distance);
    // distance_from_target_lower_bound=2*D*Sin(0.5*optimal_angle);
    if (optimal_distance_from_target <= distance_from_target_lower_bound) {
      // break;
    }
    for (int i = 0; i < 2; ++i) {
      const float s_angle = i == 0 ? angle : -angle;
      const bool inside =
          abs(normalize(relative_start_angle + s_angle)) < aperture;
      if (out[i] == 1 && !inside) out[i] = 2;
      if (out[i] == 0 && inside) out[i] = 1;
      if (inside) {
        // free distance to travel before collision
        const float free_distance = collision_computation.dynamic_free_distance(
            start_angle + s_angle, max_distance, target_speed);
        const float dist =
            distance_from_target(angle, free_distance, max_distance);
        if (dist < optimal_distance_from_target) {
          optimal_distance_from_target = dist;
          optimal_angle = s_angle;
          found = true;
        }
      }
      if (!angle) break;
    }
    angle += da;
  }
  if (!found) return Vector2::Zero();
  const float static_distance = collision_computation.static_free_distance(
      start_angle + optimal_angle, max_distance);
  const float desired_speed = fmin(target_speed, static_distance / eta);
  return desired_speed * unit(start_angle + optimal_angle);
}

Vector2 HLBehavior::desired_velocity_towards_velocity(
    const Vector2 &target_velocity, float dt) {
  const float speed = target_velocity.norm();
  if (speed) {
    return desired_velocity_towards_point(
        pose.position + target_velocity / speed * effective_horizon, speed, dt);
  }
  return Vector2::Zero();
}

void HLBehavior::prepare(float target_speed) {
  effective_horizon = horizon;
  if (state.changed() ||
      Behavior::changed(POSITION | ORIENTATION | RADIUS | HORIZON |
                        SAFETY_MARGIN) ||
      target_speed != cached_target_speed) {
    cached_target_speed = target_speed;
    std::vector<DiscCache> ns;
    ns.reserve(state.get_neighbors().size());
    for (const Neighbor &d : state.get_neighbors()) {
      const auto c = make_neighbor_cache(d);
      if (collision_computation.dynamic_may_collide(c, effective_horizon,
                                                    target_speed)) {
        ns.push_back(c);
      }
    }
    std::vector<DiscCache> ss;
    ss.reserve(state.get_static_obstacles().size());

    for (const Disc &d : state.get_static_obstacles()) {
      const auto c = make_obstacle_cache(d);
      if (collision_computation.static_may_collide(c, effective_horizon)) {
        ss.push_back(c);
      }
    }
    collision_computation.setup(pose, radius + safety_margin,
                                state.get_line_obstacles(), std::move(ss),
                                std::move(ns));
  }
  state.reset_changes();
  Behavior::reset_changes();
}

Twist2 HLBehavior::relax(const Twist2 &current_value, const Twist2 &value, float dt) const {
  if (kinematics->is_wheeled()) {
    auto wheel_speeds = wheel_speeds_from_twist(value);
    auto current_wheel_speeds = wheel_speeds_from_twist(current_value);
    return twist_from_wheel_speeds(
        ::relax(current_wheel_speeds, wheel_speeds, tau, dt));
  } else {
    // TODO(Jerome old): same than before when I relaxed the absolute velocity,
    // not the relative but different than original paper CHANGED(J 2023): relax
    // in arbitrary frame
    return ::relax(current_value.frame == value.frame
                       ? current_value
                       : to_frame(current_value, value.frame),
                   value, tau, dt);
  }
}

Twist2 HLBehavior::compute_cmd(float dt, std::optional<Frame> frame) {
  if (!kinematics) {
    std::cerr << "Missing kinematics" << std::endl;
    return {};
  }
  Twist2 old_actuated_twist = actuated_twist;
  Twist2 cmd_twist = Behavior::compute_cmd(dt, frame);
  if (tau > 0) {
    cmd_twist = to_frame(relax(old_actuated_twist, cmd_twist, dt), twist.frame);
  }
  if (assume_cmd_is_actuated) {
    actuated_twist = cmd_twist;
  }
  return cmd_twist;
}

}  // namespace navground::core
