/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behaviors/HL.h"

#include <assert.h>

#include <algorithm>
#include <iostream>

#include "navground/core/collision_computation.h"
#include "navground/core/common.h"

static ng_float_t relax(ng_float_t x0, ng_float_t x1, ng_float_t tau,
                        ng_float_t dt) {
  if (tau == 0) return x1;
  // float dt=0.1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}

static std::vector<ng_float_t> relax(const std::vector<ng_float_t> &v0,
                                     const std::vector<ng_float_t> &v1,
                                     ng_float_t tau, ng_float_t dt) {
  if (tau == 0) return v1;
  auto v2 = std::vector<ng_float_t>(v0.size());
  for (size_t i = 0; i < v0.size(); i++) {
    v2[i] = relax(v0[i], v1[i], tau, dt);
  }
  return v2;
}

static navground::core::Twist2 relax(const navground::core::Twist2 &v0,
                                     const navground::core::Twist2 &v1,
                                     ng_float_t tau, ng_float_t dt) {
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
  ng_float_t margin = radius + safety_margin + neighbor.radius;
  ng_float_t distance = delta.norm() - margin;
  if (epsilon > 0 && distance < epsilon) {
    delta = delta / delta.norm() * (margin + epsilon);
    distance = epsilon;
  }
  margin += social_margin.get(0, distance);
  return {delta, margin, neighbor.velocity, barrier_angle};
}

DiscCache HLBehavior::make_obstacle_cache(const Disc &obstacle) {
  Vector2 delta = obstacle.position - pose.position;
  const ng_float_t margin = radius + safety_margin + obstacle.radius;
  const ng_float_t distance = delta.norm() - margin;
  if (epsilon > 0 && distance < epsilon) {
    delta = delta / delta.norm() * (margin + epsilon);
  }
  return {delta, margin, Vector2::Zero(), barrier_angle};
}

// HLBehavior::~HLBehavior() = default;

std::valarray<ng_float_t> HLBehavior::get_collision_distance(
    bool assuming_static, std::optional<ng_float_t> speed) {
  ng_float_t target_speed = speed.value_or(cached_target_speed);
  prepare(target_speed);
  return collision_computation.get_free_distance_for_sector(
      pose.orientation - aperture, 2 * aperture, resolution, effective_horizon,
      !assuming_static, target_speed);
}

std::valarray<ng_float_t> HLBehavior::get_collision_angles() const {
  return collision_computation.get_angles_for_sector(
      pose.orientation - aperture, 2 * aperture, resolution);
}

// distance from agent center to target
static inline ng_float_t distance_from_target(Radians angle,
                                              ng_float_t free_distance,
                                              ng_float_t horizon) {
  if (cos(angle) * horizon < free_distance) {
    return fabs(sin(angle) * horizon);
  }
  return sqrt(horizon * horizon + free_distance * free_distance -
              2 * free_distance * horizon * cos(angle));
}

// TODO(J:revision2023): check why we need effective_horizon
// output is in absolute frame
Vector2 HLBehavior::desired_velocity_towards_point(
    const Vector2 &point, ng_float_t target_speed,
    [[maybe_unused]] ng_float_t dt) {
  prepare(target_speed);
  const Vector2 delta_target = point - pose.position;
  const Radians start_angle = orientation_of(delta_target);
  const Radians relative_start_angle = start_angle - pose.orientation;
  const Radians da = get_angular_resolution();
  // float max_distance = agentToTarget.norm();
  // effective_horizon = horizon;
  // effective_horizon=fmin(horizon,D);
  // Vector2 effectiveTarget = agentToTarget / D * effective_horizon;
  const ng_float_t max_distance = effective_horizon;  // - radius;
  const Radians max_angle{1.6};                       // Radians::PI_OVER_TW0;
  Radians angle = 0;
  // relative to target direction
  Radians optimal_angle = 0;
  ng_float_t optimal_distance_from_target = max_distance;
  int out[2] = {0, 0};
  bool found = false;
  while (angle < max_angle && !(out[0] == 2 && out[1] == 2)) {
    ng_float_t distance_from_target_lower_bound =
        fabs(sin(angle) * max_distance);
    // distance_from_target_lower_bound=2*D*Sin(0.5*optimal_angle);
    if (optimal_distance_from_target <= distance_from_target_lower_bound) {
      // break;
    }
    for (int i = 0; i < 2; ++i) {
      const ng_float_t s_angle = i == 0 ? angle : -angle;
      const bool inside =
          abs(normalize(relative_start_angle + s_angle)) < aperture;
      if (out[i] == 1 && !inside) out[i] = 2;
      if (out[i] == 0 && inside) out[i] = 1;
      if (inside) {
        // free distance to travel before collision
        const ng_float_t free_distance =
            collision_computation.dynamic_free_distance(
                start_angle + s_angle, max_distance, target_speed);
        const ng_float_t dist =
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
  const ng_float_t static_distance = collision_computation.static_free_distance(
      start_angle + optimal_angle, max_distance);
  const ng_float_t desired_speed = fmin(target_speed, static_distance / eta);
  return desired_speed * unit(start_angle + optimal_angle);
}

Vector2 HLBehavior::desired_velocity_towards_velocity(
    const Vector2 &target_velocity, ng_float_t dt) {
  const ng_float_t speed = target_velocity.norm();
  if (speed) {
    return desired_velocity_towards_point(
        pose.position + target_velocity / speed * effective_horizon, speed, dt);
  }
  return Vector2::Zero();
}

void HLBehavior::prepare(ng_float_t target_speed) {
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

Twist2 HLBehavior::relax(const Twist2 &current_value, const Twist2 &value,
                         ng_float_t dt) const {
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

Twist2 HLBehavior::compute_cmd(ng_float_t dt, std::optional<Frame> frame) {
  if (!kinematics) {
    std::cerr << "Missing kinematics" << std::endl;
    return {};
  }
  Twist2 old_actuated_twist = actuated_twist;
  Twist2 cmd_twist = Behavior::compute_cmd(dt, frame);
  if (tau > 0) {
    cmd_twist =
        to_frame(relax(old_actuated_twist, cmd_twist, dt), cmd_twist.frame);
  }
  if (assume_cmd_is_actuated) {
    actuated_twist = cmd_twist;
  }
  return cmd_twist;
}

const std::map<std::string, Property> HLBehavior::properties =
    Properties{
        {"tau",
         make_property<ng_float_t, HLBehavior>(
             &HLBehavior::get_tau, &HLBehavior::set_tau, default_tau, "Tau")},
        {"eta",
         make_property<ng_float_t, HLBehavior>(
             &HLBehavior::get_eta, &HLBehavior::set_eta, default_eta, "Eta")},
        {"aperture", make_property<ng_float_t, HLBehavior>(
                         &HLBehavior::get_aperture, &HLBehavior::set_aperture,
                         default_aperture, "Aperture angle")},
        {"resolution",
         make_property<int, HLBehavior>(&HLBehavior::get_resolution,
                                        &HLBehavior::set_resolution,
                                        default_resolution, "Resolution")},
        {"epsilon", make_property<ng_float_t, HLBehavior>(
                        &HLBehavior::get_epsilon, &HLBehavior::set_epsilon,
                        default_epsilon, "Epsilon")},
        {"barrier_angle",
         make_property<ng_float_t, HLBehavior>(
             &HLBehavior::get_barrier_angle, &HLBehavior::set_barrier_angle,
             default_barrier_angle, "Barrier angle")},
    } +
    Behavior::properties;

const std::string HLBehavior::type = register_type<HLBehavior>("HL");

}  // namespace navground::core
