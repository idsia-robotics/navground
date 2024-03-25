/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behaviors/ORCA.h"

#include <cmath>

#include "RVO/Agent.h"
#include "RVO/Definitions.h"
#include "RVO/Obstacle.h"
#include "RVO/Vector2.h"

namespace navground::core {

ORCABehavior::ORCABehavior(std::shared_ptr<Kinematics> kinematics,
                           ng_float_t radius)
    : Behavior(kinematics, radius),
      state(),
      use_effective_center(false),
      _RVOAgent(std::make_unique<RVO::Agent>(nullptr)) {
  _RVOAgent->maxNeighbors_ = 1000;
  _RVOAgent->timeHorizon_ = 10;
}

ORCABehavior::~ORCABehavior() = default;

// DONE(J: revision): why do I need rvo_neighbors. It is not enough to use
// _RVOAgent->rvo_neighbors_? No, because insertObstacleNeighbor may remove some
// entry

void ORCABehavior::add_line_obstacle(const LineSegment &line) {
  Vector2 pa = line.p1;  // - line.e1 * safetyMargin - line.e2 * safetyMargin;
  Vector2 pb = line.p2;  // + line.e1 * safetyMargin + line.e2 * safetyMargin;
  auto a = std::make_unique<RVO::Obstacle>();
  auto b = std::make_unique<RVO::Obstacle>();
  a->point_ = RVO::Vector2(pa[0], pa[1]);
  a->prevObstacle = b.get();
  a->nextObstacle = b.get();
  a->isConvex_ = true;
  a->unitDir_ = RVO::Vector2(line.e1[0], line.e1[1]);
  b->point_ = RVO::Vector2(pb[0], pb[1]);
  b->prevObstacle = a.get();
  b->nextObstacle = a.get();
  b->isConvex_ = true;
  b->unitDir_ = -a->unitDir_;
  rvo_obstacles.push_back(std::move(a));
  rvo_obstacles.push_back(std::move(b));
}

// TODO(old) add non penetration check!

void ORCABehavior::add_obstacle(const Disc &obstacle, float rangeSq,
                                bool push_away, ng_float_t epsilon) {
  auto a = std::make_unique<RVO::Agent>(nullptr);
  a->velocity_ = RVO::Vector2(0, 0);
  a->prefVelocity_ = a->velocity_;
  Vector2 p = obstacle.position;
  const ng_float_t margin = obstacle.radius + safety_margin + radius;
  const Vector2 delta = obstacle.position - pose.position;
  const ng_float_t distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
  }
  a->position_ = RVO::Vector2(static_cast<ng_float_t>(p.x()),
                              static_cast<ng_float_t>(p.y()));
  a->radius_ = obstacle.radius + safety_margin;
  _RVOAgent->insertAgentNeighbor(a.get(), rangeSq);
  rvo_neighbors.push_back(std::move(a));
}

void ORCABehavior::add_neighbor(const Neighbor &neighbor, float rangeSq,
                                bool push_away, ng_float_t epsilon) {
  auto a = std::make_unique<RVO::Agent>(nullptr);
  a->velocity_ = RVO::Vector2(static_cast<ng_float_t>(neighbor.velocity.x()),
                              static_cast<ng_float_t>(neighbor.velocity.y()));
  a->prefVelocity_ = a->velocity_;

  Vector2 p = neighbor.position;
  const ng_float_t margin = neighbor.radius + safety_margin + radius;
  const Vector2 delta = neighbor.position - pose.position;
  ng_float_t distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
    distance = epsilon;
  }
  a->position_ = RVO::Vector2(static_cast<ng_float_t>(p.x()),
                              static_cast<ng_float_t>(p.y()));
  a->radius_ = neighbor.radius + safety_margin + social_margin.get(0, distance);
  // [[maybe_unused]] Vector2 relative_position =
  //     obstacle_relative_position(pose.position, p, radius, d.radius,
  //     distance);
  // a->radius_ = d.radius + fmin(distance - d.radius - _RVOAgent->radius_ -
  // 0.001,
  //                              obstacle_margin(distance, radius, d.radius,
  //                                              safety_margin,
  //                                              d.social_margin));
  _RVOAgent->insertAgentNeighbor(a.get(), rangeSq);
  rvo_neighbors.push_back(std::move(a));
}

void ORCABehavior::set_time_horizon(ng_float_t value) {
  _RVOAgent->timeHorizon_ = value;
}
ng_float_t ORCABehavior::get_time_horizon() const {
  return _RVOAgent->timeHorizon_;
}

Vector2 ORCABehavior::effective_position() const {
  if (is_using_effective_center()) {
    return pose.position + unit(pose.orientation) * D;
  }
  return pose.position;
}

void ORCABehavior::prepare(const Vector2 &target_velocity, ng_float_t dt) {
  // TODO(Jerome): check if maxSpeed_ should be set to target or to max
  // TODO(Jerome): avoid repetitions (effective_position vs this function)
  if (is_using_effective_center()) {
    WheeledKinematics *wk = dynamic_cast<WheeledKinematics *>(kinematics.get());
    D = wk->get_axis() * 0.5;
    const RVO::Vector2 delta =
        RVO::Vector2(std::cos(pose.orientation), std::sin(pose.orientation)) *
        D;
    _RVOAgent->position_ =
        RVO::Vector2(pose.position.x(), pose.position.y()) + delta;
    _RVOAgent->radius_ = radius + D;
    _RVOAgent->velocity_ = RVO::Vector2(
        twist.velocity.x() - sin(pose.orientation) * D * twist.angular_speed,
        twist.velocity.y() + cos(pose.orientation) * D * twist.angular_speed);
    _RVOAgent->maxSpeed_ =
        target_velocity.norm() / sqrt(1 + RVO::sqr(0.5 * wk->get_axis() / D));
  } else {
    _RVOAgent->radius_ = radius;
    _RVOAgent->velocity_ = RVO::Vector2(twist.velocity.x(), twist.velocity.y());
    _RVOAgent->position_ = RVO::Vector2(pose.position.x(), pose.position.y());
    _RVOAgent->maxSpeed_ = target_velocity.norm();
  }
  _RVOAgent->timeStep_ = dt;
  // TODO(Jerome): why 2 * ... verify
  _RVOAgent->neighborDist_ = 2 * horizon;
  _RVOAgent->prefVelocity_ =
      RVO::Vector2(target_velocity.x(), target_velocity.y());

  const float rangeSq = (horizon * 2) * (horizon * 2);
  if (state.changed(GeometricState::LINE_OBSTACLES)) {
    rvo_obstacles.clear();
    _RVOAgent->obstacleNeighbors_.clear();
    for (auto &line : state.get_line_obstacles()) {
      add_line_obstacle(line);
    }

    for (auto &obstacle : rvo_obstacles) {
      _RVOAgent->insertObstacleNeighbor(obstacle.get(), rangeSq);
    }
  }
  if (state.changed(GeometricState::STATIC_OBSTACLES |
                    GeometricState::NEIGHBORS)) {
    _RVOAgent->agentNeighbors_.clear();
    rvo_neighbors.clear();
    for (const auto &n : state.get_neighbors()) {
      add_neighbor(n, rangeSq, true, 2e-3);
    }
    for (const auto &o : state.get_static_obstacles()) {
      add_obstacle(o, rangeSq, true, 2e-3);
    }
  }
  state.reset_changes();
}

Vector2 ORCABehavior::desired_velocity_towards_velocity(const Vector2 &velocity,
                                                        ng_float_t dt) {
  prepare(velocity, dt);
  _RVOAgent->computeNewVelocity();
  return Vector2(_RVOAgent->newVelocity_.x(), _RVOAgent->newVelocity_.y());
}

Vector2 ORCABehavior::desired_velocity_towards_point(const Vector2 &point,
                                                     ng_float_t speed,
                                                     ng_float_t dt) {
  const Vector2 delta = point - effective_position();
  const ng_float_t n = delta.norm();
  Vector2 velocity;
  if (n) {
    velocity = delta / n * std::max<ng_float_t>(0, speed);
  }
  return desired_velocity_towards_velocity(velocity, dt);
}

// TODO(J 2023): review ... should I check feasibility?
Twist2 ORCABehavior::twist_towards_velocity(const Vector2 &absolute_velocity,
                                            Frame frame) {
  if (is_using_effective_center()) {
    Radians angle = orientation_of(absolute_velocity) - pose.orientation;
    ng_float_t speed = absolute_velocity.norm();
    if (speed) {
      WheeledKinematics *wk =
          dynamic_cast<WheeledKinematics *>(kinematics.get());
      WheelSpeeds speeds = {
          speed * (std::cos(angle) - wk->get_axis() / 2 / D * std::sin(angle)),
          speed * (std::cos(angle) + wk->get_axis() / 2 / D * std::sin(angle))};
      Twist2 twist = wk->twist(speeds);
      return to_frame(twist, frame);
    } else {
      return {Vector2::Zero(), 0, frame};
    }
  }
  return Behavior::twist_towards_velocity(absolute_velocity, frame);
}

// const char *ORCABehavior::name = register_type<ORCABehavior>("ORCA");

const std::map<std::string, Property> ORCABehavior::properties =
Properties{
    {"time_horizon",
     make_property<ng_float_t, ORCABehavior>(
         &ORCABehavior::get_time_horizon, &ORCABehavior::set_time_horizon,
         10, "Time horizon")},
    {"effective_center",
     make_property<bool, ORCABehavior>(
         &ORCABehavior::is_using_effective_center,
         &ORCABehavior::should_use_effective_center, false,
         "Whenever to use an effective center to handle non-holonomic "
         "kinematics")},
} +
Behavior::properties;

const std::string ORCABehavior::type = register_type<ORCABehavior>("ORCA");

}  // namespace navground::core
