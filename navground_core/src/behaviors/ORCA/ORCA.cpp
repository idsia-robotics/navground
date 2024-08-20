/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behaviors/ORCA.h"

#include <cmath>

#include "RVO2/Agent.h"
#include "RVO2/Line.h"
#include "RVO2/Obstacle.h"
#include "RVO2/Vector2.h"

namespace navground::core {

RVO::Vector2 vector_from(const Vector2 &value) {
  return RVO::Vector2(value[0], value[1]);
}

Vector2 vector_from(const RVO::Vector2 &value) {
  return Vector2(value.x(), value.y());
}

bool ORCABehavior::get_treat_obstacles_as_agents() const {
  return treat_obstacles_as_agents;
}

void ORCABehavior::set_treat_obstacles_as_agents(bool value) {
  treat_obstacles_as_agents = value;
}

ORCABehavior::ORCABehavior(std::shared_ptr<Kinematics> kinematics,
                           ng_float_t radius)
    : Behavior(kinematics, radius),
      state(),
      use_effective_center(false),
      treat_obstacles_as_agents(true),
      _RVOAgent(std::make_unique<RVO::Agent>()),
      rvo_neighbors(),
      rvo_static_obstacles(),
      rvo_line_obstacles(),
      rvo_square_obstacles() {
  _RVOAgent->maxNeighbors_ = 1000;
  _RVOAgent->timeHorizon_ = 10;
  _RVOAgent->timeHorizonObst_ = 10;
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
  a->point_ = vector_from(pa);
  a->previous_ = b.get();
  a->next_ = b.get();
  a->isConvex_ = true;
  a->direction_ = vector_from(line.e1);
  b->point_ = vector_from(pb);
  b->previous_ = a.get();
  b->next_ = a.get();
  b->isConvex_ = true;
  b->direction_ = -a->direction_;
  rvo_line_obstacles.push_back(std::move(a));
  rvo_line_obstacles.push_back(std::move(b));
}

// TODO(old) add non penetration check!

void ORCABehavior::add_obstacle_as_agent(const Disc &obstacle, bool push_away,
                                         ng_float_t epsilon) {
  auto a = std::make_unique<RVO::Agent>();
  a->velocity_ = RVO::Vector2(0, 0);
  a->prefVelocity_ = a->velocity_;
  Vector2 p = obstacle.position;
  const ng_float_t margin = obstacle.radius + safety_margin + radius;
  const Vector2 delta = obstacle.position - pose.position;
  const ng_float_t distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
  }
  a->position_ = vector_from(p);
  a->radius_ = obstacle.radius;
  rvo_static_obstacles.push_back(std::move(a));
}

void ORCABehavior::add_obstacle_as_square(const Disc &obstacle, bool push_away,
                                          ng_float_t epsilon) {
  Vector2 p = obstacle.position;
  const ng_float_t margin = obstacle.radius + safety_margin + radius;
  const Vector2 delta = p - pose.position;
  const ng_float_t distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
  }
  const ng_float_t r = obstacle.radius;
  std::array<Vector2, 4> ps{Vector2{r, r}, Vector2{-r, r}, Vector2{-r, -r},
                            Vector2{r, -r}};
  std::array<Vector2, 4> e{Vector2{-1, 0}, Vector2{0, -1}, Vector2{1, 0},
                           Vector2{0, 1}};
  RVO::Obstacle *po = nullptr;
  RVO::Obstacle *fo = nullptr;
  for (int i = 0; i < 4; ++i) {
    /* code */
    auto o = std::make_unique<RVO::Obstacle>();
    const Vector2 x = p + ps[i];
    o->point_ = vector_from(x);
    o->isConvex_ = true;
    o->direction_ = vector_from(e[i]);
    if (po) {
      o->previous_ = po;
      po->next_ = o.get();
    }
    po = o.get();
    if (!fo) {
      fo = o.get();
    }
    rvo_square_obstacles.push_back(std::move(o));
  }
  fo->previous_ = po;
  po->next_ = fo;
}

void ORCABehavior::add_neighbor(const Neighbor &neighbor, bool push_away,
                                ng_float_t epsilon) {
  auto a = std::make_unique<RVO::Agent>();
  a->velocity_ = vector_from(neighbor.velocity);
  a->prefVelocity_ = a->velocity_;

  Vector2 p = neighbor.position;
  const ng_float_t margin = neighbor.radius + safety_margin + radius;
  const Vector2 delta = neighbor.position - pose.position;
  ng_float_t distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
    distance = epsilon;
  }
  a->position_ = vector_from(p);
  a->radius_ = neighbor.radius + social_margin.get(0, distance);
  // [[maybe_unused]] Vector2 relative_position =
  //     obstacle_relative_position(pose.position, p, radius, d.radius,
  //     distance);
  // a->radius_ = d.radius + fmin(distance - d.radius - _RVOAgent->radius_ -
  // 0.001,
  //                              obstacle_margin(distance, radius, d.radius,
  //                                              safety_margin,
  //                                              d.social_margin));
  rvo_neighbors.push_back(std::move(a));
}

void ORCABehavior::set_time_horizon(ng_float_t value) {
  _RVOAgent->timeHorizon_ = value;
}
ng_float_t ORCABehavior::get_time_horizon() const {
  return _RVOAgent->timeHorizon_;
}

void ORCABehavior::set_static_time_horizon(ng_float_t value) {
  _RVOAgent->timeHorizonObst_ = value;
}
ng_float_t ORCABehavior::get_static_time_horizon() const {
  return _RVOAgent->timeHorizonObst_;
}

void ORCABehavior::set_max_number_of_neighbors(unsigned value) {
  _RVOAgent->maxNeighbors_ = value;
}

unsigned ORCABehavior::get_max_number_of_neighbors() const {
  return static_cast<unsigned>(_RVOAgent->maxNeighbors_);
}

Vector2 ORCABehavior::effective_position() const {
  if (is_using_effective_center()) {
    return pose.position + unit(pose.orientation) * D;
  }
  return pose.position;
}

void ORCABehavior::prepare(const Vector2 &target_velocity) {
  // TODO(Jerome): check if maxSpeed_ should be set to target or to max
  // TODO(Jerome): avoid repetitions (effective_position vs this function)
  if (is_using_effective_center()) {
    WheeledKinematics *wk = dynamic_cast<WheeledKinematics *>(kinematics.get());
    D = wk->get_axis() / 2;
    const RVO::Vector2 delta = vector_from(unit(pose.orientation) * D);
    _RVOAgent->position_ = vector_from(pose.position) + delta;
    _RVOAgent->radius_ = safety_margin + radius + D;
    _RVOAgent->velocity_ = vector_from(
        twist.velocity + normal(pose.orientation) * D * twist.angular_speed);
    _RVOAgent->maxSpeed_ =
        target_velocity.norm() / sqrt(1 + std::pow(wk->get_axis() / D / 2, 2));
  } else {
    _RVOAgent->radius_ = safety_margin + radius;
    _RVOAgent->velocity_ = vector_from(twist.velocity);
    _RVOAgent->position_ = vector_from(pose.position);
    _RVOAgent->maxSpeed_ = target_velocity.norm();
  }
  // TODO(Jerome): why 2 * ... verify
  _RVOAgent->neighborDist_ = 2 * horizon;
  _RVOAgent->prefVelocity_ = vector_from(target_velocity);
  if (state.changed(GeometricState::LINE_OBSTACLES)) {
    rvo_line_obstacles.clear();
    for (auto &line : state.get_line_obstacles()) {
      add_line_obstacle(line);
    }
  }
  if (state.changed(GeometricState::STATIC_OBSTACLES) ||
      Behavior::changed(POSITION | SAFETY_MARGIN | RADIUS)) {
    if (treat_obstacles_as_agents) {
      rvo_static_obstacles.clear();
      for (const auto &o : state.get_static_obstacles()) {
        add_obstacle_as_agent(o, true, static_cast<ng_float_t>(2e-3));
      }
    } else {
      rvo_square_obstacles.clear();
      for (const auto &o : state.get_static_obstacles()) {
        add_obstacle_as_square(o, true, static_cast<ng_float_t>(2e-3));
      }
    }
  }
  if (state.changed(GeometricState::NEIGHBORS) ||
      Behavior::changed(POSITION | SAFETY_MARGIN | RADIUS)) {
    rvo_neighbors.clear();
    for (const auto &n : state.get_neighbors()) {
      add_neighbor(n, true, static_cast<ng_float_t>(2e-3));
    }
  }
  // Does the same job as RVO::Agent::computeNeighbors()

  // TODO:

  _RVOAgent->obstacleNeighbors_.clear();
  _RVOAgent->agentNeighbors_.clear();
  float rangeSq = (horizon * 2) * (horizon * 2);
  float rangeSqObst = std::pow(
      _RVOAgent->timeHorizonObst_ * _RVOAgent->maxSpeed_ + _RVOAgent->radius_,
      2);
  for (const auto &obstacle : rvo_line_obstacles) {
    const auto next_obstacle = obstacle->next_;
    const float agentLeftOfLine = RVO::leftOf(
        obstacle->point_, next_obstacle->point_, _RVOAgent->position_);
    const float distSqLine =
        agentLeftOfLine * agentLeftOfLine /
        RVO::absSq(next_obstacle->point_ - obstacle->point_);
    if (distSqLine < rangeSqObst) {
      if (agentLeftOfLine < 0.0f) {
        _RVOAgent->insertObstacleNeighbor(obstacle.get(), rangeSqObst);
      }
    }
  }
  if (treat_obstacles_as_agents) {
    for (const auto &obstacle : rvo_static_obstacles) {
      _RVOAgent->insertAgentNeighbor(obstacle.get(), rangeSq);
    }
  } else {
    for (const auto &obstacle : rvo_square_obstacles) {
      _RVOAgent->insertObstacleNeighbor(obstacle.get(), rangeSqObst);
    }
  }
  for (const auto &neighbor : rvo_neighbors) {
    _RVOAgent->insertAgentNeighbor(neighbor.get(), rangeSq);
  }
  state.reset_changes();
}

Vector2 ORCABehavior::desired_velocity_towards_velocity(const Vector2 &velocity,
                                                        ng_float_t dt) {
  prepare(velocity);
  _RVOAgent->computeNewVelocity(dt);
  return vector_from(_RVOAgent->newVelocity_);
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

std::vector<ORCABehavior::Line> ORCABehavior::get_lines() const {
  const auto ls = _RVOAgent->orcaLines_;
  std::vector<ORCABehavior::Line> rs;
  std::transform(ls.cbegin(), ls.cend(), std::back_inserter(rs),
                 [](const RVO::Line &line) {
                   return ORCABehavior::Line{vector_from(line.point),
                                             vector_from(line.direction)};
                 });
  return rs;
}

// const char *ORCABehavior::name = register_type<ORCABehavior>("ORCA");

const std::map<std::string, Property> ORCABehavior::properties =
    Properties{
        {"time_horizon",
         make_property<ng_float_t, ORCABehavior>(
             &ORCABehavior::get_time_horizon, &ORCABehavior::set_time_horizon,
             10, "Time horizon")},
        {"static_time_horizon",
         make_property<ng_float_t, ORCABehavior>(
             &ORCABehavior::get_static_time_horizon,
             &ORCABehavior::set_static_time_horizon, 10,
             "Time horizon applied to static linear obstacles")},
        {"effective_center",
         make_property<bool, ORCABehavior>(
             &ORCABehavior::is_using_effective_center,
             &ORCABehavior::should_use_effective_center, false,
             "Whenever to use an effective center to handle non-holonomic "
             "kinematics")},
        {"treat_obstacles_as_agents",
         make_property<bool, ORCABehavior>(
             &ORCABehavior::get_treat_obstacles_as_agents,
             &ORCABehavior::set_treat_obstacles_as_agents, true,
             "Whenever to treat static obstacles as static [RVO] agents")},
        {"max_neighbors", make_property<int, ORCABehavior>(
                              &ORCABehavior::get_max_number_of_neighbors,
                              &ORCABehavior::set_max_number_of_neighbors, 1000,
                              "The maximal number of [RVO] neighbors")},
    } +
    Behavior::properties;

const std::string ORCABehavior::type = register_type<ORCABehavior>("ORCA");

}  // namespace navground::core
