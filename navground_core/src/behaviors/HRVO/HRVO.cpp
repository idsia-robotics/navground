/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behaviors/HRVO.h"

#include "HRVO/Agent.h"
#include "HRVO/HRVO.h"
#include "HRVO/Obstacle.h"

namespace navground::core {

HRVOBehavior::HRVOBehavior(std::shared_ptr<Kinematics> kinematics,
                           ng_float_t radius)
    : Behavior(kinematics, radius),
      state(),
      agentIndex(0),
      // rangeSq(0),
      _HRVOAgent(std::make_unique<HRVO::Agent>()) {
  _HRVOAgent->maxNeighbors_ = 1000;
}
HRVOBehavior::~HRVOBehavior() = default;

void HRVOBehavior::prepare(const Vector2 &target_velocity) {
  _HRVOAgent->radius_ = radius;
  _HRVOAgent->velocity_ = HRVO::Vector2(twist.velocity.x(), twist.velocity.y());
  _HRVOAgent->orientation_ = normalize(pose.orientation);
  _HRVOAgent->position_ = HRVO::Vector2(pose.position.x(), pose.position.y());
  _HRVOAgent->neighborDist_ = 2 * horizon;
  _HRVOAgent->isColliding_ = false;
  ng_float_t rangeSq = (horizon * 2) * (horizon * 2);

  _HRVOAgent->prefVelocity_ =
      HRVO::Vector2(target_velocity[0], target_velocity[1]);

  // This is actually useless as prefSpeed_
  // is only used in HRVO::Agent::computePreferredVelocity

  _HRVOAgent->prefSpeed_ = target_velocity.norm();
  // TODO(Jerome): check if max should be set to target or to max
  _HRVOAgent->maxSpeed_ = _HRVOAgent->prefSpeed_;
  //_HRVOAgent->maxSpeed_ = get_max_speed();
  // _HRVOAgent->uncertaintyOffset_ = 0;

  if (state.changed(GeometricState::NEIGHBORS |
                    GeometricState::STATIC_OBSTACLES) ||
      Behavior::changed(POSITION | RADIUS | SAFETY_MARGIN)) {
    _HRVOAgent->neighbors_.clear();
    // These are not used anyway
    // for (uint i = 0; i < _HRVOAgent->obstacles_.size(); i++) {
    //   delete _HRVOAgent->obstacles_[i];
    // }

    for (uint i = 0; i < _HRVOAgent->agents_.size(); i++) {
      delete _HRVOAgent->agents_[i];
    }
    // _HRVOAgent->obstacles_.clear();
    _HRVOAgent->agents_.clear();
    agentIndex = 0;

    for (const auto &n : state.get_neighbors()) {
      add_neighbor(n, rangeSq, true, 2e-3);
    }
    for (const auto &o : state.get_static_obstacles()) {
      add_obstacle(o, rangeSq, true, 2e-3);
    }
  }

  state.reset_changes();
  Behavior::reset_changes();
}

void HRVOBehavior::add_obstacle(const Disc &d, float rangeSq, bool push_away,
                                ng_float_t epsilon) {
  HRVO::Agent *a = new HRVO::Agent();
  // a->velocity_ = HRVO::Vector2((float)d.velocity.x(), (float)d.velocity.y());
  // a->prefVelocity_ = a->velocity_;
  Vector2 p = d.position;
  const ng_float_t margin = d.radius + safety_margin + radius;
  const Vector2 delta = d.position - pose.position;
  const ng_float_t distance = delta.norm() - margin;
  if (push_away && distance < epsilon) {
    p += delta / delta.norm() * (-distance + epsilon);
  }
  a->position_ = HRVO::Vector2(static_cast<ng_float_t>(p.x()),
                               static_cast<ng_float_t>(p.y()));
  a->radius_ = d.radius + safety_margin;
  _HRVOAgent->agents_.push_back(a);
  _HRVOAgent->insertAgentNeighbor(agentIndex, rangeSq);
  agentIndex++;
}

void HRVOBehavior::add_neighbor(const Neighbor &neighbor, float rangeSq,
                                bool push_away, ng_float_t epsilon) {
  HRVO::Agent *a = new HRVO::Agent();
  a->velocity_ = HRVO::Vector2(static_cast<ng_float_t>(neighbor.velocity.x()),
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
  a->position_ = HRVO::Vector2(static_cast<ng_float_t>(p.x()),
                               static_cast<ng_float_t>(p.y()));
  a->radius_ = neighbor.radius + safety_margin + social_margin.get(0, distance);

  // float distance;
  // [[maybe_unused]] Vector2 relative_position =
  //     obstacle_relative_position(pose.position, p, radius, d.radius,
  //     distance);
  ////
  /// a->radius_=r+marginForObstacleAtDistance(distance,r,safetyMargin,socialMargin);
  // a->radius_ =
  //     d.radius + fmin(distance - d.radius - _HRVOAgent->radius_ - 0.001,
  //                     obstacle_margin(distance, radius, d.radius,
  //                     safety_margin,
  //                                     d.social_margin));
  //// printf("Obstacle radius %.3f\n",a->radius_);

  ////
  /// a->radius_=r+marginForObstacleAtDistance(p.norm(),r,safetyMargin,socialMargin);
  _HRVOAgent->agents_.push_back(a);
  _HRVOAgent->insertAgentNeighbor(agentIndex, rangeSq);
  agentIndex++;
}

// DONE: HRVO reduce the speed if it would overshoot the target,
// see L545 in HRVO/Agent.cpp. We do the same here.
Vector2 HRVOBehavior::desired_velocity_towards_point(const Vector2 &point,
                                                     ng_float_t speed,
                                                     ng_float_t dt) {
  const auto delta = point - pose.position;
  const ng_float_t n = delta.norm();
  Vector2 velocity;
  if (n) {
    velocity = delta / n *
               std::max<ng_float_t>(
                   0, dt ? std::min<ng_float_t>(speed, n / dt) : speed);
  }
  return desired_velocity_towards_velocity(velocity, dt);
}

Vector2 HRVOBehavior::desired_velocity_towards_velocity(
    const Vector2 &velocity, [[maybe_unused]] ng_float_t dt) {
  prepare(velocity);
  _HRVOAgent->computeNewVelocity();
  return {_HRVOAgent->newVelocity_.x(), _HRVOAgent->newVelocity_.y()};
}

ng_float_t HRVOBehavior::get_uncertainty_offset() const {
  return _HRVOAgent->uncertaintyOffset_;
}

void HRVOBehavior::set_uncertainty_offset(ng_float_t value) {
  _HRVOAgent->uncertaintyOffset_ = value;
}

void HRVOBehavior::set_max_number_of_neighbors(unsigned value) {
  _HRVOAgent->maxNeighbors_ = value;
}

unsigned HRVOBehavior::get_max_number_of_neighbors() const {
  return _HRVOAgent->maxNeighbors_;
}

const std::map<std::string, Property> HRVOBehavior::properties =
    Properties{
        {"uncertainty_offset",
         make_property<ng_float_t, HRVOBehavior>(
             &HRVOBehavior::get_uncertainty_offset,
             &HRVOBehavior::set_uncertainty_offset, 0, "Uncertainty offset")},
        {"max_neighbors", make_property<int, HRVOBehavior>(
                              &HRVOBehavior::get_max_number_of_neighbors,
                              &HRVOBehavior::set_max_number_of_neighbors, 1000,
                              "The maximal number of [HRVO] neighbors")},
    } +
    Behavior::properties;

const std::string HRVOBehavior::type = register_type<HRVOBehavior>("HRVO");

}  // namespace navground::core
