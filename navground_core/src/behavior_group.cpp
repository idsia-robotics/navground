/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior_group.h"

#include <iostream>
#include <stdexcept>

namespace navground::core {

size_t BehaviorGroup::index(Behavior *behavior) const {
  return std::find(_behaviors.begin(), _behaviors.end(), behavior) -
         _behaviors.begin();
}

bool BehaviorGroup::has(Behavior *behavior) const {
  return std::find(_behaviors.begin(), _behaviors.end(), behavior) !=
         _behaviors.end();
}

void BehaviorGroup::add(Behavior *behavior) {
  if (!has(behavior)) {
    _behaviors.push_back(behavior);
    _cmds.push_back(Twist2());
  }
}

void BehaviorGroup::remove(Behavior *behavior) {
  const auto i = index(behavior);
  if (i < _behaviors.size()) {
    _behaviors.erase(std::next(_behaviors.begin(), i));
    _cmds.erase(std::next(_cmds.begin(), i));
  }
}

Twist2 BehaviorGroup::compute_cmd(ng_float_t dt, Behavior *behavior) {
  const auto i = index(behavior);
  if (i >= _behaviors.size()) {
    std::cerr << "Unknown behavior!" << std::endl;
    return Twist2{Vector2::Zero(), 0, Frame::relative};
  }
  if (i == 0) {
    _cmds = compute_cmds(dt);
  }
  return _cmds[i];
}

size_t BehaviorGroup::size() const { return _behaviors.size(); }

Twist2 BehaviorGroupMember::compute_cmd_internal(ng_float_t dt) {
  if (_group) {
    return _group->compute_cmd(dt, this);
  }
  std::cerr << "Missing group!" << std::endl;
  return Twist2{Vector2::Zero(), 0, Frame::relative};
}

void BehaviorGroupMember::remove_group(size_t key) {
  auto groups = get_groups();
  groups->erase(key);
}

std::shared_ptr<BehaviorGroup>
BehaviorGroupMember::get_or_create_group(size_t key) {
  auto groups = get_groups();
  if (groups->count(key) == 0) {
    groups->emplace(key, make_group());
  }
  return groups->at(key);
}

void BehaviorGroupMember::set_group(
    const std::shared_ptr<BehaviorGroup> &value) {
  if (_group != value) {
    if (_group) {
      _group->remove(this);
    }
    if (_group && _group->size() == 0) {
      const auto key = get_group_hash();
      remove_group(key);
    }
    _group = value;
    if (_group) {
      _group->add(this);
    }
  }
}

void BehaviorGroupMember::prepare() {
  if (!get_group()) {
    const auto key = get_group_hash();
    set_group(get_or_create_group(key));
  }
}

void BehaviorGroupMember::close() { set_group(nullptr); }

BehaviorGroupMember::Groups *BehaviorGroupMember::get_groups() {
  static BehaviorGroupMember::Groups _groups;
  return &_groups;
}

} // namespace navground::core
