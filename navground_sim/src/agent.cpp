/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/agent.h"

namespace navground::sim {

void Agent::prepare(World *world) {
  if (ready)
    return;
  for(auto & state_estimation : state_estimations) {
    state_estimation->prepare(this, world);
  }
  collision_correction = Vector2::Zero();
  if (behavior) {
    behavior->set_kinematics(kinematics);
    behavior->set_radius(radius);
    behavior->prepare();
    controller.set_behavior(behavior);
    // TODO(J): should be optional now that we added support for external
    // run-loops
    // controller.set_cmd_frame(core::Frame::absolute);
  }
  if (task) {
    task->prepare(this, world);
  }
  ready = true;
}

void Agent::close() {
  if (!ready)
    return;
  if (task) {
    task->close();
  }
  if (behavior) {
    behavior->close();
  }
  if (state_estimation) {
    state_estimation->close();
  }
  ready = false;
}

void Agent::update(ng_float_t dt, ng_float_t time, World *world) {
  if (external)
    return;
  // TODO(J): should update the task anyway to record the logs
  control_deadline -= dt;
  if (control_deadline > 0) {
    return;
  }
  if (behavior) {
    behavior->set_actuated_twist(last_cmd);
    behavior->set_twist(twist);
    behavior->set_pose(pose);
  }
  for(auto & state_estimation : state_estimations) {
    state_estimation->update(this, world);
  }
  if (task)
    task->update(this, world, time);
}

void Agent::update_control(ng_float_t dt, ng_float_t time) {
  if (external)
    return;
  if (control_deadline > 0) {
    return;
  }
  control_deadline += control_period;
  last_cmd = controller.update(std::max(control_period, dt));
  if (behavior) {
    if (behavior->is_stuck() && time > 0) {
      if (is_stuck_since_time < 0) {
        is_stuck_since_time = time;
      }
    } else {
      is_stuck_since_time = -1.0;
    }
  }
}

void Agent::actuate(ng_float_t dt) {
  if (external)
    return;
  actuate(last_cmd, dt);
}

void Agent::actuate(const Twist2 &cmd, ng_float_t dt) {
  if (!kinematics)
    return;
  actuated_cmd = kinematics->feasible_from_current(
      cmd.to_frame(core::Frame::relative, pose),
      twist.to_frame(core::Frame::relative, pose), dt);
  twist = actuated_cmd.to_frame(core::Frame::absolute, pose);
  pose = pose.integrate(twist, dt);
}

void Agent::set_behavior(const std::shared_ptr<Behavior> &value) {
  behavior = value;
  controller.set_behavior(value);
  if (behavior) {
    behavior->set_radius(radius);
    if (!behavior->get_kinematics()) {
      behavior->set_kinematics(kinematics);
    }
  }
}

bool Agent::idle() const {
  return (!task || task->done()) && controller.idle() &&
         (!behavior || !behavior->has_target());
}

Twist2 Agent::get_last_cmd(core::Frame frame) const {
  if (last_cmd.frame == frame) {
    return last_cmd;
  }
  if (behavior) {
    return behavior->to_frame(last_cmd, frame);
  }
  return {};
}

} // namespace navground::sim
