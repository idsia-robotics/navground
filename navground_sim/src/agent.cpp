/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/agent.h"

namespace navground::sim {

void Agent::prepare(World *world) {
  if (ready) return; 
  if (state_estimation) {
    // state_estimation->world = this;
    state_estimation->prepare(this, world);
  }
  if (task) {
    task->prepare(this, world);
  }
  collision_correction = Vector2::Zero();
  if (behavior) {
    behavior->set_kinematics(kinematics);
    behavior->set_radius(radius);
    controller.set_behavior(behavior);
    // TODO(J): should be optional now that we added support for external
    // run-loops
    controller.set_cmd_frame(core::Frame::absolute);
  }
  ready = true;
}

void Agent::update(ng_float_t dt, ng_float_t time, World *world) {
  if (external) return;
  // TODO(J): should update the task anyway to record the logs
  control_deadline -= dt;
  if (control_deadline > 0) {
    return;
  }
  control_deadline += control_period;

  if (task) task->update(this, world, time);
  if (state_estimation) state_estimation->update(this, world);
  if (behavior) {
    behavior->set_actuated_twist(last_cmd);
    behavior->set_twist(twist);
    behavior->set_pose(pose);
    if (behavior->is_stuck() && time > 0) {
      if (is_stuck_since_time < 0) {
        is_stuck_since_time = time;
      }
    } else {
      is_stuck_since_time = -1.0;
    }
  }
  last_cmd = controller.update(std::max(control_period, dt));
  // last_cmd = behavior->get_actuated_twist(true);
}

void Agent::actuate(ng_float_t dt) {
  if (external) return;
  twist = last_cmd;  // + collision_force / mass * dt;
  pose = pose.integrate(twist, dt);
}

void Agent::actuate(const Twist2 &cmd, ng_float_t dt) {
  if (!kinematics) return;
  last_cmd = kinematics->feasible(cmd.to_frame(kinematics->cmd_frame(), pose));
  twist = last_cmd;
  pose = pose.integrate(cmd, dt);
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
  return (!task || task->done()) && controller.idle();
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

}  // namespace navground::sim
