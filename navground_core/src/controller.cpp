/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/controller.h"

namespace navground::core {

float Action::tick(Controller *controller, [[maybe_unused]] float dt) {
  return controller->estimate_time_until_target_satisfied();
}

void Action::update(Controller *controller, float dt) {
  if (state == State::running) {
    const float progress = tick(controller, dt);
    if (done()) {
      if (done_cb) (*done_cb)(state);
    } else {
      if (running_cb) (*running_cb)(progress);
    }
  }
}

void Action::abort() {
  if (state == State::running) {
    state = State::failure;
    if (done_cb) (*done_cb)(state);
  }
}

float MoveAction::tick(Controller *controller, float dt) {
  float time = Action::tick(controller, dt);
  if (time == 0.0f && controller->is_still()) {
    state = State::success;
  }
  return time;
}

std::shared_ptr<Action> Controller::go_to_position(const Vector2 &point,
                                                   float tolerance) {
  if (action) {
    action->abort();
  }
  if (behavior) {
    behavior->set_target(Target::Point(point, tolerance));
  }
  action = std::make_shared<MoveAction>();
  action->state = Action::State::running;
  action->update(this, 0.0);
  return action;
}

std::shared_ptr<Action> Controller::go_to_pose(const Pose2 &pose,
                                               float position_tolerance,
                                               float orientation_tolerance) {
  if (action) {
    action->abort();
  }
  if (behavior) {
    behavior->set_target(
        Target::Pose(pose, position_tolerance, orientation_tolerance));
  }
  action = std::make_shared<MoveAction>();
  action->state = Action::State::running;
  action->update(this, 0.0);
  return action;
}

std::shared_ptr<Action> Controller::follow_point(const Vector2 &point) {
  if (!std::dynamic_pointer_cast<FollowAction>(action)) {
    if (action) {
      action->abort();
    }
    action = std::make_shared<FollowAction>();
    action->state = Action::State::running;
    action->update(this, 0.0);
  }
  if (behavior) {
    behavior->set_target(Target::Point(point));
  }
  return action;
}

std::shared_ptr<Action> Controller::follow_pose(const Pose2 &pose) {
  if (!std::dynamic_pointer_cast<FollowAction>(action)) {
    if (action) {
      action->abort();
    }
    action = std::make_shared<FollowAction>();
    action->state = Action::State::running;
    action->update(this, 0.0);
  }
  if (behavior) {
    behavior->set_target(Target::Pose(pose));
  }
  return action;
}

std::shared_ptr<Action> Controller::follow_direction(const Vector2 &direction) {
  if (!std::dynamic_pointer_cast<FollowTwistAction>(action)) {
    if (action) {
      action->abort();
    }
    action = std::make_shared<FollowTwistAction>();
    action->state = Action::State::running;
    action->update(this, 0.0);
  }
  if (behavior) {
    behavior->set_target(Target::Direction(direction));
  }
  return action;
}

std::shared_ptr<Action> Controller::follow_velocity(const Vector2 &velocity) {
  if (!std::dynamic_pointer_cast<FollowTwistAction>(action)) {
    if (action) {
      action->abort();
    }
    action = std::make_shared<FollowTwistAction>();
    action->state = Action::State::running;
    action->update(this, 0.0);
  }
  if (behavior) {
    behavior->set_target(Target::Velocity(velocity));
  }
  return action;
}

std::shared_ptr<Action> Controller::follow_twist(const Twist2 &twist) {
  if (!std::dynamic_pointer_cast<FollowTwistAction>(action)) {
    if (action) {
      action->abort();
    }
    action = std::make_shared<FollowTwistAction>();
    action->state = Action::State::running;
    action->update(this, 0.0);
  }
  if (behavior) {
    behavior->set_target(Target::Twist(twist));
  }
  return action;
}

Twist2 Controller::update(float time_step) {
  if (action) {
    action->update(this, time_step);
    if (action && action->done()) {
      action = nullptr;
    }
  }
  if (action && behavior) {
    Twist2 cmd = behavior->compute_cmd(time_step, cmd_frame);
    if (cmd_cb) {
      (*cmd_cb)(cmd);
    }
    return cmd;
  }
  return {};
}

void Controller::stop() {
  if (action) {
    action->abort();
    action = nullptr;
  }
}

}  // namespace navground::core
