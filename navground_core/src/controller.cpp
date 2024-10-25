/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/controller.h"

namespace navground::core {

ng_float_t Action::tick(Controller *controller,
                        [[maybe_unused]] ng_float_t dt) {
  return controller->estimate_time_until_target_satisfied();
}

void Action::update(Controller *controller, ng_float_t dt) {
  if (state == State::running) {
    const ng_float_t progress = tick(controller, dt);
    if (done()) {
      if (done_cb)
        (*done_cb)(state);
    } else {
      if (running_cb)
        (*running_cb)(progress);
    }
  }
}

void Action::abort() {
  if (state == State::running) {
    state = State::failure;
    if (done_cb)
      (*done_cb)(state);
  }
}

ng_float_t MoveAction::tick(Controller *controller, ng_float_t dt) {
  ng_float_t time = Action::tick(controller, dt);
  if (time == 0 && controller->is_still()) {
    state = State::success;
  }
  return time;
}

std::shared_ptr<Action>
Controller::go_to_position(const Vector2 &point, ng_float_t tolerance,
                           std::optional<Path> along_path) {
  if (action) {
    action->abort();
  }
  if (behavior) {
    behavior->set_target(Target::Point(point, tolerance, along_path));
  }
  action = std::make_shared<MoveAction>();
  action->state = Action::State::running;
  action->update(this, 0.0);
  return action;
}

std::shared_ptr<Action> Controller::go_to_pose(const Pose2 &pose,
                                               ng_float_t position_tolerance,
                                               ng_float_t orientation_tolerance,
                                               std::optional<Path> along_path) {
  if (action) {
    action->abort();
  }
  if (behavior) {
    behavior->set_target(
        Target::Pose(pose, position_tolerance, orientation_tolerance, along_path));
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

std::shared_ptr<Action> Controller::follow_path(const Path &path, ng_float_t tolerance) {
  const auto & p = path.curve(path.length);
  return go_to_position(std::get<0>(p), tolerance, path);
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

Twist2 Controller::update(ng_float_t time_step) {
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

} // namespace navground::core
