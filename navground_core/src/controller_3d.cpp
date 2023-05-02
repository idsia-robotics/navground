/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include "navground/core/controller_3d.h"

namespace navground::core {

static bool overlaps(float a1, float a2, float b1, float b2) {
  return b1 <= a2 || a1 <= b2;
}

// TODO(J): filter becomes outdated when z or target z changes!!!

void Controller3::set_neighbors(const std::vector<Neighbor3>& neighbors) {
  if (!behavior) return;
  GeometricState* geometric_state =
      dynamic_cast<GeometricState*>(behavior->get_environment_state());
  if (!geometric_state) return;

  std::vector<Neighbor> neighbors_2d;
  for (const auto& cylinder : neighbors) {
    if (!limit_to_2d && altitude.value_set && cylinder.height > 0) {
      float z1, z2;
      if (altitude.target_set) {
        if (altitude.target < altitude.value) {
          z1 = altitude.target;
          z2 = altitude.value;
        } else {
          z2 = altitude.target;
          z1 = altitude.value;
        }
      } else {
        z1 = z2 = altitude.value;
      }
      if (!overlaps(cylinder.position[2],
                    cylinder.position[2] + cylinder.height, z1, z2)) {
        continue;
      }
    }
    neighbors_2d.push_back(cylinder.neighbor());
  }
  geometric_state->set_neighbors(neighbors_2d);
}

// TODO(Jerome): dry up
void Controller3::set_static_obstacles(const std::vector<Cylinder>& neighbors) {
  if (!behavior) return;
  GeometricState* geometric_state =
      dynamic_cast<GeometricState*>(behavior->get_environment_state());
  if (!geometric_state) return;
  std::vector<Disc> obstacles_2d;
  for (const auto& cylinder : neighbors) {
    if (!limit_to_2d && altitude.value_set && cylinder.height > 0) {
      float z1, z2;
      if (altitude.target_set) {
        if (altitude.target < altitude.value) {
          z1 = altitude.target;
          z2 = altitude.value;
        } else {
          z2 = altitude.target;
          z1 = altitude.value;
        }
      } else {
        z1 = z2 = altitude.value;
      }
      if (!overlaps(cylinder.position[2],
                    cylinder.position[2] + cylinder.height, z1, z2)) {
        continue;
      }
    }
    obstacles_2d.push_back(cylinder.disc());
  }
  geometric_state->set_static_obstacles(obstacles_2d);
}

std::shared_ptr<Action> Controller3::go_to_position(const Vector3& point,
                                                    float tolerance) {
  altitude.target = point[2];
  altitude.mode = SimpleControl::Mode::value;
  altitude.target_set = true;
  return Controller::go_to_position(point.head<2>(), tolerance);
}

std::shared_ptr<Action> Controller3::go_to_pose(const Pose3& pose,
                                                float position_tolerance,
                                                float orientation_tolerance) {
  altitude.target = pose.position[2];
  altitude.target_set = true;
  altitude.mode = SimpleControl::Mode::value;
  return Controller::go_to_pose(pose.project(), position_tolerance,
                                orientation_tolerance);
}

std::shared_ptr<Action> Controller3::follow_point(const Vector3& point) {
  altitude.target = point[2];
  altitude.target_set = true;
  altitude.mode = SimpleControl::Mode::value;
  return Controller::follow_point(point.head<2>());
}

std::shared_ptr<Action> Controller3::follow_pose(const Pose3& pose) {
  altitude.target = pose.position[2];
  altitude.target_set = true;
  altitude.mode = SimpleControl::Mode::value;
  return Controller::follow_pose(pose.project());
}

std::shared_ptr<Action> Controller3::follow_velocity(const Vector3& velocity) {
  altitude.target_speed = velocity[2];
  altitude.target_speed_set = true;
  altitude.mode = SimpleControl::Mode::speed;
  return Controller::follow_velocity(velocity.head<2>());
}

std::shared_ptr<Action> Controller3::follow_twist(const Twist3& twist) {
  altitude.target_speed = twist.velocity[2];
  altitude.target_speed_set = true;
  altitude.mode = SimpleControl::Mode::speed;
  return Controller::follow_twist(twist.project());
}

Twist3 Controller3::update_3d(float time_step) {
  if (action && behavior) {
    action->update(this, time_step);
    if (action->done()) {
      action = nullptr;
      behavior->set_target(Target::Stop());
    }
    Twist2 twist = behavior->compute_cmd(time_step);
    const float cmd_z = limit_to_2d ? 0.0f : altitude.update(time_step);
    Twist3 cmd = Twist3(twist, cmd_z);
    if (cmd_cb_3) {
      (*cmd_cb_3)(cmd);
    }
    return cmd;
  }
  return {};
}

}  // namespace navground::core
