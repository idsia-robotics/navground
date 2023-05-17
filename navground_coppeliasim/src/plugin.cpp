#include "plugin.h"

#include <memory>
#include <optional>
#include <vector>

#include "config.h"
#include "navground/core/behavior.h"
#include "navground/core/common.h"
#include "navground/core/controller_3d.h"
#include "navground/core/kinematics.h"
#include "navground/core/property.h"
#include "navground/core/states/geometric.h"
#include "simPlusPlus/Plugin.h"
#include "stubs.h"

using namespace navground::core;

static std::shared_ptr<navground::core::Kinematics> make_kinematics(
    const kinematics_t &k) {
  auto kinematics = Kinematics::make_type(k.type);
  if (!kinematics) {
    kinematics = std::make_shared<OmnidirectionalKinematics>();
  }
  kinematics->set_max_speed(k.max_speed);
  kinematics->set_max_angular_speed(k.max_angular_speed);
  if (WheeledKinematics *wk = dynamic_cast<WheeledKinematics *>(kinematics.get())) {
    wk->set_axis(k.wheel_axis);
  }
  return kinematics;
}

static vector2_t to_vector2_t(const Vector2 &v) {
  vector2_t value;
  value.x = v[0];
  value.y = v[1];
  return value;
}

static Vector2 from_vector2_t(const vector2_t &v) { return Vector2(v.x, v.y); }

static std::optional<Property::Field> from_property_field_t(
    const property_field_t &value) {
  std::vector<Vector2> vs;
  switch (value.type) {
    case 0:
      return value.bool_value;
    case 1:
      return value.int_value;
    case 2:
      return value.float_value;
    case 3:
      return value.string_value;
    case 4:
      return from_vector2_t(value.vector_value);
    case 5:
      return value.bool_list;
    case 6:
      return value.int_list;
    case 7:
      return value.float_list;
    case 8:
      return value.string_list;
    case 9:
      for (const auto i : value.vector_list) {
        vs.push_back(from_vector2_t(i));
      }
      return vs;
    default:
      std::cerr << "Unknown property_field_t type " << value.type << std::endl;
      return std::nullopt;
  }
}

static property_field_t to_property_field_t(const Property::Field &v) {
  property_field_t value;
  value.type = v.index();
  switch (value.type) {
    case 0:
      value.bool_value = std::get<bool>(v);
      break;
    case 1:
      value.int_value = std::get<int>(v);
      break;
    case 2:
      value.float_value = std::get<float>(v);
      break;
    case 3:
      value.string_value = std::get<std::string>(v);
      break;
    case 4:
      value.vector_value = to_vector2_t(std::get<Vector2>(v));
      break;
    case 5:
      value.bool_list = std::get<std::vector<bool>>(v);
      break;
    case 6:
      value.int_list = std::get<std::vector<int>>(v);
      break;
    case 7:
      value.float_list = std::get<std::vector<float>>(v);
      break;
    case 8:
      value.string_list = std::get<std::vector<std::string>>(v);
      break;
    case 9:
      auto rs = std::get<std::vector<Vector2>>(v);
      for (const auto &i : rs) {
        value.vector_list.push_back(to_vector2_t(i));
      }
      break;
  }
  return value;
}

class Plugin : public sim::Plugin {
 public:
  Plugin() : sim::Plugin(), controllers() {}

  void onStart() {
    if (!registerScriptStuff())
      throw std::runtime_error("script stuff initialization failed");
    setExtVersion("Navground");
    setBuildDate(BUILD_DATE);
  }

  void onSimulationAboutToEnd() { controllers.clear(); }

  Controller3 *controller_at_index(unsigned i) const {
    if (i < controllers.size()) {
      return controllers[i].get();
    }
    return nullptr;
  }

  Behavior *behavior_at_index(unsigned i) const {
    if (i < controllers.size()) {
      return controllers[i]->get_behavior().get();
    }
    return nullptr;
  }

  void Controller(Controller_in *in, Controller_out *out) {
    int handle = controllers.size();
    auto kinematics = make_kinematics(in->kinematics);
    if (!kinematics) {
      out->handle = -1;
      return;
    }
    out->handle = handle;  // TODO(J): add -1 to mark failures
    auto behavior = Behavior::make_type(in->behavior);
    behavior->set_radius(in->radius);
    behavior->set_kinematics(kinematics);
    auto controller = std::make_unique<Controller3>(behavior);
    controllers.push_back(std::move(controller));
  }

  void go_to_position(go_to_position_in *in, go_to_position_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->go_to_position(
          Vector3{in->position[0], in->position[1], in->position[2]},
          in->tolerance);
    }
  }

  void go_to_pose(go_to_pose_in *in, go_to_pose_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->go_to_pose(
          Pose3{{in->position[0], in->position[1], in->position[2]},
                in->orientation},
          in->position_tolerance, in->orientation_tolerance);
    }
  }

  void follow_point(follow_point_in *in, follow_point_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->follow_point(
          Vector3{in->point[0], in->point[1], in->point[2]});
    }
  }

  void follow_pose(follow_pose_in *in, follow_pose_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->follow_pose(
          Pose3{{in->position[0], in->position[1], in->position[2]},
                in->orientation});
    }
  }

  void set_pose(set_pose_in *in, set_pose_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->set_pose(
          Pose3{{in->position[0], in->position[1], in->position[2]},
                in->orientation});
    }
  }

  void set_twist(set_twist_in *in, set_twist_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->set_twist(
          Twist3{{in->velocity[0], in->velocity[1], in->velocity[2]},
                 in->angular_speed});
    }
  }

  void update(update_in *in, update_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      auto twist = controller->update(in->time_step);
      out->velocity = {twist.velocity[0], twist.velocity[1], twist.velocity[2]};
      out->angular_speed = twist.angular_speed;
      out->state = static_cast<int>(controller->get_state());
    }
  }

  // TODO(J): extend to 3dr
  void set_static_obstacles(set_static_obstacles_in *in,
                            set_static_obstacles_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (GeometricState *state =
            dynamic_cast<GeometricState *>(behavior->get_environment_state())) {
      std::vector<Disc> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](const obstacle_t &o) {
                       return Disc(Vector2(o.position[0], o.position[1]),
                                   o.radius);
                     });
      state->set_static_obstacles(obstacles);
    }
  }

  void set_neighbors(set_neighbors_in *in, set_neighbors_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (GeometricState *state =
            dynamic_cast<GeometricState *>(behavior->get_environment_state())) {
      std::vector<Neighbor> obstacles;
      std::transform(in->neighbors.cbegin(), in->neighbors.cend(),
                     std::back_inserter(obstacles), [](const neighbor_t &o) {
                       return Neighbor(
                           Vector2(o.position[0], o.position[1]), o.radius,
                           Vector2(o.velocity[0], o.velocity[1]), o.id);
                     });
      state->set_neighbors(obstacles);
    }
  }

  void set_line_obstacles(set_line_obstacles_in *in,
                          set_line_obstacles_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (GeometricState *state =
            dynamic_cast<GeometricState *>(behavior->get_environment_state())) {
      std::vector<LineSegment> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](line_t o) {
                       return LineSegment(Vector2(o.p1[0], o.p1[1]),
                                          Vector2(o.p2[0], o.p2[1]));
                     });
      state->set_line_obstacles(obstacles);
    }
  }

  void get_state(get_state_in *in, get_state_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      out->state = static_cast<int>(controller->get_state());
    }
  }

  void set_rotation_tau(set_rotation_tau_in *in, set_rotation_tau_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_rotation_tau(in->value);
    }
  }

  void set_horizon(set_horizon_in *in, set_horizon_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_horizon(in->value);
    }
  }

  void set_safety_margin(set_safety_margin_in *in, set_safety_margin_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_safety_margin(in->value);
    }
  }

  void set_optimal_speed(set_optimal_speed_in *in, set_optimal_speed_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_optimal_speed(in->value);
    }
  }

  void set_speed_tolerance(set_speed_tolerance_in *in, set_speed_tolerance_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->set_speed_tolerance(in->value);
    }
  }

  void set_heading_behavior(set_heading_behavior_in *in,
                            set_heading_behavior_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      behavior->set_heading_behavior(static_cast<Behavior::Heading>(in->value));
    }
  }

  void should_be_limited_to_2d(should_be_limited_to_2d_in *in,
                               should_be_limited_to_2d_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->should_be_limited_to_2d(in->value);
    }
  }

  void set_cmd_frame(set_cmd_frame_in *in, set_cmd_frame_out *out) {
    auto controller = controller_at_index(in->handle);
    if (controller) {
      controller->set_cmd_frame(static_cast<navground::core::Frame>(in->value));
    }
  }

  void get_actuated_wheel_speeds(get_actuated_wheel_speeds_in *in,
                                 get_actuated_wheel_speeds_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      out->speeds = behavior->get_actuated_wheel_speeds();
    }
  }

  void set_property(set_property_in *in, set_property_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      auto value = from_property_field_t(in->value);
      if (value) {
        behavior->set(in->name, *value);
      }
    }
  }

  void get_property(get_property_in *in, get_property_out *out) {
    auto behavior = behavior_at_index(in->handle);
    if (behavior) {
      try {
        auto value = behavior->get(in->name);
        out->value = to_property_field_t(value);
      } catch (const std::exception &e) {
      }
    }
  }

 private:
  std::vector<std::unique_ptr<Controller3>> controllers;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
