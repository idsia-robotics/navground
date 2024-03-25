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
#include "navground/core/yaml/yaml.h"
#include "navground/sim/agent.h"
#include "navground/sim/experiment.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/task.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/world.h"
#include "simPlusPlus/Plugin.h"
#include "stubs.h"
#include "yaml-cpp/yaml.h"

namespace core = navground::core;
namespace nsim = navground::sim;

#if SIM_PROGRAM_VERSION_NB >= 40500
typedef double simFloat;
#else
typedef float simFloat;
#endif

static std::shared_ptr<core::Kinematics> make_kinematics(
    const kinematics_t &k) {
  auto kinematics = core::Kinematics::make_type(k.type);
  if (!kinematics) {
    kinematics = std::make_shared<core::OmnidirectionalKinematics>();
  }
  kinematics->set_max_speed(k.max_speed);
  kinematics->set_max_angular_speed(k.max_angular_speed);
  if (core::WheeledKinematics *wk =
          dynamic_cast<core::WheeledKinematics *>(kinematics.get())) {
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

static std::optional<core::Property::Field> from_property_field_t(
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

static int property_type(const core::Property &property) {
  return property.default_value.index();
}

static property_field_t to_property_field_t(const core::Property::Field &v) {
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
      value.float_value = std::get<ng_float_t>(v);
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
      value.float_list = std::get<std::vector<ng_float_t>>(v);
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
  Plugin() : sim::Plugin(), controllers(), frame(-1) {}

#if SIM_PROGRAM_VERSION_NB < 40600
  void onStart() {
#else
  void onInit() {
#endif
    if (!registerScriptStuff())
      throw std::runtime_error("script stuff initialization failed");
    setExtVersion("Navground");
    setBuildDate(BUILD_DATE);
  }

  void onSimulationAboutToEnd() {
    controllers.clear();
    // if (world) {
    //   std::cout << YAML::dump<nsim::World>(world.get()) << std::endl;
    // }
    if (experiment) {
      if (exp_run) {
        experiment->stop_run(*exp_run);
        exp_run = nullptr;
      }
      experiment->stop();
      experiment.release();
    }
    world = nullptr;
    agents.clear();
    agent_handles.clear();
  }

#if SIM_PROGRAM_VERSION_NB < 40600
  void onModuleHandle(char *customData) {
#else
  void onSimulationBeforeActuation() {
#endif
    if (world) {
      if (experiment && !experiment->is_running()) {
        experiment->start();
        exp_run = &(experiment->init_run(seed, get_world()));
        experiment->start_run(*exp_run);
      }
      const auto dt = simGetSimulationTimeStep();
      // sync the state of all agents from coppeliaSim
      for (const auto &agent : world->get_agents()) {
        // TODO(Jerome): add compatibility with 4.4
        simFloat ps[3];
        simFloat os[3];
        int uid = agent->uid;
        int handle = agent_handles[uid];
        int r = simGetObjectPosition(handle, frame, ps);
        if (r == -1) continue;
        r = simGetObjectOrientation(handle, frame, os);
        if (r == -1) continue;
        const core::Pose2 pose({ps[0], ps[1]}, os[2]);
        if (!has_set_twist[handle]) {
          // const Vector2 velocity = (pose.position - agent->pose.position) /
          // dt; const auto angular_speed =
          //   core::normalize(pose.orientation - agent->pose.orientation) / dt;
          //   agent->twist = core::Twist2(velocity, angular_speed);
          simFloat linearVelocity[3];
          simFloat angularVelocity[3];
          simGetObjectVelocity(handle, linearVelocity, angularVelocity);
          if (abs(linearVelocity[0]) > 20.0 || abs(linearVelocity[1]) > 20.0) {
          } else {
            agent->twist = core::Twist2({linearVelocity[0], linearVelocity[1]},
                                        angularVelocity[2]);
          }
        }
        agent->pose = pose;
        has_set_twist[handle] = false;
      }
      // update the world without physics or collisions
      world->update_dry(dt);
      if (experiment && exp_run) {
        experiment->update_run(*exp_run);
      }
      if (experiment && world->get_step() == experiment->get_steps()) {
        simAddLog("navground", sim_verbosity_scriptwarnings,
                  "Experiment finished ... won't record data anymore");
      }
    }
  }

  core::Controller3 *controller_with_handle(unsigned handle) const {
    if (controllers.count(handle)) {
      return controllers.at(handle).get();
    }
    std::cerr << "No controller with handle " << handle << std::endl;
    return nullptr;
  }

  core::Controller *controller2_with_handle(unsigned handle) const {
    auto agent = agent_with_handle(handle);
    if (agent) {
      return agent->get_controller();
    }
    if (controllers.count(handle)) {
      return controllers.at(handle).get();
    }
    return nullptr;
  }

  core::Behavior *behavior_with_handle(unsigned handle) const {
    auto agent = agent_with_handle(handle);
    if (agent) {
      return agent->get_behavior();
    }
    if (controllers.count(handle)) {
      return controllers.at(handle)->get_behavior().get();
    }
    return nullptr;
  }

  nsim::Agent *agent_with_handle(unsigned handle) const {
    if (agents.count(handle)) {
      return agents.at(handle).get();
    }
    return nullptr;
  }

  void get_agents(get_agents_in *in, get_agents_out *out) {
    for (const auto &[_, k] : agent_handles) {
      out->handles.push_back(k);
    }
  }

  void make_controller(make_controller_in *in, make_controller_out *out) {
    int handle = in->handle;
    auto kinematics = make_kinematics(in->kinematics);
    if (!kinematics) {
      out->handle = -1;
      return;
    }
    auto behavior = core::Behavior::make_type(in->behavior);
    behavior->set_radius(in->radius);
    behavior->set_kinematics(kinematics);
    auto controller = std::make_unique<core::Controller3>(behavior);
    controllers.emplace(handle, std::move(controller));
    out->handle = handle;
  }

  void go_to_position(go_to_position_in *in, go_to_position_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->go_to_position(
          core::Vector3{in->position[0], in->position[1], in->position[2]},
          in->tolerance);
    }
  }

  void go_to_pose(go_to_pose_in *in, go_to_pose_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->go_to_pose(
          core::Pose3{{in->position[0], in->position[1], in->position[2]},
                      in->orientation},
          in->position_tolerance, in->orientation_tolerance);
    }
  }

  void follow_point(follow_point_in *in, follow_point_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->follow_point(
          core::Vector3{in->point[0], in->point[1], in->point[2]});
    }
  }

  void follow_pose(follow_pose_in *in, follow_pose_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->follow_pose(
          core::Pose3{{in->position[0], in->position[1], in->position[2]},
                      in->orientation});
    }
  }

  void follow_velocity(follow_velocity_in *in, follow_velocity_out *out) {
    auto controller = controller2_with_handle(in->handle);
    if (controller) {
      controller->follow_velocity(
          core::Vector2(in->velocity[0], in->velocity[1]));
    }
  }

  // void follow_velocity(follow_velocity_in *in, follow_velocity_out *out) {
  //   auto controller = controller_with_handle(in->handle);
  //   if (controller) {
  //     controller->follow_velocity(
  //         core::Vector3(in->velocity[0], in->velocity[1], in->velocity[2]));
  //   }
  // }

  void set_pose(set_pose_in *in, set_pose_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->set_pose(
          core::Pose3{{in->position[0], in->position[1], in->position[2]},
                      in->orientation});
    }
  }

  void get_pose(get_pose_in *in, get_pose_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      const auto &pose = agent->pose;
      const auto &p = pose.position;
      out->position = {p.x(), p.y()};
      out->orientation = pose.orientation;
      return;
    }
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      const auto pose = behavior->get_pose();
      const auto &p = pose.position;
      // std::cout << p << std::endl;
      out->position = {p.x(), p.y()};
      out->orientation = pose.orientation;
    }
  }

  void get_target(get_target_in *in, get_target_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      const auto &target = agent->get_behavior()->get_target();
      const auto &p = target.position;
      out->point.x = p->x();
      out->point.y = p->y();
    }
  }

  void set_twist(set_twist_in *in, set_twist_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      agent->twist =
          core::Twist2{{in->velocity[0], in->velocity[1]}, in->angular_speed};
      has_set_twist[in->handle] = true;
      return;
    }
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->set_twist(
          core::Twist3{{in->velocity[0], in->velocity[1], in->velocity[2]},
                       in->angular_speed});
    }
  }

  void get_twist(get_twist_in *in, get_twist_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      const auto &twist = agent->twist;
      const auto &v = twist.velocity;
      out->velocity = {v.x(), v.y()};
      out->angular_speed = twist.angular_speed;
      return;
    }
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      const auto twist = behavior->get_twist();
      const auto &v = twist.velocity;
      out->velocity = {v.x(), v.y()};
      out->angular_speed = twist.angular_speed;
    }
  }

  void update(update_in *in, update_out *out) {
    auto controller = controller_with_handle(in->handle);
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
    auto behavior = behavior_with_handle(in->handle);
    if (GeometricState *state = dynamic_cast<core::GeometricState *>(
            behavior->get_environment_state())) {
      std::vector<core::Disc> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](const obstacle_t &o) {
                       return Disc(Vector2(o.position[0], o.position[1]),
                                   o.radius);
                     });
      state->set_static_obstacles(obstacles);
    }
  }

  void set_neighbors(set_neighbors_in *in, set_neighbors_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (core::GeometricState *state = dynamic_cast<core::GeometricState *>(
            behavior->get_environment_state())) {
      std::vector<core::Neighbor> obstacles;
      std::transform(in->neighbors.cbegin(), in->neighbors.cend(),
                     std::back_inserter(obstacles), [](const neighbor_t &o) {
                       return core::Neighbor(
                           core::Vector2(o.position[0], o.position[1]),
                           o.radius,
                           core::Vector2(o.velocity[0], o.velocity[1]), o.id);
                     });
      state->set_neighbors(obstacles);
    }
  }

  void set_line_obstacles(set_line_obstacles_in *in,
                          set_line_obstacles_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (core::GeometricState *state = dynamic_cast<core::GeometricState *>(
            behavior->get_environment_state())) {
      std::vector<core::LineSegment> obstacles;
      std::transform(in->obstacles.cbegin(), in->obstacles.cend(),
                     std::back_inserter(obstacles), [](line_t o) {
                       return core::LineSegment(
                           core::Vector2(o.p1[0], o.p1[1]),
                           core::Vector2(o.p2[0], o.p2[1]));
                     });
      state->set_line_obstacles(obstacles);
      std::cerr << "Set line_obstacles" << std::endl;
    } else {
      std::cerr << "Not set line_obstacles" << std::endl;
    }
  }

  void get_state(get_state_in *in, get_state_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      out->state = static_cast<int>(controller->get_state());
    }
  }

  void set_rotation_tau(set_rotation_tau_in *in, set_rotation_tau_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      behavior->set_rotation_tau(in->value);
    }
  }

  void set_horizon(set_horizon_in *in, set_horizon_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      behavior->set_horizon(in->value);
    }
  }

  void get_horizon(get_horizon_in *in, get_horizon_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      out->value = behavior->get_horizon();
    }
  }

  void set_safety_margin(set_safety_margin_in *in, set_safety_margin_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      behavior->set_safety_margin(in->value);
    }
  }

  void get_safety_margin(get_safety_margin_in *in, get_safety_margin_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      out->value = behavior->get_safety_margin();
    }
  }

  void set_optimal_speed(set_optimal_speed_in *in, set_optimal_speed_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      behavior->set_optimal_speed(in->value);
    }
  }

  void get_optimal_speed(get_optimal_speed_in *in, get_optimal_speed_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      out->value = behavior->get_optimal_speed();
    }
  }

  void set_speed_tolerance(set_speed_tolerance_in *in,
                           set_speed_tolerance_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->set_speed_tolerance(in->value);
    }
  }

  void set_heading_behavior(set_heading_behavior_in *in,
                            set_heading_behavior_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      behavior->set_heading_behavior(static_cast<Behavior::Heading>(in->value));
    }
  }

  void should_be_limited_to_2d(should_be_limited_to_2d_in *in,
                               should_be_limited_to_2d_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->should_be_limited_to_2d(in->value);
    }
  }

  void set_cmd_frame(set_cmd_frame_in *in, set_cmd_frame_out *out) {
    auto controller = controller_with_handle(in->handle);
    if (controller) {
      controller->set_cmd_frame(static_cast<core::Frame>(in->value));
    }
  }

  void get_actuated_wheel_speeds(get_actuated_wheel_speeds_in *in,
                                 get_actuated_wheel_speeds_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      out->speeds = behavior->get_actuated_wheel_speeds();
    }
  }

  void set_behavior_property(set_behavior_property_in *in,
                             set_behavior_property_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      auto value = from_property_field_t(in->value);
      if (value) {
        behavior->set(in->name, *value);
      }
    }
  }

  void get_behavior_property(get_behavior_property_in *in,
                             get_behavior_property_out *out) {
    auto behavior = behavior_with_handle(in->handle);
    if (behavior) {
      try {
        auto value = behavior->get(in->name);
        out->value = to_property_field_t(value);
      } catch (const std::exception &e) {
      }
    }
  }

  void _set_property(_set_property_in *in, _set_property_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      auto value = from_property_field_t(in->value);
      if (value) {
        switch (in->owner) {
          case 0:
            agent->get_behavior()->set(in->name, *value);
            break;
          case 1:
            agent->get_kinematics()->set(in->name, *value);
            break;
          case 2:
            agent->get_state_estimation()->set(in->name, *value);
            break;
          case 3:
            agent->get_task()->set(in->name, *value);
            break;
        }
      }
    }
  }

  void properties(properties_in *in, properties_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      core::Properties properties;
      switch (in->owner) {
        case 0:
          properties = agent->get_behavior()->get_properties();
          break;
        case 1:
          properties = agent->get_kinematics()->get_properties();
          break;
        case 2:
          properties = agent->get_state_estimation()->get_properties();
          break;
        case 3:
          properties = agent->get_task()->get_properties();
          break;
        default:
          return;
      }
      for (const auto &[name, property] : properties) {
        out->properties.push_back(
            {name, property_type(property), property.description});
      }
    }
  }

  void _get_property(_get_property_in *in, _get_property_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      core::Property::Field value;
      switch (in->owner) {
        case 0:
          value = agent->get_behavior()->get(in->name);
          break;
        case 1:
          value = agent->get_kinematics()->get(in->name);
          break;
        case 2:
          value = agent->get_state_estimation()->get(in->name);
          break;
        case 3:
          value = agent->get_task()->get(in->name);
          break;
        default:
          return;
      }
      try {
        out->value = to_property_field_t(value);
      } catch (const std::exception &e) {
      }
    }
  }

  void add_obstacle(add_obstacle_in *in, add_obstacle_out *out) {
    simFloat ps[3];
    int r = simGetObjectPosition(in->handle, frame, ps);
    if (r == -1) return;
    get_world()->add_obstacle(
        nsim::Obstacle{core::Vector2{ps[0], ps[1]}, in->radius});
  }

  void add_wall(add_wall_in *in, add_wall_out *out) {
    get_world()->add_wall(nsim::Wall{core::Vector2{in->p1[0], in->p1[1]},
                                     core::Vector2{in->p2[0], in->p2[1]}});
  }

  void add_agent_from_yaml(add_agent_from_yaml_in *in,
                           add_agent_from_yaml_out *out) {
    // int agent_handle = agent_handles.size();
    YAML::Node node;
    try {
      node = YAML::Load(in->yaml);
    } catch (const YAML::ParserException &e) {
      std::cerr << "[Error] " << e.what() << std::endl;
      out->handle = -1;
      return;
    }
    auto agent = node.as<std::shared_ptr<nsim::Agent>>();
    if (agent) {
      // std::cout << YAML::dump<nsim::Agent>(agent.get()) << std::endl;
      agent_handles[agent->uid] = in->handle;
      has_set_twist[in->handle] = false;
      get_world()->add_agent(agent);
      agents[in->handle] = agent;
      out->handle = in->handle;
    } else {
      out->handle = -1;
    }
  }

  void remove_agent(remove_agent_in *in, remove_agent_out *out) {
    int handle = in->handle;
    auto agent = agent_with_handle(handle);
    if (agent) {
      get_world()->remove_agent_with_uid(agent->uid);
      agent_handles.erase(agent->uid);
      has_set_twist.erase(handle);
      agents.erase(handle);
    }
  }

  void get_last_cmd(get_last_cmd_in *in, get_last_cmd_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      const auto twist =
          agent->get_last_cmd(static_cast<core::Frame>(in->frame));
      out->velocity = {twist.velocity[0], twist.velocity[1], 0.0};
      out->angular_speed = twist.angular_speed;
    }
  }

  void get_last_wheel_cmd(get_last_wheel_cmd_in *in,
                          get_last_wheel_cmd_out *out) {
    auto agent = agent_with_handle(in->handle);
    if (agent) {
      auto twist = agent->get_last_cmd(core::Frame::relative);
      const auto kinematics = agent->get_kinematics();
      if (kinematics && kinematics->is_wheeled()) {
        core::WheeledKinematics *wk =
            dynamic_cast<core::WheeledKinematics *>(kinematics);
        out->speeds = wk->wheel_speeds(twist);
      }
    }
  }

  std::shared_ptr<nsim::World> get_world() {
    if (!world) {
      world = std::make_shared<nsim::World>();
    }
    return world;
  }

  void enable_recording(enable_recording_in *in, enable_recording_out *out) {
    const auto time_step = simGetSimulationTimeStep();
    experiment = std::make_unique<nsim::Experiment>(time_step);
    experiment->name = in->config.name;
    experiment->save_directory = in->config.directory;
    experiment->record_config.time = in->config.time;
    experiment->record_config.pose = in->config.pose;
    experiment->record_config.twist = in->config.twist;
    experiment->record_config.cmd = in->config.cmd;
    experiment->record_config.target = in->config.target;
    experiment->record_config.collisions = in->config.collisions;
    experiment->record_config.safety_violation = in->config.safety_violation;
    experiment->record_config.task_events = in->config.task_events;
    experiment->record_config.deadlocks = in->config.deadlocks;
    experiment->record_config.efficacy = in->config.efficacy;
    seed = in->config.seed;
  }

  void set_frame(set_frame_in *in, set_frame_out *out) { frame = in->handle; }

  void set_lattice(set_lattice_in *in, set_lattice_out *out) {
    get_world()->set_lattice(in->coordinate_index,
                             std::make_tuple(in->from, in->to));
  }

  void get_lattice(get_lattice_in *in, get_lattice_out *out) {
    auto lattice = get_world()->get_lattice(in->coordinate_index);
    if (lattice) {
      out->enabled = true;
      out->from = std::get<0>(*lattice);
      out->to = std::get<1>(*lattice);
    } else {
      out->enabled = false;
    }
  }

 private:
  std::map<int, std::unique_ptr<core::Controller3>> controllers;
  std::shared_ptr<nsim::World> world;
  std::unique_ptr<nsim::Experiment> experiment;
  nsim::ExperimentalRun *exp_run;
  // uid -> handle
  std::map<int, int> agent_handles;
  // handle -> agent
  std::map<int, std::shared_ptr<nsim::Agent>> agents;
  // handle -> value
  std::map<int, bool> has_set_twist;
  int frame;
  int seed;
};

#if SIM_PROGRAM_VERSION_NB < 40600
SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#else
SIM_PLUGIN(Plugin)
#endif

#include "stubsPlusPlus.cpp"
