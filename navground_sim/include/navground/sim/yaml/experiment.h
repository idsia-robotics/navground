#ifndef NAVGROUND_SIM_YAML_EXPERIMENT_H
#define NAVGROUND_SIM_YAML_EXPERIMENT_H

#include <iostream>
#include <memory>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenario.h"
#include "navground/sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

using navground::sim::Experiment;
using navground::sim::Scenario;

namespace YAML {

struct convert_experiment {
  static Node encode(const Experiment& rhs) {
    Node node;
    node["time_step"] = rhs.run_config.time_step;
    node["steps"] = rhs.run_config.steps;
    node["runs"] = rhs.number_of_runs;
    node["save_directory"] = rhs.save_directory.string();
    node["record_time"] = rhs.record_config.time;
    node["record_pose"] = rhs.record_config.pose;
    node["record_twist"] = rhs.record_config.twist;
    node["record_cmd"] = rhs.record_config.cmd;
    node["record_target"] = rhs.record_config.target;
    node["record_collisions"] = rhs.record_config.collisions;
    node["record_safety_violation"] = rhs.record_config.safety_violation;
    node["record_task_events"] = rhs.record_config.task_events;
    node["record_deadlocks"] = rhs.record_config.deadlocks;
    node["record_efficacy"] = rhs.record_config.efficacy;
    node["terminate_when_all_idle_or_stuck"] = rhs.run_config.terminate_when_all_idle_or_stuck;
    node["name"] = rhs.name;
    node["run_index"] = rhs.run_index;
    return node;
  }
  static bool decode(const Node& node, Experiment& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["time_step"]) {
      rhs.run_config.time_step = node["time_step"].as<float>(0.1f);
    }
    if (node["steps"]) {
      rhs.run_config.steps = node["steps"].as<unsigned>(0);
    }
    if (node["runs"]) {
      rhs.number_of_runs = node["runs"].as<unsigned>(1);
    }
    if (node["save_directory"]) {
      rhs.save_directory = node["save_directory"].as<std::string>("");
    }
    if (node["record_time"]) {
      rhs.record_config.time = node["record_time"].as<bool>();
    }
    if (node["record_pose"]) {
      rhs.record_config.pose = node["record_pose"].as<bool>();
    }
    if (node["record_twist"]) {
      rhs.record_config.twist = node["record_twist"].as<bool>();
    }
    if (node["record_cmd"]) {
      rhs.record_config.cmd = node["record_cmd"].as<bool>();
    }
    if (node["record_target"]) {
      rhs.record_config.target = node["record_target"].as<bool>();
    }
    if (node["record_collisions"]) {
      rhs.record_config.collisions = node["record_collisions"].as<bool>();
    }
    if (node["record_safety_violation"]) {
      rhs.record_config.safety_violation = node["record_safety_violation"].as<bool>();
    }
    if (node["record_task_events"]) {
      rhs.record_config.task_events = node["record_task_events"].as<bool>();
    }
    if (node["record_deadlocks"]) {
      rhs.record_config.deadlocks = node["record_deadlocks"].as<bool>();
    }
    if (node["record_efficacy"]) {
      rhs.record_config.efficacy = node["record_efficacy"].as<bool>();
    }
    if (node["name"]) {
      rhs.name = node["name"].as<std::string>();
    }
    if (node["terminate_when_all_idle_or_stuck"]) {
      rhs.run_config.terminate_when_all_idle_or_stuck = node["terminate_when_all_idle_or_stuck"].as<bool>();
    }
    if (node["run_index"]) {
      rhs.run_index = std::max(node["run_index"].as<int>(), 0);
    }
    return true;
  }
};

template <>
struct convert<Experiment> {
  static Node encode(const Experiment& rhs) {
    Node node = convert_experiment::encode(rhs);
    if (rhs.scenario) {
      node["scenario"] = *(rhs.scenario);
      // node["scenario"] = convert_scenario<World>::encode(*(rhs.scenario));
    }
    return node;
  }
  static bool decode(const Node& node, Experiment& rhs) {
    if (convert_experiment::decode(node, rhs)) {
      if (node["scenario"]) {
        rhs.scenario = node["scenario"].as<std::shared_ptr<Scenario>>();
      }
      return true;
    }
    return false;
  }
};

}  // namespace YAML

#endif  // NAVGROUND_SIM_YAML_EXPERIMENT_H
