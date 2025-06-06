#ifndef NAVGROUND_SIM_YAML_EXPERIMENT_H
#define NAVGROUND_SIM_YAML_EXPERIMENT_H

#include <iostream>
#include <memory>

#include "navground/core/types.h"
#include "navground/core/yaml/schema.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenario.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

using navground::sim::Experiment;
using navground::sim::RecordNeighborsConfig;
using navground::sim::RecordSensingConfig;
using navground::sim::Scenario;
using navground::sim::Sensor;
using navground::sim::StateEstimation;

namespace YAML {

template <> struct convert<RecordNeighborsConfig> {
  static Node encode(const RecordNeighborsConfig &rhs) {
    Node node;
    node["enabled"] = rhs.enabled;
    node["number"] = rhs.number;
    node["relative"] = rhs.relative;
    return node;
  }
  static bool decode(const Node &node, RecordNeighborsConfig &rhs) {
    if (node["enabled"]) {
      rhs.enabled = node["enabled"].as<bool>();
    }
    if (node["number"]) {
      rhs.number = node["number"].as<int>();
    }
    if (node["relative"]) {
      rhs.relative = node["relative"].as<bool>();
    }
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["enabled"] = schema::type<bool>();
    node["properties"]["number"] = schema::type<unsigned>();
    node["properties"]["relative"] = schema::type<bool>();
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "record_neighbors_config";
};

template <> struct convert<RecordSensingConfig> {
  static Node encode(const RecordSensingConfig &rhs) {
    Node node;
    if (rhs.sensor) {
      node["sensor"] = dynamic_cast<StateEstimation &>(*rhs.sensor);
    }
    node["name"] = rhs.name;
    if (rhs.agent_indices.size()) {
      node["agent_indices"] = rhs.agent_indices;
    }
    return node;
  }
  static bool decode(const Node &node, RecordSensingConfig &rhs) {
    if (node["sensor"]) {
      auto se = node["sensor"].as<std::shared_ptr<StateEstimation>>();
      if (auto sensor = std::dynamic_pointer_cast<Sensor>(se)) {
        rhs.sensor = sensor;
      }
    }
    if (node["agent_indices"]) {
      rhs.agent_indices = node["agent_indices"].as<std::vector<unsigned>>();
    }
    if (node["name"]) {
      rhs.name = node["name"].as<std::string>();
    }
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["sensor"] = schema::ref<StateEstimation>();
    node["properties"]["name"] = schema::type<std::string>();
    node["properties"]["agent_indices"] = schema::type<std::vector<int>>();
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "record_sensing_config";
};

struct convert_experiment {
  static Node encode(const Experiment &rhs) {
    Node node;
    node["time_step"] = rhs.run_config.time_step;
    node["steps"] = rhs.run_config.steps;
    node["runs"] = rhs.number_of_runs;
    node["save_directory"] = rhs.save_directory.string();
    node["record_time"] = rhs.record_config.time;
    node["record_pose"] = rhs.record_config.pose;
    node["record_twist"] = rhs.record_config.twist;
    node["record_cmd"] = rhs.record_config.cmd;
    node["record_actuated_cmd"] = rhs.record_config.actuated_cmd;
    node["record_target"] = rhs.record_config.target;
    node["record_collisions"] = rhs.record_config.collisions;
    node["record_safety_violation"] = rhs.record_config.safety_violation;
    node["record_task_events"] = rhs.record_config.task_events;
    node["record_deadlocks"] = rhs.record_config.deadlocks;
    node["record_efficacy"] = rhs.record_config.efficacy;
    node["record_world"] = rhs.record_config.world;
    node["use_agent_uid_as_key"] = rhs.record_config.use_agent_uid_as_key;
    if (rhs.record_config.neighbors.enabled) {
      node["record_neighbors"] = rhs.record_config.neighbors;
    }
    if (rhs.record_config.sensing.size()) {
      node["record_sensing"] = rhs.record_config.sensing;
    }
    node["terminate_when_all_idle_or_stuck"] =
        rhs.run_config.terminate_when_all_idle_or_stuck;
    node["name"] = rhs.name;
    node["run_index"] = rhs.run_index;
    node["reset_uids"] = rhs.reset_uids;
    node["record_scenario_properties"] = rhs.record_scenario_properties;
    return node;
  }
  static bool decode(const Node &node, Experiment &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["time_step"]) {
      rhs.run_config.time_step = node["time_step"].as<ng_float_t>(0.1);
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
    if (node["record_actuated_cmd"]) {
      rhs.record_config.actuated_cmd = node["record_actuated_cmd"].as<bool>();
    }
    if (node["record_target"]) {
      rhs.record_config.target = node["record_target"].as<bool>();
    }
    if (node["record_collisions"]) {
      rhs.record_config.collisions = node["record_collisions"].as<bool>();
    }
    if (node["record_safety_violation"]) {
      rhs.record_config.safety_violation =
          node["record_safety_violation"].as<bool>();
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
    if (node["record_world"]) {
      rhs.record_config.world = node["record_world"].as<bool>();
    }
    if (node["record_neighbors"]) {
      rhs.record_config.neighbors =
          node["record_neighbors"].as<RecordNeighborsConfig>();
    }
    if (node["record_sensing"]) {
      rhs.record_config.sensing =
          node["record_sensing"].as<std::vector<RecordSensingConfig>>();
    }
    if (node["name"]) {
      rhs.name = node["name"].as<std::string>();
    }
    if (node["terminate_when_all_idle_or_stuck"]) {
      rhs.run_config.terminate_when_all_idle_or_stuck =
          node["terminate_when_all_idle_or_stuck"].as<bool>();
    }
    if (node["run_index"]) {
      rhs.run_index = std::max(node["run_index"].as<int>(), 0);
    }
    if (node["reset_uids"]) {
      rhs.reset_uids = node["reset_uids"].as<bool>();
    }
    if (node["record_scenario_properties"]) {
      rhs.record_scenario_properties = node["record_scenario_properties"].as<bool>();
    }
    rhs.record_config.use_agent_uid_as_key =
        node["use_agent_uid_as_key"].as<bool>(true);
    return true;
  }
};

template <> struct convert<Experiment> {
  static Node encode(const Experiment &rhs) {
    Node node = convert_experiment::encode(rhs);
    if (rhs.scenario) {
      node["scenario"] = *(rhs.scenario);
      // node["scenario"] = convert_scenario<World>::encode(*(rhs.scenario));
    }
    return node;
  }
  static bool decode(const Node &node, Experiment &rhs) {
    if (convert_experiment::decode(node, rhs)) {
      if (node["scenario"]) {
        rhs.scenario = node["scenario"].as<std::shared_ptr<Scenario>>();
      }
      return true;
    }
    return false;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["time_step"] = schema::type<schema::positive_float>();
    node["properties"]["steps"] = schema::type<unsigned>();
    node["properties"]["runs"] = schema::type<unsigned>();
    node["properties"]["save_directory"] = schema::type<std::string>();
    node["properties"]["record_time"] = schema::type<bool>();
    node["properties"]["record_pose"] = schema::type<bool>();
    node["properties"]["record_twist"] = schema::type<bool>();
    node["properties"]["record_cmd"] = schema::type<bool>();
    node["properties"]["record_actuated_cmd"] = schema::type<bool>();
    node["properties"]["record_target"] = schema::type<bool>();
    node["properties"]["record_collisions"] = schema::type<bool>();
    node["properties"]["record_safety_violation"] = schema::type<bool>();
    node["properties"]["record_task_events"] = schema::type<bool>();
    node["properties"]["record_deadlocks"] = schema::type<bool>();
    node["properties"]["record_efficacy"] = schema::type<bool>();
    node["properties"]["record_world"] = schema::type<bool>();
    node["properties"]["use_agent_uid_as_key"] = schema::type<bool>();
    node["properties"]["record_neighbors"] =
        schema::ref<RecordNeighborsConfig>();
    node["properties"]["record_sensing"] = schema::ref<RecordSensingConfig>();
    node["properties"]["terminate_when_all_idle_or_stuck"] =
        schema::type<bool>();
    node["properties"]["name"] = schema::type<std::string>();
    node["properties"]["run_index"] = schema::type<unsigned>();
    node["properties"]["reset_uids"] = schema::type<bool>();
    node["properties"]["record_scenario_properties"] = schema::type<bool>();
    node["properties"]["scenario"] = schema::ref<Scenario>();
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "experiment";
};

} // namespace YAML

#endif // NAVGROUND_SIM_YAML_EXPERIMENT_H
