/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/gridmap_state_estimation.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/yaml/world.h"

namespace navground::sim {

using navground::core::Properties;
using navground::core::Property;

std::optional<core::Pose2>
LocalGridMapStateEstimation::read_transform_with_name(core::SensingState &state,
                                                      const std::string &name) {
  const auto buffer =
      state.get_buffer(Sensor::get_field_name("transformation", name));
  if (!buffer) {
    return std::nullopt;
  }
  const auto data = buffer->get_data<ng_float_t>();
  if (!data || data->size() != 3) {
    return std::nullopt;
  }
  return core::Pose2{{(*data)[0], (*data)[1]}, (*data)[2]};
}

std::optional<core::Pose2>
LocalGridMapStateEstimation::read_transform(core::SensingState &state) const {
  return read_transform_with_name(state, get_name());
}

std::optional<core::GridMap>
LocalGridMapStateEstimation::read_gridmap_with_name(core::SensingState &state,
                                                    const std::string &name) {
  const auto map_buffer =
      state.get_buffer(Sensor::get_field_name("local_gridmap", name));
  if (!map_buffer) {
    return std::nullopt;
  }
  const auto data = map_buffer->get_data<uint8_t>();
  if (!data) {
    return std::nullopt;
  }
  const auto origin_buffer =
      state.get_buffer(Sensor::get_field_name("origin", name));
  if (!origin_buffer) {
    return std::nullopt;
  }
  const auto origin = origin_buffer->get_data<ng_float_t>();
  if (!origin) {
    return std::nullopt;
  }
  const auto resolution_buffer =
      state.get_buffer(Sensor::get_field_name("resolution", name));
  if (!resolution_buffer) {
    return std::nullopt;
  }
  const auto resolution = resolution_buffer->get_data<ng_float_t>();
  if (!resolution) {
    return std::nullopt;
  }
  const auto shape = map_buffer->get_shape();
  if (shape.size() != 2) {
    return std::nullopt;
  }
  const Vector2 p((*origin)[0], (*origin)[1]);
  return core::GridMap(const_cast<uint8_t *>(&(*data)[0]), shape[0], shape[1],
                       (*resolution)[0], p);
}

std::optional<core::GridMap>
LocalGridMapStateEstimation::read_gridmap(core::SensingState &state) const {
  const auto map_buffer =
      state.get_buffer(Sensor::get_field_name("local_gridmap", get_name()));
  if (!map_buffer) {
    return std::nullopt;
  }
  const auto data = map_buffer->get_data<uint8_t>();
  if (!data) {
    return std::nullopt;
  }
  const auto origin_buffer =
      state.get_buffer(Sensor::get_field_name("origin", get_name()));
  if (!origin_buffer) {
    return std::nullopt;
  }
  const auto origin = origin_buffer->get_data<ng_float_t>();
  if (!origin) {
    return std::nullopt;
  }
  const Vector2 p((*origin)[0], (*origin)[1]);
  return core::GridMap(const_cast<uint8_t *>(&(*data)[0]), _width, _height,
                       _resolution, p);
}

void LocalGridMapStateEstimation::encode(YAML::Node &node) const {
  for (const auto &lidar : _internal_lidars) {
    node["lidars"].push_back(*std::static_pointer_cast<StateEstimation>(lidar));
  }
  if (_internal_odometry) {
    node["odometry"] =
        *std::static_pointer_cast<StateEstimation>(_internal_odometry);
  }
}

void LocalGridMapStateEstimation::decode(const YAML::Node &node) {
  _internal_lidars.clear();
  _internal_odometry = nullptr;
  if (node["lidars"]) {
    for (auto c : node["lidars"]) {
      auto se = c.as<std::shared_ptr<StateEstimation>>();
      if (auto lidar = std::dynamic_pointer_cast<LidarStateEstimation>(se)) {
        _internal_lidars.push_back(lidar);
      }
    }
  }
  if (node["odometry"]) {
    auto se = node["odometry"].as<std::shared_ptr<StateEstimation>>();
    _internal_odometry = std::dynamic_pointer_cast<OdometryStateEstimation>(se);
  }
}

void LocalGridMapStateEstimation::prepare_state(
    core::SensingState &state) const {
  Sensor::prepare_state(state);
  read_gridmap(state)->set_value(128);
}

void LocalGridMapStateEstimation::update(Agent *agent, World *world,
                                         EnvironmentState *state) {

  // if (_internal_lidars.empty() && _external_lidars.empty()) {
  //   std::cerr << "No lidars" << std::endl;
  //   return;
  // }
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    if (_state->get_buffer(get_field_name(field_name)) == nullptr) {
      prepare_state(*_state);
    }
    auto gridmap = read_gridmap(*_state);
    const Pose2 pose_world = agent->get_pose();
    Pose2 pose = pose_world;
    bool in_odom = false;
    if (_internal_odometry) {
      _internal_odometry->update_odom(agent, world);
      pose = _internal_odometry->get_pose();
      in_odom = true;
    } else if (!_external_odometry.empty()) {
      const auto maybe_pose = OdometryStateEstimation::read_pose_with_name(
          *_state, _external_odometry);
      if (maybe_pose) {
        pose = *maybe_pose;
      }
      in_odom = true;
    }

    // const auto pose = agent->get_pose();
    gridmap->move_center(pose.position, 128);
    if (_include_transformation) {
      if (in_odom) {
        const Pose2 pose_map{gridmap->get_center(), 0};
        const auto transformation = pose_world * pose_map.inverse();
        get_or_init_buffer(*_state, "transformation")
            ->set_data(std::valarray<ng_float_t>({transformation.position[0],
                                                  transformation.position[1],
                                                  transformation.orientation}));
      } else {
        get_or_init_buffer(*_state, "transformation")
            ->set_data(std::valarray<ng_float_t>({0, 0, 0}));
      }
    }
    const auto origin = gridmap->get_origin();
    get_or_init_buffer(*_state, "origin")
        ->set_data(std::valarray<ng_float_t>({origin[0], origin[1]}));
    get_or_init_buffer(*_state, "resolution")
        ->set_data(std::valarray<ng_float_t>{_resolution});

    if (_footprint == "rectangular") {
      gridmap->set_value_in_rectangle(
          pose.position - Vector2{agent->radius, agent->radius},
          2 * agent->radius, 2 * agent->radius, 255);
    } else if (_footprint == "circular") {
      gridmap->set_value_in_disc(pose.position, agent->radius, 255);
    }

    // gridmap->set_footprint_as_freespace(agent->radius);
    // gridmap->set_values_in_disc(agent->radius, 255);
    //
    // size_t i = 0;
    for (const auto &name : _external_lidars) {
      auto data = LidarStateEstimation::read_scan_with_name(*_state, name);
      if (!data)
        continue;
      // std::string prefix = name.empty() ? "" : name + "/";
      // const auto ranges =
      //     _state->get_buffer(prefix + "range")->get_data<ng_float_t>();
      // const auto start_angle =
      //     _state->get_buffer(prefix +
      //     "start_angle")->get_data<ng_float_t>();
      // const auto fov =
      //     _state->get_buffer(prefix + "fov")->get_data<ng_float_t>();
      // const auto range =
      //     _state->get_buffer(prefix + "max_range")->get_data<ng_float_t>();
      // const auto begin = pose.orientation + (*start_angle)[0];
      // const auto delta = (*fov)[0] / (ranges->size() - 1);
      const auto begin = pose.orientation + data->start_angle;
      const auto delta = data->get_angular_increment();
      raycast_freespace(*gridmap, pose.position, data->ranges, begin, delta);
      add_obstacles(*gridmap, pose.position, data->ranges, begin, delta,
                    data->max_range - 1e-3);
    }

    for (const auto &lidar : _internal_lidars) {
      // lidar->update(agent, world, &_lidar_states[i]);
      // lidar->update(agent, world, _state);
      // const auto ranges =
      //     _lidar_states[i].get_buffer("range")->get_data<ng_float_t>();
      const auto ranges = lidar->measure_ranges(agent, world);
      const auto begin = pose.orientation + lidar->get_start_angle();
      const auto delta = lidar->get_angular_increment();
      raycast_freespace(*gridmap, pose.position, ranges, begin, delta);
      add_obstacles(*gridmap, pose.position, ranges, begin, delta,
                    lidar->get_range() - 1e-3);
      // i++;
    }
  }
}

void LocalGridMapStateEstimation::raycast_freespace(
    core::GridMap &gridmap, const Vector2 &x0,
    const std::valarray<ng_float_t> &ranges, ng_float_t begin_angle,
    ng_float_t delta_angle) {
  // get the map coordinates of the origin of the sensor
  // const Vector2 x0 = Vector2::Zero();
  const Vector2 a = gridmap.get_bottom_left();
  const Vector2 b = gridmap.get_top_right();
  const auto c0 = gridmap.get_possible_cell_at_position(x0);
  const size_t n = ranges.size();
  Eigen::Map<Array> rs(const_cast<ng_float_t *>(&ranges[0]), n);

  const Array angles =
      Array::LinSpaced(n, begin_angle, begin_angle + delta_angle * (n - 1));
  // std::cerr << "ANGLES: " << angles << std::endl;
  // TODO(jerome): try not to be too liberal with free space updates, else it
  // eats up the obstacles. Check what nav2 does.
  const Array xs = angles.cos() * (rs - gridmap.get_resolution());
  const Array ys = angles.sin() * (rs - gridmap.get_resolution());
  // const ng_float_t max_range = 10;
  // const ng_float_t min_range = 0.01;
  // std::cerr << n << std::endl;
  for (size_t i = 0; i < n; i++) {

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    const Vector2 delta = Vector2{xs[i], ys[i]};
    Vector2 x1 = x0 + delta;

    // std::cerr << "raycast_freespace " << x0 << " -- " << x1 << std::endl;

    for (int i = 0; i < 2; ++i) {
      // the minimum value to raytrace from is the origin
      if (x1[i] < a[i]) {
        const auto t = (a[i] - x0[i]) / delta[i];
        x1[i] = a[i];
        x1[1 - i] = x0[1 - i] + delta[1 - i] * t;
      }
      // the maximum value to raytrace to is the end of the map
      if (x1[i] > b[i]) {
        const auto t = (b[i] - x0[i]) / delta[i];
        x1[i] = b[i] - .001;
        x1[1 - i] = x0[1 - i] + delta[1 - i] * t;
      }
    }

    // std::cerr << "=> raycast_freespace " << x0 << " -- " << x1 <<
    // std::endl;

    const auto c1 = gridmap.get_possible_cell_at_position(x1);
    if (!c1) {
      // std::cerr << "ERROR\n";
      continue;
    }
    // std::cerr << "--------" << std::endl  << x0 << std::endl << x1 <<
    // std::endl << *c0 << std::endl << *c1 << std::endl; std::cerr <<
    // "--------" << std::endl;
    gridmap.set_value_between_cells(*c0, *c1, 255);
  }
}
void LocalGridMapStateEstimation::add_obstacles(
    core::GridMap &gridmap, const Vector2 &x0,
    const std::valarray<ng_float_t> &ranges, ng_float_t begin_angle,
    ng_float_t delta_angle, ng_float_t max_range) {
  const size_t n = ranges.size();
  Eigen::Map<Array> rs(const_cast<ng_float_t *>(&ranges[0]), n);
  const Array angles =
      Array::LinSpaced(n, begin_angle, begin_angle + delta_angle * (n - 1));
  const Array xs = angles.cos() * rs;
  const Array ys = angles.sin() * rs;
  // const ng_float_t max_range = 10;
  const ng_float_t min_range = 0.01;
  // std::cerr << n << std::endl;
  for (size_t i = 0; i < n; i++) {
    // std::cerr << rs[i] << std::endl;
    // if the point is far enough away... we won't consider it
    if (rs[i] >= max_range) {
      continue;
    }
    // if the point is too close, do not consider it
    if (rs[i] < min_range) {
      continue;
    }
    gridmap.set_value_at_point(x0 + Vector2{xs[i], ys[i]}, 0);
  }
}

const std::string LocalGridMapStateEstimation::type = register_type<
    LocalGridMapStateEstimation>(
    "LocalGridMap",
    Properties{
        {"external_lidars",
         Property::make(&LocalGridMapStateEstimation::get_external_lidars,
                        &LocalGridMapStateEstimation::set_external_lidars,
                        std::vector<std::string>{},
                        "Name of [external] lidar sensors")},
        {"external_odometry",
         Property::make(&LocalGridMapStateEstimation::get_external_odometry,
                        &LocalGridMapStateEstimation::set_external_odometry,
                        std::string(), "Name of [external] odometry sensor")},
        {"include_transformation",
         Property::make(
             &LocalGridMapStateEstimation::get_include_transformation,
             &LocalGridMapStateEstimation::set_include_transformation, false,
             "Whether to include the transformation between map and world "
             "frame")},
        {"footprint",
         Property::make(
             &LocalGridMapStateEstimation::get_footprint,
             &LocalGridMapStateEstimation::set_footprint, default_footprint,
             "Footprint type: one of \"rectangular\", \"circular\", \"none\"")},
        {"resolution",
         Property::make(&LocalGridMapStateEstimation::get_resolution,
                        &LocalGridMapStateEstimation::set_resolution,
                        default_resolution, "Resolution [meter/cell]",
                        &YAML::schema::strict_positive)},
        {"width",
         Property::make(&LocalGridMapStateEstimation::get_width,
                        &LocalGridMapStateEstimation::set_width, default_width,
                        "Width [meter]", &YAML::schema::strict_positive)},
        {"height", Property::make(&LocalGridMapStateEstimation::get_height,
                                  &LocalGridMapStateEstimation::set_height,
                                  default_height, "Height [meter]",
                                  &YAML::schema::strict_positive)}} +
        Sensor::properties);

} // namespace navground::sim
