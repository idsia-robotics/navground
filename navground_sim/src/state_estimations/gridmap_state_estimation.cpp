/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/gridmap_state_estimation.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/yaml/world.h"

namespace navground::sim {

using navground::core::Properties;
using navground::core::Property;

std::optional<core::GridMap> get_gridmap(core::SensingState &state,
                                         unsigned width, unsigned height,
                                         ng_float_t resolution) {
  const auto data = state.get_buffer("local_gridmap")->get_data<uint8_t>();
  const auto origin = state.get_buffer("origin")->get_data<ng_float_t>();
  if (!data || !origin) {
    // std::cerr << "ERROR\n";
    return std::nullopt;
  }
  const Vector2 p((*origin)[0], (*origin)[1]);
  return core::GridMap(const_cast<uint8_t *>(&(*data)[0]), width, height,
                       resolution, p);
}

void LocalGridMapStateEstimation::encode(YAML::Node &node) const {
  // node["lidars"] = std::map<std::string, StateEstimation>();
  for (const auto &[name, lidar] : _lidars) {
    node["lidars"][name] = *std::static_pointer_cast<StateEstimation>(lidar);
  }
}

void LocalGridMapStateEstimation::decode(const YAML::Node &node) {
  _lidars.clear();
  if (node["lidars"]) {
    auto candidates =
        node["lidars"]
            .as<std::map<std::string, std::shared_ptr<StateEstimation>>>();
    for (const auto &[name, se] : candidates) {
      if (auto lidar = std::dynamic_pointer_cast<LidarStateEstimation>(se)) {
        _lidars.emplace(name, lidar);
        lidar->set_name(name);
      }
    }
  }
}

void LocalGridMapStateEstimation::prepare(Agent *agent, World *world) {
  Sensor::prepare(agent, world);
  if (core::SensingState *_state = get_state(agent)) {
    auto gridmap = get_gridmap(*_state, _width, _height, _resolution);
    gridmap->set_value(128);
  }
  // for (const auto &lidar : _lidars) {
  //   core::SensingState state;
  //   lidar->prepare(state);
  //   _lidar_states.push_back(state);
  // }
}

void LocalGridMapStateEstimation::update(Agent *agent, World *world,
                                         EnvironmentState *state) {

  if (_lidars.empty()) {
    return;
  }
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    bool should_init = _state->get_buffer(field_name) == nullptr;
    if (should_init) {
      Sensor::prepare_state(*_state);
    }
    auto gridmap = get_gridmap(*_state, _width, _height, _resolution);
    if (should_init) {
      gridmap->set_value(128);
    }
    const auto pose = agent->get_pose();
    gridmap->move_center(pose.position);
    const auto origin = gridmap->get_origin();
    _state->get_buffer("origin")->set_data(
        std::valarray<ng_float_t>{origin[0], origin[1]});
    gridmap->set_value_in_rectangle(pose.position -
                                        Vector2{agent->radius, agent->radius},
                                    2 * agent->radius, 2 * agent->radius, 255);
    // gridmap->set_footprint_as_freespace(agent->radius);
    // gridmap->set_values_in_disc(agent->radius, 255);
    //
    // size_t i = 0;
    for (const auto &[name, lidar] : _lidars) {
      // lidar->update(agent, world, &_lidar_states[i]);
      lidar->update(agent, world, _state);
      // const auto ranges =
      //     _lidar_states[i].get_buffer("range")->get_data<ng_float_t>();
      const auto ranges = lidar->read_ranges(*_state);
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
  const auto c0 = gridmap.get_cell_at_position(x0);
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

    // std::cerr << "=> raycast_freespace " << x0 << " -- " << x1 << std::endl;

    const auto c1 = gridmap.get_cell_at_position(x1);
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

const std::string LocalGridMapStateEstimation::type =
    register_type<LocalGridMapStateEstimation>(
        "LocalGridMap",
        Properties{
            {"resolution",
             Property::make(&LocalGridMapStateEstimation::get_resolution,
                            &LocalGridMapStateEstimation::set_resolution,
                            default_resolution, "Resolution [meter/cell]",
                            &YAML::schema::strict_positive)},
            {"width", Property::make(&LocalGridMapStateEstimation::get_width,
                                     &LocalGridMapStateEstimation::set_width,
                                     default_width, "Width [meter]",
                                     &YAML::schema::strict_positive)},
            {"height", Property::make(&LocalGridMapStateEstimation::get_height,
                                      &LocalGridMapStateEstimation::set_height,
                                      default_height, "Height [meter]",
                                      &YAML::schema::strict_positive)}} +
            Sensor::properties);

} // namespace navground::sim
