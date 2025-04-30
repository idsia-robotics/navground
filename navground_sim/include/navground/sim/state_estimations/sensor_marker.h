/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_MARKER_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_MARKER_H_

#include <random>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * @brief      A sensor that report the position of a marker relative to the
 * agent.
 *
 * *Registered properties*:
 *
 *   - `marker_position` (float, \ref get_marker_position)
 *
 *   - `reference_orientation` (str, \ref get_reference_orientation)
 *
 *   - `min_x` (float, \ref get_bounds)
 *
 *   - `min_y` (float, \ref get_bounds)
 *
 *   - `max_x` (float, \ref get_bounds)
 *
 *   - `max_y` (float, \ref get_bounds)
 *
 *   - `include_x` (int, \ref get_include_x)
 *
 *   - `include_y` (int, \ref get_include_y)
 */
struct NAVGROUND_SIM_EXPORT MarkerStateEstimation : public Sensor {
  static const std::string type;

  enum struct ReferenceOrientation {
    /** target orientation */
    target_direction,
    /** agent frame */
    agent,
    /** world frame */
    world
  };
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  marker_position  The marker position in the world frame.
   * @param[in]  reference_orientation  The reference frame used for orientation
   * @param[in]  min_x,min_y,max_x,max_y The bounding box
   * @param[in]  include_x  Whether to include the x-coordinate
   * @param[in]  include_y  Whether to include the y-coordinate
   * @param[in]  name  The name to use as a prefix
   */
  explicit MarkerStateEstimation(
      core::Vector2 marker_position = core::Vector2::Zero(),
      ReferenceOrientation reference_orientation = ReferenceOrientation::agent,
      ng_float_t min_x = -std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t min_y = -std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t max_x = std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t max_y = std::numeric_limits<ng_float_t>::infinity(),
      bool include_x = true, bool include_y = true,
      const std::string &name = "")
      : Sensor(name), _marker_position(marker_position),
        _reference_orientation(reference_orientation), _min_x(min_x),
        _min_y(min_y), _max_x(max_x), _max_y(max_y), _include_x(include_x),
        _include_y(include_y) {}

  virtual ~MarkerStateEstimation() = default;
  /**
   * @brief      Sets the marker position in the world frame.
   *
   * @param[in]  value     The desired value
   */
  void set_marker_position(const core::Vector2 &value) {
    _marker_position = value;
  }
  /**
   * @brief      Sets the reference frame used for orientation.
   *
   * @param[in]  value     The desired value
   */
  void set_reference_orientation(ReferenceOrientation value) {
    _reference_orientation = value;
  }
  /**
   * @brief      Gets the marker position in the world frame.
   * rangings.
   *
   * @return     The position.
   */
  core::Vector2 get_marker_position() const { return _marker_position; }
  /**
   * @brief      Gets the reference frame used for orientation.
   *
   * @return     The orientation frame.
   */
  ReferenceOrientation get_reference_orientation() const {
    return _reference_orientation;
  }
  /**
   * @brief      Gets the reference frame used for orientation.
   *
   * @return     The orientation frame name.
   */
  std::string get_reference_orientation_name() const {
    switch (get_reference_orientation()) {
    case ReferenceOrientation::agent:
      return "agent";
    case ReferenceOrientation::world:
      return "world";
    case ReferenceOrientation::target_direction:
      return "target_direction";
    }
  }
  /**
   * @brief      Sets the reference frame used for orientation.
   *
   * @return     The desired frame name.
   */
  void set_reference_orientation_name(const std::string &name) {
    if (name == "agent") {
      set_reference_orientation(ReferenceOrientation::agent);
      return;
    }
    if (name == "world") {
      set_reference_orientation(ReferenceOrientation::world);
      return;
    }
    if (name == "target_direction") {
      set_reference_orientation(ReferenceOrientation::target_direction);
      return;
    }
  }

  /**
   * @brief      Computes the relative marker pose.
   *
   * @param[in]  value     The pose
   */
  core::Vector2 get_measured_marker_position() const {
    return _measured_marker_position;
  }
  /**
   * @brief      Gets the bounding box
   *
   * @param[in]  value     The bounding box
   */
  BoundingBox get_bounds() const { return {_min_x, _max_x, _min_y, _max_y}; }
  /**
   * @brief      Sets the bounding box
   *
   * @param[in]  value     The desired value
   */
  void set_bounds(const BoundingBox &value) {
    _min_x = value.getMinX();
    _min_y = value.getMinY();
    _max_x = value.getMaxX();
    _max_y = value.getMaxY();
  }

  /**
   * @brief      Gets the lowest bound on the x-coordinate.
   *
   * @return     The lowest x-coordinate.
   */
  ng_float_t get_min_x() const { return _min_x; }
  /**
   * @brief      Gets the lowest bound on the y-coordinate.
   *
   * @return     The lowest y-coordinate.
   */
  ng_float_t get_min_y() const { return _min_y; }
  /**
   * @brief      Gets the highest bound on the x-coordinate.
   *
   * @return     The highest x-coordinate.
   */
  ng_float_t get_max_x() const { return _max_x; }
  /**
   * @brief      Gets the highest bound on the y-coordinate.
   *
   * @return     The highest y-coordinate.
   */
  ng_float_t get_max_y() const { return _max_y; }
  /**
   * @brief      Sets the lowest bound on the x-coordinate.
   *
   * @param[in]  value  The desired value
   */
  void set_min_x(ng_float_t value) { _min_x = value; }
  /**
   * @brief      Sets the lowest bound on the y-coordinate.
   *
   * @param[in]  value  The desired value
   */
  void set_min_y(ng_float_t value) { _min_y = value; }
  /**
   * @brief      Sets the highest bound on the x-coordinate.
   *
   * @param[in]  value  The desired value
   */
  void set_max_x(ng_float_t value) { _max_x = value; }
  /**
   * @brief      Sets the highest bound on the y-coordinate.
   *
   * @param[in]  value  The desired value
   */
  void set_max_y(ng_float_t value) { _max_y = value; }
  /**
   * @brief      Sets whether to include the x-coordinate.
   *
   * @param[in]  value  True to include it.
   */
  void set_include_x(bool value) { _include_x = value; }
  /**
   * @brief      Gets whether to include the x-coordinate.
   *
   * @return     True when including it.
   */
  bool get_include_x() const { return _include_x; }
  /**
   * @brief      Sets whether to include the y-coordinate.
   *
   * @param[in]  value  True to include it.
   */
  void set_include_y(bool value) { _include_y = value; }
  /**
   * @brief      Gets whether to include the y-coordinate.
   *
   * @return     True when including it.
   */
  bool get_include_y() const { return _include_y; }

  /*
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  /**
   * @brief      Runs an update
   *
   * Like \ref update but without writing a \ref core::SensingState.
   *
   * @param      agent  The agent
   * @param      world  The world
   */
  void update_marker(Agent *agent, World *world);
  /**
   * @brief      Reads the marker position from a \ref core::SensingState
   *
   * @param      state  The state
   * @param[in]  name   The namespace of the sensor
   *
   * @return     A position or null if none was found.
   */
  static std::optional<core::Vector2>
  read_marker_position_with_name(core::SensingState &state,
                                 const std::string &name);
  /**
   * @brief      Reads the marker position from a \ref core::SensingState
   *
   * Calls \ref read_marker_position_with_name, passing \ref get_name.
   *
   * @param      state  The state
   *
   * @return     A position or null if none was found.
   */
  std::optional<core::Vector2>
  read_marker_position(core::SensingState &state) const {
    return read_marker_position_with_name(state, get_name());
  }

  /**
   * @private
   */
  Description get_description() const override {
    Description desc;
    if (_include_x) {
      desc[get_field_name("x")] =
          core::BufferDescription::make<ng_float_t>({1}, _min_x, _max_x);
    }
    if (_include_y) {
      desc[get_field_name("y")] =
          core::BufferDescription::make<ng_float_t>({1}, _min_y, _max_y);
    }
    return desc;
  }

private:
  core::Vector2 _measured_marker_position;
  core::Vector2 _marker_position;
  ReferenceOrientation _reference_orientation;
  ng_float_t _min_x, _min_y, _max_x, _max_y;
  bool _include_x;
  bool _include_y;
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_STATE_ESTIMATIONS_MARKER_H_ */
