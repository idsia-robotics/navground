/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_GRIDMAP_STATE_ESTIMATION_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_GRIDMAP_STATE_ESTIMATION_H_

#include <limits>
#include <map>
#include <string>

#include "navground/core/common.h"
#include "navground/core/states/gridmap.h"
#include "navground/core/types.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/state_estimations/sensor_lidar.h"
#include "navground/sim/state_estimations/sensor_odometry.h"
#include "navground/sim/world.h"

#include "navground/sim/export.h"

namespace navground::sim {

using Array = Eigen::Array<ng_float_t, Eigen::Dynamic, 1>;

/**
 * @brief      A state estimation that integrates scans and optional odometry in
 * a local grid map.
 *
 * Uses one of the simplest mapping algorithms, following the implementation of
 * the obstacle layer in nav2 local costmap, see
 * https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/obstacle_layer.hpp.
 *
 * 1. recenter the map at the agent, setting new cells to 128 (= unknown)
 *
 * 2. set the footprint of the agent to 255 (= free)
 *
 * 3. using raycasting, for each lidar ranging measurement,
 *    sets the internal cells of the ray to 255 (= free),
 *    and the vertex of the ray (if contained in the map) to 0 (= occupied)
 *
 *
 * *Registered properties*:
 *
 *   - `width` (int, \ref get_width)
 *
 *   - `height` (int, \ref get_height)
 *
 *   - `resolution` (float, \ref get_resolution) [meter / cell]
 *
 *   - `include_transformation` (bool, \ref get_include_transformation)
 *
 *   - `external_lidars` (list[str], \ref get_external_lidars)
 *
 *   - `external_odometry` (str, \ref get_external_odometry)
 *
 *   - `footprint` (str, \ref set_footprint)
 */
struct NAVGROUND_SIM_EXPORT LocalGridMapStateEstimation : public Sensor {
  static const std::string type;

  /**
   * The default resolution
   */
  inline static const int default_width = 10;
  inline static const int default_height = 10;
  inline static const ng_float_t default_resolution = 0.1;
  /**
   * The name of the buffer set by the sensor
   */
  inline static const std::string field_name = "local_gridmap";

  /**
   * @brief      The type of footprint to use
   */
  enum struct FootprintType {
    /** a rectangular area */
    rectangular,
    /** a circular are */
    circular,
    /** no footprint */
    none
  };

  inline static const FootprintType default_footprint =
      FootprintType::rectangular;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  lidars  The lidars sensor
   * @param[in]  external_lidars  The name of lidars sensor
   * @param[in]  odometry  The odometry sensor
   * @param[in]  external_odometry  The name of odometry sensor
   * @param[in]  width   The map width in pixels
   * @param[in]  height  The map height in pixels
   * @param[in]  resolution  The size of a cell in meters
   * @param[in]  include_transformation Whether to include
   *             the transformation between map and world frames.
   * @param[in]  footprint Which type of footprint to use.
   * @param[in]  name     The name to use as a prefix
   */
  explicit LocalGridMapStateEstimation(
      std::vector<std::shared_ptr<LidarStateEstimation>> lidars = {},
      std::vector<std::string> external_lidars = {},
      std::shared_ptr<OdometryStateEstimation> odometry = nullptr,
      std::string external_odometry = "", unsigned width = default_width,
      unsigned height = default_height,
      ng_float_t resolution = default_resolution,
      bool include_transformation = false,
      FootprintType footprint = default_footprint, const std::string &name = "")
      : Sensor(name), _internal_lidars(lidars),
        _external_lidars(external_lidars), _internal_odometry(odometry),
        _external_odometry(external_odometry), _width(width), _height(height),
        _resolution(resolution),
        _include_transformation(include_transformation), _footprint(footprint) {
  }

  virtual ~LocalGridMapStateEstimation() = default;

  /**
   * @brief      Reads a grid map from a \ref core::SensingState
   *
   * @param      state  The state
   * @param[in]  name   The namespace of the map
   *
   * @return     A grid map or null if none was found.
   */
  static std::optional<core::GridMap>
  read_gridmap_with_name(core::SensingState &state, const std::string &name);

  /**
   * @brief      Reads a grid map from a \ref core::SensingState
   *
   * Calls \ref read_gridmap_with_name, passing \ref get_name.
   *
   * @param      state  The state
   *
   * @return     A grid map or null if none was found.
   */
  std::optional<core::GridMap> read_gridmap(core::SensingState &state) const;

  /**
   * @brief      Reads a transformation from a \ref core::SensingState
   *
   * @param      state  The state
   * @param[in]  name   The namespace of the transformation
   *
   * @return     A pose or null if none was found.
   */
  static std::optional<core::Pose2>
  read_transform_with_name(core::SensingState &state, const std::string &name);

  /**
   * @brief      Reads a transformation from a \ref core::SensingState
   *
   * Calls \ref read_transform_with_name, passing \ref get_name.
   *
   * @param      state  The state
   *
   * @return     A pose or null if none was found.
   */
  std::optional<core::Pose2> read_transform(core::SensingState &state) const;

  /**
   * @brief      Sets the size of a cell.
   *
   * @param[in]  value     The new value in meters
   */
  void set_resolution(ng_float_t value) { _resolution = value; };

  /**
   * @brief      Gets the size of a cell.
   *
   * @return     The size in meters.
   */
  ng_float_t get_resolution() const { return _resolution; }

  /**
   * @brief      Sets the width of the gridmap in cells.
   *
   * @param[in]  value     The new number of cells.
   */
  void set_width(int value) { _width = value; };

  /**
   * @brief      Gets the width of the gridmap in cells.
   *
   * @return     The number of cells.
   */
  int get_width() const { return _width; }

  /**
   * @brief      Sets the height of the gridmap in cells.
   *
   * @param[in]  value     The new number of cells.
   */
  void set_height(int value) { _height = value; };

  /**
   * @brief      Gets the height of the gridmap in cells.
   *
   * @return     The number of cells.
   */
  int get_height() const { return _height; }

  /**
   * @brief      Sets the names of the (external) lidars.
   *
   * @param[in]  value     The new names.
   */
  void set_external_lidars(const std::vector<std::string> &value) {
    _external_lidars = value;
  };

  /**
   * @brief      Gets the names of the (external) lidars
   *
   * @return     A vector of names.
   */
  const std::vector<std::string> &get_external_lidars() const {
    return _external_lidars;
  }

  /**
   * @brief      Sets the (internal) lidars.
   *
   * @param[in]  value     The new lidars.
   */
  void
  set_lidars(const std::vector<std::shared_ptr<LidarStateEstimation>> &value) {
    _internal_lidars = value;
  };

  /**
   * @brief      Gets the (external) lidars
   *
   * @return     A vector of lidars.
   */
  const std::vector<std::shared_ptr<LidarStateEstimation>> &get_lidars() const {
    return _internal_lidars;
  }

  /**
   * @brief      Sets the names of the (external) odometry.
   *
   * @param[in]  value     The new name.
   */
  void set_external_odometry(const std::string &value) {
    _external_odometry = value;
  };

  /**
   * @brief      Gets the names of the (external) odometry
   *
   * @return     A valid name.
   */
  const std::string &get_external_odometry() const {
    return _external_odometry;
  }

  /**
   * @brief      Sets the (internal) odometry.
   *
   * @param[in]  value     The new odometry.
   */
  void set_odometry(const std::shared_ptr<OdometryStateEstimation> &value) {
    _internal_odometry = value;
  };

  /**
   * @brief      Gets the (external) odometry
   *
   * @return     An odometry sensor.
   */
  const std::shared_ptr<OdometryStateEstimation> &get_odometry() const {
    return _internal_odometry;
  }

  /**
   * @brief      Sets whether to include the transformation
   * between map and world frames.
   *
   * @param[in]  value     The desired value.
   */
  void set_include_transformation(bool value) {
    _include_transformation = value;
  };

  /**
   * @brief      Gets whether to include the transformation
   * between map and world frames.
   *
   * @return     True if it includes the transformation.
   */
  bool get_include_transformation() const { return _include_transformation; }

  /**
   * @brief      Sets footprint type.
   *
   * @param[in]  value  The desired value
   */
  void set_footprint(FootprintType value) { _footprint = value; };

  /**
   * @brief      Gets the footprint type
   *
   * @return     The footprint type
   */
  FootprintType get_footprint() const { return _footprint; }

  /**
   * @brief      Sets footprint type.
   *
   * @param[in]  value  The value: one of "circular", "rectangular", "none"
   */
  void set_footprint_from_string(const std::string &value) {
    if (value == "rectangular") {
      _footprint = FootprintType::rectangular;
    } else if (value == "circular") {
      _footprint = FootprintType::circular;
    } else {
      _footprint = FootprintType::none;
    }
  };

  /**
   * @brief      Gets the footprint type:
   * one of "circular", "rectangular", "none".
   *
   * @return     The footprint type
   */
  std::string get_footprint_as_string() const {
    switch (_footprint) {
    case FootprintType::rectangular:
      return "rectangular";
    case FootprintType::circular:
      return "circular";
    default:
      return "none";
    }
  }

  /**
   * @private
   */
  void update(Agent *agent, World *world, EnvironmentState *state) override;

  /**
   * @private
   */
  void prepare_state(core::SensingState &state) const override;

  /**
   * @private
   */
  Description get_description() const override {
    Description ds{
        {get_field_name(field_name),
         core::BufferDescription::make<uint8_t>({_height, _width}, 0, 255)},
        {get_field_name("origin"),
         core::BufferDescription::make<ng_float_t>(
             {2}, std::numeric_limits<ng_float_t>::min(),
             std::numeric_limits<ng_float_t>::max())},
        {get_field_name("resolution"),
         core::BufferDescription::make<ng_float_t>(
             {1}, 0, std::numeric_limits<ng_float_t>::max())}};
    if (_include_transformation) {
      ds[get_field_name("transformation")] =
          core::BufferDescription::make<ng_float_t>(
              {3}, std::numeric_limits<ng_float_t>::min(),
              std::numeric_limits<ng_float_t>::max());
    }
    return ds;
  }

  /**
   * @private
   */
  void encode(YAML::Node &node) const override;
  /**
   * @private
   */
  void decode(const YAML::Node &node) override;

private:
  std::vector<std::shared_ptr<LidarStateEstimation>> _internal_lidars;
  std::vector<std::string> _external_lidars;
  std::shared_ptr<OdometryStateEstimation> _internal_odometry;
  std::string _external_odometry;
  // std::vector<SensingState> _lidar_states;
  unsigned _width;
  unsigned _height;
  ng_float_t _resolution;
  bool _include_transformation;
  FootprintType _footprint;

  void raycast_freespace(core::GridMap &gridmap, const Vector2 &x0,
                         const std::valarray<ng_float_t> &ranges,
                         ng_float_t begin, ng_float_t delta);
  void add_obstacles(core::GridMap &gridmap, const Vector2 &x0,
                     const std::valarray<ng_float_t> &ranges, ng_float_t begin,
                     ng_float_t delta, ng_float_t max_range);
};

} // namespace navground::sim

#endif // NAVGROUND_SIM_STATE_ESTIMATIONS_GRIDMAP_STATE_ESTIMATION_H_
