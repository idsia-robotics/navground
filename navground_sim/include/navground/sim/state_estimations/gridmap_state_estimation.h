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
#include "navground/sim/world.h"

#include "navground/sim/export.h"

namespace navground::sim {

using Array = Eigen::Array<ng_float_t, Eigen::Dynamic, 1>;

/**
 * @brief      A local grid map that integrates scans.
 *
 * *Registered properties*:
 *
 *   - `width` (int, \ref get_width)
 *
 *   - `height` (int, \ref get_height)
 *
 *   - `resolution` (float, \ref get_resolution) [meter / cell]
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
   * @brief      Constructs a new instance.
   *
   * @param[in]  lidars  The lidars sensor
   * @param[in]  width   The map width in pixels
   * @param[in]  height  The map height in pixels
   * @param[in]  resolution  The size of a cell in meters
   * @param[in]  name     The name to use as a prefix
   */
  explicit LocalGridMapStateEstimation(
      std::map<std::string, std::shared_ptr<LidarStateEstimation>> lidars = {},
      unsigned width = default_width, unsigned height = default_height,
      ng_float_t resolution = default_resolution, const std::string &name = "")
      : Sensor(name), _lidars(lidars), _width(width), _height(height),
        _resolution(resolution) {}

  virtual ~LocalGridMapStateEstimation() = default;

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
   * @private
   */
  void update(Agent *agent, World *world, EnvironmentState *state) override;

  /**
   * @private
   */
  void prepare(Agent *agent, World *world) override;

  Description get_description() const override {
    Description ds{{field_name, core::BufferDescription::make<uint8_t>(
                                    {_height, _width}, 0, 255)},
                   {"origin", core::BufferDescription::make<ng_float_t>(
                                  {2}, std::numeric_limits<ng_float_t>::min(),
                                  std::numeric_limits<ng_float_t>::max())}};
    for (const auto &[name, lidar] : _lidars) {
      ds.merge(lidar->get_description());
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
  std::map<std::string, std::shared_ptr<LidarStateEstimation>> _lidars;
  // std::vector<SensingState> _lidar_states;
  unsigned _width;
  unsigned _height;
  ng_float_t _resolution;

  void raycast_freespace(core::GridMap &gridmap, const Vector2 &x0,
                         const std::valarray<ng_float_t> &ranges,
                         ng_float_t begin, ng_float_t delta);
  void add_obstacles(core::GridMap &gridmap, const Vector2 &x0,
                     const std::valarray<ng_float_t> &ranges, ng_float_t begin,
                     ng_float_t delta, ng_float_t max_range);
};

} // namespace navground::sim

#endif // NAVGROUND_SIM_STATE_ESTIMATIONS_GRIDMAP_STATE_ESTIMATION_H_
