/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_LIDAR_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_LIDAR_H_

#include <vector>

#include "navground/core/collision_computation.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

using navground::core::BufferDescription;
using navground::core::CollisionComputation;
using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

/**
 * @brief      A distance scanner.
 *
 * *Registered properties*:
 *
 *   - `range` (float, \ref get_range)
 *
 *   - `start_angle` (float, \ref get_start_angle)
 *
 *   - `field_of_view` (float, \ref get_field_of_view)
 *
 *   - `resolution` (float, \ref get_resolution)
 */
struct NAVGROUND_SIM_EXPORT LidarStateEstimation : public Sensor {
  /**
   * The default range
   */
  inline static const float default_range = 1.0f;
  /**
   * The default start angle [radians]
   */
  inline static const float default_start_angle = -M_PI;
  /**
   * The default field of view [radians]
   */
  inline static const float default_field_of_view = 2 * M_PI;
  /**
   * The default resolution
   */
  inline static const int default_resolution = 100;
  /**
   * The name of the buffer set by the sensor
   */
  inline static const std::string field_name = "range";

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range_  The maximal range of the sensor
   * @param[in]  start_angle_  The starting angle
   * @param[in]  field_of_view_  The field of view
   * @param[in]  resolution_  The number of ranging measurements per scan
   */
  explicit LidarStateEstimation(float range_ = default_range,
                                float start_angle_ = default_start_angle,
                                float field_of_view_ = default_field_of_view,
                                unsigned resolution_ = default_resolution)
      : Sensor(),
        range(range_),
        start_angle(start_angle_),
        field_of_view(field_of_view_),
        resolution(resolution_),
        cc() {}

  virtual ~LidarStateEstimation() = default;

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range(float value) { range = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  float get_range() const { return range; }

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_start_angle(float value) { start_angle = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  float get_start_angle() const { return start_angle; }

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_field_of_view(float value) { field_of_view = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  float get_field_of_view() const { return field_of_view; }

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_resolution(int value) { resolution = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  int get_resolution() const { return resolution; }

  /**
   * @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties =
      Properties{
          {"range", make_property<float, LidarStateEstimation>(
                        &LidarStateEstimation::get_range,
                        &LidarStateEstimation::set_range, default_range,
                        "Maximal range")},
          {"start_angle", make_property<float, LidarStateEstimation>(
                              &LidarStateEstimation::get_start_angle,
                              &LidarStateEstimation::set_start_angle,
                              default_start_angle, "Start angle")},
          {"field_of_view", make_property<float, LidarStateEstimation>(
                                &LidarStateEstimation::get_field_of_view,
                                &LidarStateEstimation::set_field_of_view,
                                default_field_of_view, "Total angle")},
          {"resolution", make_property<int, LidarStateEstimation>(
                             &LidarStateEstimation::get_resolution,
                             &LidarStateEstimation::set_resolution,
                             default_resolution, "Resolution")},
      } +
      StateEstimation::properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) const override;

  Description get_description() const override {
    return {
        {field_name, BufferDescription::make<float>({resolution}, 0.0, range)}};
  }

 private:
  float range;
  float start_angle;
  float field_of_view;
  int resolution;
  CollisionComputation cc;
  inline const static std::string type =
      register_type<LidarStateEstimation>("Lidar");
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_LIDAR_H_ */
