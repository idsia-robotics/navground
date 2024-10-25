/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_COMBINE_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_COMBINE_H_

#include <algorithm>
#include <memory>
#include <vector>

#include "navground/core/types.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"
#include "navground/sim/export.h"

namespace navground::sim {

/**
 * @brief      A combination of sensors
 *
 */
struct NAVGROUND_SIM_EXPORT SensorCombination : public Sensor {
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  sensors   The sensors to use
   */
  explicit SensorCombination(
      const std::vector<std::shared_ptr<Sensor>> &sensors = {})
      : Sensor(), _sensors(sensors) {}

  virtual ~SensorCombination() = default;

  /**
   * @brief      Sets the sensors to use.
   *
   * @param[in]  value     The new value
   */
  void set_sensors(const std::vector<std::shared_ptr<Sensor>> &value) {
    _sensors = value;
  }

  /**
   * @brief      Gets the used sensors
   *
   * @return     The sensors
   */
  const std::vector<std::shared_ptr<Sensor>> &get_sensors() const {
    return _sensors;
  }

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  Description get_description() const override;

  /**
   * @private
   */
  void encode(YAML::Node& node) const override;
  /**
   * @private
   */
  void decode(const YAML::Node& node) override;

 private:
  std::vector<std::shared_ptr<Sensor>> _sensors;
  const static std::string type;
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_COMBINE_H_ */
