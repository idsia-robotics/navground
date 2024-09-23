/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 */

#ifndef NAVGROUND_SIM_PROBE_H_
#define NAVGROUND_SIM_PROBE_H_

#include <highfive/H5File.hpp>

#include "navground/core/types.h"
#include "navground/sim/dataset.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"
#include "navground/sim/export.h"

namespace navground::sim {

class ExperimentalRun;

/**
 * @brief      The base class for all probes.
 *
 * Probes are callbacks called at the begin, at the end,
 * and during each simulation step of an \ref ExperimentalRun.
 *
 * Concrete classes should overwrite \ref prepare, \ref update, and/or \ref
 * finalize as the base class does nothing.
 */
class NAVGROUND_SIM_EXPORT Probe {
 public:
  Probe() {}
  virtual ~Probe() = default;
  /**
   * @brief      Called at the begin of a simulation run.
   *
   * @param[in]  run The run
   */
  virtual void prepare(ExperimentalRun *) {}
  /**
   * @brief      Called at each simulation step
   *
   * @param[in]  run The run
   */
  virtual void update(ExperimentalRun *) {}
  /**
   * @brief      Called at end of a simulation run
   *
   * @param[in]  run The run
   */
  virtual void finalize(ExperimentalRun *) {}
};

/**
 * @brief      Base class for probes that records numerical
 *             data on a single dataset, possibly to be saved
 *             in HDF5.
 *
 * Subclasses are expected to overwrite \ref Probe::update, \ref get_shape
 * and to redefine \ref Type.
 *
 */
class NAVGROUND_SIM_EXPORT RecordProbe : public Probe {
 public:
  /**
   * The type of data to record. Subclasses must define their own type for
   * \ref ExperimentalRun::add_record_probe<T> to work with them.
   */
  using Type = ng_float_t;

  /**
   * @brief      The constructor
   *
   * @private
   *
   * @param[in]  data  The data
   */
  explicit RecordProbe(std::shared_ptr<Dataset> _data) : Probe(), data(_data) {}

  /**
   * @brief      Gets the item shape that the record should use,
   * see \ref Dataset::set_item_shape.
   *
   * Subclasses should overwrite it to configure their records.
   *
   * @param[in]  world  The world being simulated
   *
   * @return     The shape
   */
  virtual Dataset::Shape get_shape(const World &world) const { return {}; }

  /**
   * @private
   */
  void prepare(ExperimentalRun *) override;

  /**
   * The recorded data
   */
  std::shared_ptr<Dataset> data;
};

/**
 * @brief      Base class for probes that record a group of datasets,
 *             possibly to be saved in HDF5. Records are keyed by strings.
 *
 * Subclasses are expected to overwrite \ref Probe::update, \ref get_shapes
 * and to redefine \ref Type.
 */
class NAVGROUND_SIM_EXPORT GroupRecordProbe : public Probe {
 public:
  /**
   * The type of data to record. Subclasses must define their own type for
   * \ref ExperimentalRun::add_group_record_probe<T> to work with them.
   */

  /**
   * Generates dataset for a given key
   */
  using Factory =
      std::function<std::shared_ptr<Dataset>(const std::string &key)>;

  /**
   * A map of shapes.
   */
  using ShapeMap = std::map<std::string, Dataset::Shape>;

  /**
   * @brief      Gets the item shapes that the records should use,
   * see \ref Dataset::set_item_shape.
   *
   * Subclasses should overwrite it to configure their records.
   *
   * @param[in]  world  The world being simulated
   *
   * @return     The shapes
   */
  virtual ShapeMap get_shapes(const World &world) const { return {}; }

  /**
   * @private
   */
  void prepare(ExperimentalRun *) override;

  /**
   * @brief      Gets the recorded data,
   * possibly after instanting a dataset if none is yet associated to the key.
   *
   * @param[in]  key   The key
   *
   * @return     The data.
   */
  std::shared_ptr<Dataset> get_data(const std::string &key) {
    if (!_data.count(key)) {
      _data[key] = _factory(key);
    }
    return _data[key];
  }

  /**
   * @private
   */
  inline static const Factory default_factory = [](const std::string &) {
    return nullptr;
  };

  /**
   * @brief      Constructor
   *
   * @private
   *
   * @param[in]  factory  The datasets generator
   */
  explicit GroupRecordProbe(std::optional<Factory> factory = std::nullopt)
      : Probe(), _factory(factory.value_or(default_factory)), _data() {}

  /**
   * @brief      Sets the datasets generator.
   *
   * @private
   *
   * @param[in]  factory  The factory
   */
  void set_factory(Factory factory) { _factory = factory; }

 private:
  Factory _factory;
  /**
   * The recorded data
   */
  std::map<std::string, std::shared_ptr<Dataset>> _data;
};

/**
 * @brief      A concrete probe to record readings from a specific sensor.
 *
 */
class NAVGROUND_SIM_EXPORT SensingProbe : public Probe {
 public:

  using Data = std::map<unsigned, std::map<std::string, std::shared_ptr<Dataset>>>;

  explicit SensingProbe(const std::string name = "sensing",
                        const std::shared_ptr<Sensor> &sensor = nullptr,
                        const std::vector<unsigned> &agent_indices = {})
      : Probe(),
        _data(),
        _sensor(sensor),
        _states(),
        _agent_indices(agent_indices),
        _name(name) {}

  /**
   * @private
   */
  void prepare(ExperimentalRun *) override;

  /**
   * @private
   */
  void update(ExperimentalRun *run) override;

  /**
   * @brief      Gets the stored sensor readings, indexed by UID or index.
   *
   * @return     The data.
   */
  const Data & get_data() const {
    return _data;
  }

 private:
  Data _data;
  std::shared_ptr<Sensor> _sensor;
  std::map<unsigned, core::SensingState> _states;
  std::vector<unsigned> _agent_indices;
  std::string _name;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_PROBE_H_ */
