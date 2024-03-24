/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 */

#ifndef NAVGROUND_SIM_RECORD_H_
#define NAVGROUND_SIM_RECORD_H_

#include <highfive/H5File.hpp>

#include "navground/core/types.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

namespace navground::sim {

/**
 * @brief      The base class for all probes.
 * Probes are injected in the simulation when running an \ref ExperimentalRun
 * where their \ref update method is called at every simulation update.
 *
 * They record numerical data and store it in HDF5 files.
 *
 * Concrete classes should overwrite \ref prepare, \ref update, and \ref
 * finalize.
 */
class BaseProbe {
 public:
  /**
   * The type of recorded data
   */
  using Type = std::variant<float, double, int64_t, int32_t, int16_t, int8_t,
                            uint64_t, uint32_t, uint16_t, uint8_t>;

  /**
   * The shape of the multi-dimensional recorded data
   */
  using Shape = std::vector<size_t>;

  BaseProbe() {}
  virtual ~BaseProbe() = default;
  /**
   * @brief      Called at the begin of a simulation run.
   *
   * @param[in]  world          The simulated world
   * @param[in]  maximal_steps  The maximal steps that will be performed
   */
  virtual void prepare(const World& world, unsigned maximal_steps) {}
  /**
   * @brief      Called at each simulation step
   *
   * @param[in]  world  The simulated world
   */
  virtual void update(const World& world) {}
  /**
   * @brief      Called at end of a simulation run
   *
   * @param[in]  world  The simulated world
   */
  virtual void finalize(const World& world, unsigned steps) {}
  /**
   * @brief      Save to HDF5
   *
   * @private
   *
   * @param[in]  key    The key
   * @param      group  The group
   */
  virtual void save(const std::string& key, HighFive::Group& group) const {}
};

/**
 * @brief      Base class for probes that store multi-dimensional data in
 * vectors.
 */
class NAVGROUND_SIM_EXPORT Probe : public BaseProbe {
 public:
  /**
   * The data container to store the multi-dimensional record
   */
  using Data = std::variant<std::vector<float>, std::vector<double>,
                            std::vector<int64_t>, std::vector<int32_t>,
                            std::vector<int16_t>, std::vector<int8_t>,
                            std::vector<uint64_t>, std::vector<uint32_t>,
                            std::vector<uint16_t>, std::vector<uint8_t>>;

  /**
   * @brief      The constructor
   *
   * @private
   *
   * @param[in]  data  The data
   */
  explicit Probe(const Data& data = {}) : BaseProbe(), _data(data), _steps(0) {}
  /**
   * @brief      Returns the shape of the multi-dimensional record
   *
   * @return     The shape
   */
  virtual Shape shape() const { return {}; }

  /**
   * @brief      Returns the total number of items stored.
   *
   * @return     The size
   */
  size_t size() const;

  /**
   * @brief      Sets the type of data to record.
   * Will clear the data.
   *
   * @tparam     T     The desired numerical type.
   */
  template <typename T>
  void set_type() {
    _data = std::vector<T>();
    _steps = 0;
  }
  /**
   * @private
   */
  void save(const std::string& key, HighFive::Group& group) const override;
  /**
   * @brief      Add an item to the record.
   * The item will be implicitly converted to the current data type, see \ref
   * set_type.
   *
   * @param[in]  value  The value to add
   *
   * @tparam     T      The type of the item. Must be convertible to the current
   * data type.
   */
  template <typename T>
  void push(const T& value) {
    std::visit([value](auto&& arg) { arg.push_back(value); }, _data);
  }
  /**
   * @brief      Append items to the record.
   * The items will be implicitly converted to the current data type, see \ref
   * set_type.
   *
   * @param[in]  values  The values to add
   *
   * @tparam     T       The type of the items. Must be convertible to the
   * current data type.
   */
  template <typename T>
  void append(const std::vector<T>& values) {
    std::visit(
        [&values](auto&& arg) {
          std::copy(values.begin(), values.end(), std::back_inserter(arg));
        },
        _data);
  }

  /**
   * @brief      Increment the steps count.
   */
  void record_step() { _steps++; }

  /**
   * @brief      Returns how many steps have been recorded
   *
   * @return     The steps.
   */
  unsigned get_steps() const { return _steps; }

  /**
   * @brief      Returns the recorded data.
   *
   * @return     The data.
   */
  const Data& get_data() const { return _data; }

  /**
   * @brief      Returns the recorded data.
   *
   *
   * @tparam     T    The desired type.
   *
   * @return     A pointer to the stored data or null if data has a different
   * type than ``T``.
   */
  template <typename T>
  const std::vector<T>* get_typed_data() const {
    return std::get_if<std::vector<T>>(&_data);
  }

 private:
  Data _data;
  unsigned _steps;
};

/**
 * @brief      Base class for probes that store multi-dimensional data in a map.
 *
 * @tparam     K     The type of keys to assign to multi-dimensional data
 */
template <typename K>
class MapProbe : public BaseProbe {
 public:
  /**
   * The type of keys
   */
  using KeyType = K;

  /**
   * @brief      Returns the shape of the multi-dimensional record for a given
   * key
   *
   * @param[in]  key  The key
   *
   * @return     { description_of_the_return_value }
   */
  virtual Shape shape(const K& key) const { return {}; }

  /**
   * The container for the multi-dimensional data map
   */
  using Data = std::variant<
      std::map<K, std::vector<float>>, std::map<K, std::vector<double>>,
      std::map<K, std::vector<int64_t>>, std::map<K, std::vector<int32_t>>,
      std::map<K, std::vector<int16_t>>, std::map<K, std::vector<int8_t>>,
      std::map<K, std::vector<uint64_t>>, std::map<K, std::vector<uint32_t>>,
      std::map<K, std::vector<uint16_t>>, std::map<K, std::vector<uint8_t>>>;

  /**
   * @private
   */
  explicit MapProbe(const Data& data = {})
      : BaseProbe(), _data(data), _steps() {}

  /**
   * @brief      Sets the type of data to record.
   * Will clear the data.
   *
   * @tparam     T     The desired numerical type.
   */
  template <typename T>
  void set_type() {
    _data = std::map<K, std::vector<T>>();
    _steps.clear();
  }

  /**
   * @private
   */
  void save(const std::string& key, HighFive::Group& group) const override {
    auto g = group.createGroup(key);
    std::visit(
        [this, &g](auto&& arg) {
          using T = typename std::remove_reference<
              decltype(arg)>::type::mapped_type::value_type;
          for (const auto [k, v] : arg) {
            if (v.empty()) continue;
            g.createDataSet<T>(std::to_string(k), HighFive::DataSpace(shape(k)))
                .write_raw(v.data());
          }
        },
        _data);
  }
  /**
   * @brief      Add an item to the record for a given key.
   * The item will be implicitly converted to the current data type, see \ref
   * set_type.
   *
   * @param[in]  The key
   * @param[in]  value  The value to add
   *
   * @tparam     T      The type of the item. Must be convertible to the current
   * data type.
   */
  template <typename T>
  void push(const K& key, const T& value) {
    std::visit([&value, &key](auto&& arg) { arg[key].push_back(value); },
               _data);
  }
  /**
   * @brief      Append items to the record for a given key.
   * The items will be implicitly converted to the current data type, see \ref
   * set_type.
   *
   * @param[in]  The key
   * @param[in]  values  The values to add
   *
   * @tparam     T       The type of the items. Must be convertible to the
   * current data type.
   */
  template <typename T>
  void append(const K& key, const std::vector<T>& values) {
    std::visit(
        [&values, &key](auto&& arg) {
          std::copy(values.begin(), values.end(), std::back_inserter(arg[key]));
        },
        _data);
  }

  /**
   * @brief      Returns the total number of items stored for a given key.
   *
   * @param[in]  The key
   *
   * @return     The size
   */
  size_t size(const K& key) const {
    return std::visit(
        [&key](auto&& arg) -> size_t {
          if (!arg.count(key)) return 0;
          return arg.at(key).size();
        },
        _data);
  }

  /**
   * @brief      Increment the steps count for a given key.
   *
   * @param[in]  The key
   */
  void record_step(const K& key) { _steps[key]++; }

  /**
   * @brief      Returns how many steps have been recorded for a given key
   *
   * @param[in]  The key
   *
   * @return     The steps.
   */
  unsigned get_steps(const K& key) const {
    if (!_steps.count(key)) return 0;
    return _steps.at(key);
  }

  /**
   * @brief      Returns the recorded data.
   *
   * @return     The data.
   */
  const Data& get_data() const { return _data; }

  /**
   * @brief      Returns the recorded data.
   *
   * @tparam     T    The desired type.
   *
   * @return     A pointer to the stored data or null if data has a different
   * type than ``T``.
   */
  template <typename T>
  const std::map<K, std::vector<T>>* get_typed_data() const {
    return std::get_if<std::map<K, std::vector<T>>>(&_data);
  }

  std::vector<K> get_keys() const {
    return std::visit(
        [](auto&& arg) {
          std::vector<K> keys;
          std::transform(std::begin(arg), std::end(arg), back_inserter(keys),
                         [](auto&& item) { return item.first; });
          return keys;
        },
        _data);
  }

 private:
  Data _data;
  std::map<unsigned, size_t> _steps;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_RECORD_H_ */
