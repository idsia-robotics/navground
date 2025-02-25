/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 */

#ifndef NAVGROUND_SIM_DATASET_H_
#define NAVGROUND_SIM_DATASET_H_

#include <highfive/H5File.hpp>
// #include <Eigen/CXX11/Tensor>
#include <unsupported/Eigen/CXX11/Tensor>

#include "navground/core/buffer.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * @brief      Dynamic homogeneous multi-dimensional numerical
 *             data stored in ``std::vector``. Used to record data
 *             collected during an \ref ExperimentalRun.
 */
class NAVGROUND_SIM_EXPORT Dataset {
public:
  /**
   * The data type
   */
  using Scalar = std::variant<float, double, int64_t, int32_t, int16_t, int8_t,
                              uint64_t, uint32_t, uint16_t, uint8_t>;

  /**
   * The data container type
   */
  using Data = std::variant<std::vector<float>, std::vector<double>,
                            std::vector<int64_t>, std::vector<int32_t>,
                            std::vector<int16_t>, std::vector<int8_t>,
                            std::vector<uint64_t>, std::vector<uint32_t>,
                            std::vector<uint16_t>, std::vector<uint8_t>>;
  /**
   * The shape type
   */
  using Shape = std::vector<size_t>;

  /**
   * The indices type
   */
  using Indices = std::vector<size_t>;

  /**
   * @brief      The constructor
   *
   * @private
   *
   * @param[in]  item_shape  The shape of all axis except the first.
   *             Leave empty to instantiate a flat dataset.
   */
  explicit Dataset(const Shape &item_shape = {})
      : _data(), _item_shape(), _item_size(1) {
    set_item_shape(item_shape);
  }

  /**
   * @brief      Make a dataset of a given type
   *
   * @private
   *
   * @param[in]  item_shape  The shape of all axis except the first.
   *             Leave empty to instantiate a flat dataset
   *
   * @tparam     T  The type of data to store
   *
   * @return     A shared pointer to the dataset
   */
  template <typename T>
  static std::shared_ptr<Dataset> make(const Shape &item_shape = {}) {
    auto ds = std::make_shared<Dataset>(item_shape);
    ds->set_dtype<T>();
    return ds;
  }

  /**
   * @brief      Configure this dataset to accumulate data from a buffer
   *
   * @param[in]  buffer  The buffer
   */
  void config_to_hold_buffer(const navground::core::Buffer &buffer);

  // static std::shared_ptr<Dataset> make(const core::Buffer& buffer);

  /**
   * @brief      Returns the shape of the multi-dimensional dataset
   *
   * @return     The shape
   */
  Shape get_shape() const;

  /**
   * @brief      Returns the total number of stored items.
   *
   * @return     The size
   */
  size_t size() const { return get_size(_data); }

  /**
   * @brief      Gets the shape of all axis except the first.
   *
   * @return     The shape.
   */
  Shape get_item_shape() const { return _item_shape; }
  /**
   * @brief      Sets the shape of all axis except the first.
   *
   * @param[in]  item_shape  The desired shape
   */
  void set_item_shape(const Shape &item_shape);

  /**
   * @brief      Determines if valid.
   *
   * @return     True if the number of items fill the shape
   *             exactly, False otherwise.
   */
  bool is_valid() const;

  /**
   * @brief      Clear all data.
   */
  void reset();

  /**
   * @brief      Sets the type of data.
   *
   * Will clear the data if the type is different than the current.
   *
   * @tparam     T     The desired numerical type.
   */
  template <typename T> void set_dtype() {
    if (!std::holds_alternative<std::vector<T>>(_data)) {
      _data = std::vector<T>();
    }
  }

  /**
   * @private
   */
  void save(const std::string &key, HighFive::Group &group) const;

  /**
   * @brief      Add an item.
   *
   * The item will be implicitly converted to
   * the current data type, see \ref set_dtype.
   *
   * @param[in]  value  The value to add
   *
   * @tparam     T      The type of the item.
   *                    Must be convertible to the current data type.
   */
  template <typename T> void push(const T &value) {
    std::visit([value](auto &&arg) { arg.push_back(value); }, _data);
  }

  /**
   * @brief      Add an item.
   *
   * The item will be implicitly converted to
   * the current data type, see \ref set_dtype.
   *
   * @param[in]  value  The value to add
   *
   */
  void push(const Scalar &value);

  /**
   * @brief      Add items.
   *
   * The items will be implicitly converted to
   * the current data type, see \ref set_dtype.
   *
   * @param[in]  values  The values to add
   *
   * @tparam     T       The type of the items.
   *                     Must be convertible to the current data type.
   */
  template <typename T> void append(const std::vector<T> &values) {
    std::visit(
        [&values](auto &&arg) {
          std::copy(values.begin(), values.end(), std::back_inserter(arg));
        },
        _data);
  }

  /**
   * @brief      Add items.
   *
   * The items will be implicitly converted to
   * the current data type, see \ref set_dtype.
   *
   * @param[in]  values  The values to add
   *
   * @tparam     T       The type of the items.
   *                     Must be convertible to the current data type.
   */
  template <typename T> void append(const std::valarray<T> &values) {
    std::visit(
        [&values](auto &&arg) {
          std::copy(std::begin(values), std::end(values),
                    std::back_inserter(arg));
        },
        _data);
  }

  /**
   * @brief      Add items.
   *
   * The items will be implicitly converted to
   * the current data type, see \ref set_dtype.
   *
   * @param[in]  values  The values to add
   *
   */
  void append(const Data &values);

  /**
   * @brief      Add items.
   *
   * The items will be implicitly converted to
   * the current data type, see \ref set_dtype.
   *
   * @param[in]  buffer  The buffer with the values to add
   *
   */
  void append(const core::Buffer &buffer);

  /**
   * @brief      Gets the stored data.
   *
   * @return     The data.
   */
  const Data &get_data() const { return _data; }

  /**
   * @brief      Sets the stored data.
   *
   * @param[in]  data  The data.
   */
  void set_data(const Data &data) { _data = data; }

  /**
   * @brief      Gets the stored data.
   *
   *
   * @tparam     T    The desired type.
   *
   * @return     A pointer to the stored data or null if data has a different
   * type than ``T``.
   */
  template <typename T> const std::vector<T> *get_typed_data() const {
    return std::get_if<std::vector<T>>(&_data);
  }

  /**
   * @brief      Return a tensor view of the dataset
   *
   * @tparam     T    The desired type.
   * @tparam     N    The number of dimensions
   *
   * @return     An eigen tensor with the same shape and type as the dataset
   */
  template <typename T, int N>
  Eigen::TensorMap<Eigen::Tensor<T, N>> const as_tensor() const {
    auto data = get_typed_data<T>();
    T *ptr = const_cast<T *>(data->data());
    std::array<size_t, N> shape;
    auto _shape = get_shape();
    std::copy_n(_shape.rbegin(), N, shape.begin());
    return Eigen::TensorMap<Eigen::Tensor<T, N>>(ptr, shape);
  }

  /**
   * @brief      Writes a buffer with the content of one dataset entry.
   *
   * @param      buffer  The buffer
   * @param[in]  index   The index of the entry
   *
   * @return     True if writing was successful.
   */
  bool write_buffer(core::Buffer *buffer, unsigned index) const {
    if (!buffer || buffer->size() != _item_size) {
      return false;
    }
    if ((index + 1) * _item_size > size()) {
      return false;
    }
    std::visit(
        [this, index, buffer](auto &&arg) {
          using T = std::remove_reference_t<decltype(arg[0])>;
          using U = std::remove_const_t<T>;
          std::valarray<U> bdata(arg.data() + _item_size * index, _item_size);
          buffer->set_data(bdata);
        },
        _data);
    return true;
  }

  /**
   * @brief      Copy one dataset entry to a new buffer
   *
   * @param[in]  index   The index of the entry. Set to -1 to add all entries.
   *
   * @return     The buffer
   */
  core::Buffer get_buffer(int index) const {
    core::BufferShape buffer_shape;
    size_t n = 0;
    size_t d = 0;
    const auto i = get_number_of_items();
    if (i) {
      buffer_shape = core::BufferShape(_item_shape.size());
      std::copy(_item_shape.cbegin(), _item_shape.cend(), buffer_shape.begin());
      index = std::min<int>(index, i);
      if (index < 0) {
        buffer_shape.insert(buffer_shape.begin(), i);
        n = i * _item_size;
      } else {
        n = _item_size;
        d = _item_size * index;
      }
    }
    return std::visit(
        [d, n, buffer_shape](auto &&arg) {
          using T = std::remove_reference_t<decltype(arg[0])>;
          using U = std::remove_const_t<T>;
          return core::Buffer(core::BufferDescription::make<U>(buffer_shape),
                              std::valarray<U>(arg.data() + d, n));
        },
        _data);
  }

  size_t get_number_of_items() const { return _item_size ? size() / _item_size : 0; }

private:
  Data _data;
  Shape _item_shape;
  unsigned _item_size;

  static size_t get_size(const Data &data);
  static size_t get_shape_size(const Shape &data);
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_DATASET_H_ */
