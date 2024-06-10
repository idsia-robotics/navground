#ifndef NAVGROUND_CORE_BUFFER_HPP
#define NAVGROUND_CORE_BUFFER_HPP

#include <stdint.h>

#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <valarray>
#include <variant>
#include <vector>

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

namespace navground::core {

/**
 * The shape of a multi-dimensional buffer.
 */
using BufferShape = std::vector<ssize_t>;
/**
 * The type of the data stored in a multi-dimensional buffer.
 */
using BufferType = std::variant<float, double, int64_t, int32_t, int16_t,
                                int8_t, uint64_t, uint32_t, uint16_t, uint8_t>;
/**
 * The data container for multi-dimensional buffer
 */
using BufferData =
    std::variant<std::valarray<float>, std::valarray<double>,
                 std::valarray<int64_t>, std::valarray<int32_t>,
                 std::valarray<int16_t>, std::valarray<int8_t>,
                 std::valarray<uint64_t>, std::valarray<uint32_t>,
                 std::valarray<uint16_t>, std::valarray<uint8_t>>;

/**
 * @brief      Gets the type code corresponding to a numerical type:
 * ``"<kind><number of bytes>"`` (like `numpy`).
 *
 * For example, ``get_type<float>() -> "f4"``)
 *
 * @tparam     T     A numerical type
 *
 * @return     The code.
 */
template <typename T>
inline std::string get_type() {
  if (std::is_floating_point<T>::value) return "f" + std::to_string(sizeof(T));
  if (std::is_integral<T>::value && std::is_unsigned<T>::value)
    return "u" + std::to_string(sizeof(T));
  if (std::is_integral<T>::value && std::is_signed<T>::value)
    return "i" + std::to_string(sizeof(T));
  return "";
}

/**
 * @brief      Gets the type code corresponding to a numerical type:
 * ``"<kind><number of bytes>"`` (like `numpy`).
 *
 * @param[in]  value  A value
 *
 * @return     The code of the value type.
 */
inline std::string get_type(BufferType value) {
  return std::visit(
      [](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        return get_type<T>();
      },
      value);
}

/**
 * @brief      Gets the type code corresponding to a numerical type:
 * ``"<kind><number of bytes>"`` (like `numpy`).
 *
 * @param[in]  data  An array of values
 *
 * @return     The code of the values stored in data.
 */
inline std::string get_type(BufferData data) {
  return std::visit(
      [](auto&& arg) {
        using T = std::remove_reference_t<decltype(arg[0])>;
        return get_type<T>();
      },
      data);
}

/**
 * @brief      Gets a numerical zero of a type with a given code.
 *
 * @param[in]  dtype  The type code
 *
 * @return     Zero.
 */
inline BufferType get_zero(const std::string& dtype) {
  if (dtype == "f4") return static_cast<float>(0);
  if (dtype == "f8") return static_cast<double>(0);
  if (dtype == "i8") return static_cast<int64_t>(0);
  if (dtype == "i4") return static_cast<int32_t>(0);
  if (dtype == "i2") return static_cast<int16_t>(0);
  if (dtype == "i1") return static_cast<int8_t>(0);
  if (dtype == "u8") return static_cast<uint64_t>(0);
  if (dtype == "u4") return static_cast<uint32_t>(0);
  if (dtype == "u2") return static_cast<uint16_t>(0);
  if (dtype == "u1") return static_cast<uint8_t>(0);
  return 0.0;
}

/**
 * @brief      Gets the total number of elements.
 *
 * @param[in]  shape  The shape of a buffer
 *
 * @return     The number of elements for a buffer of a given shape.
 */
inline size_t get_size(const BufferShape& shape) {
  if (!shape.size()) {
    return 0;
  }
  return std::accumulate(std::begin(shape), std::end(shape), 1,
                         std::multiplies<>{});
}

/**
 * @brief      Gets the size of an array
 *
 * @param[in]  data  The data
 *
 * @return     The number of elements in data.
 */
inline size_t get_size(BufferData data) {
  return std::visit([](auto&& arg) { return arg.size(); }, data);
}

/**
 * @brief      Describes a typed, bounded, multi-dimensional buffer.
 *
 * Mimics Gymnasium Box spaces (https://gymnasium.farama.org/api/spaces/).
 */
struct BufferDescription {
  /**
   * The shape of the buffer
   */
  BufferShape shape;
  /**
   * The lower limits
   */
  double low;
  /**
   * The upper limits
   */
  double high;
  /**
   * Whether the [integer] data is categorical or not.
   */
  bool categorical;
  /**
   * A code that identify the type of data.
   */
  std::string type;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  shape        The shape
   * @param[in]  type         The type
   * @param[in]  low          The low
   * @param[in]  high         The high
   * @param[in]  categorical  The categorical
   */
  BufferDescription(const BufferShape& shape, const std::string& type = "",
                    double low = std::numeric_limits<double>::min(),
                    double high = std::numeric_limits<double>::max(),
                    bool categorical = false)
      : shape(shape),
        type(type),
        low(low),
        high(high),
        categorical(categorical) {}

  /**
   * @brief      Constructs a new instance for a given type.
   *
   * @param[in]  shape        The shape
   * @param[in]  low          The low
   * @param[in]  high         The high
   * @param[in]  categorical  The categorical
   *
   * @tparam     T            The type of the data
   *
   * @return     The buffer description.
   */
  template <typename T>
  static BufferDescription make(
      const BufferShape& shape, double low = std::numeric_limits<double>::min(),
      double high = std::numeric_limits<double>::max(),
      bool categorical = false) {
    return BufferDescription(shape, get_type<T>(), low, high, categorical);
  }
};

/**
 * @brief      Describe a flat array of data.
 *
 * @param[in]  data  The data
 *
 * @return     The description.
 */
inline BufferDescription get_description(const BufferData& data) {
  return BufferDescription({static_cast<ssize_t>(get_size(data))},
                           get_type(data));
}

/**
 * @brief      A typed, bounded, multi-dimensional array, similar to ``numpy``
 * arrays.
 */
class Buffer {
 public:
  /**
   * @brief      Constructs a new instance with data set to a uniform value.
   *
   * @param[in]  desc   The description
   * @param[in]  value  The value to assign to all of the buffer.
   */
  Buffer(const BufferDescription& desc, BufferType value) : description(desc) {
    std::visit(
        [this, value](auto&& arg) {
          using T = std::decay_t<decltype(arg)>;
          data = std::valarray<T>(arg, get_described_size());
        },
        value);
    description.type = get_type(value);
  }

  /**
   * @brief      Constructs a new instance, with data set to zero.
   *
   * @param[in]  desc  The description
   */
  Buffer(const BufferDescription& desc) : Buffer(desc, get_zero(desc.type)) {}

  /**
   * @brief      Constructs a new instance with data
   *
   * @param[in]  desc   The description
   * @param[in]  value  The data
   */
  Buffer(const BufferDescription& desc, const BufferData& value)
      : Buffer(desc) {
    set_data(value, true);
  }

  /**
   * @brief      Constructs an unbounded, flat buffer of data
   *
   * @param[in]  value  The data
   */
  Buffer(const BufferData& value)
      : Buffer(::navground::core::get_description(value)) {
    data = value;
  }

  /**
   * @brief      Returns the buffer size
   *
   *
   * @return    The number of elements in the buffer
   */
  size_t size() const { return get_size(data); }

  /**
   * @brief      Return the buffer type
   *
   * @return     The code of type of data stored in the buffer
   */
  std::string type() const { return get_type(data); }

  /**
   * @brief      Gets the data stored in the buffer
   *
   * @tparam     T     The desired type
   *
   * @return     The data or an null pointer if the type is different.
   */
  template <typename T>
  const std::valarray<T>* get_data() const {
    return std::get_if<std::valarray<T>>(&data);
  }

  /**
   * @brief      Gets the data container.
   *
   * @return     The data container.
   */
  const BufferData& get_data_container() const { return data; }

  /**
   * @brief      Determines if the data has a given type
   *
   * @tparam     T     The type
   *
   * @return     True if the data has the given type, False otherwise.
   */
  template <typename T>
  bool has_type() const {
    return get_data<T>() != nullptr;
  }

  /**
   * @brief      Sets the data in the buffer
   *
   * If force is set, it will set the data and possibly change type
   * and shape of the buffer when no compatible. If force is not set, it won't
   * change the data when the shape has a different size or the type is
   * different.
   *
   * @param[in]  value  The new data
   * @param[in]  force  Whenever shape and type should be changed to match the
   * new data.
   *
   * @return     Whether the data was set or not.
   */
  bool set_data(BufferData value, bool force = false) {
    if (value.index() != data.index()) {
      if (force) {
        description.type = get_type(value);
      } else {
        std::cerr << "wrong type " << get_type(value) << ", expected " << type()
                  << std::endl;
        return false;
      }
    }
    if (size() != get_size(value)) {
      if (force) {
        description.shape = {static_cast<ssize_t>(size())};
      } else {
        std::cerr << "wrong size " << get_size(value) << ", expected " << size()
                  << std::endl;
        return false;
      }
    }
    data = value;
    return true;
  }

  /**
   * @brief      Gets the pointer to the data.
   *
   * @return     A pointer to the first element of the buffer.
   */
  const void* get_ptr() const {
    return std::visit(
        [](auto&& arg) -> const void* {
          return static_cast<const void*>(&arg[0]);
        },
        data);
  }

  /**
   * @brief      Sets data from a pointer.
   *
   * If force is set, it will set the data and possibly change type
   * and shape of the buffer when no compatible. If force is not set, it won't
   * change the data when the shape has a different size or the type is
   * different.
   *
   * @param      ptr    A pointer to the first element of the new data
   * @param[in]  shape  The new shape
   * @param[in]  _type  The new type
   * @param[in]  force  Whenever shape and type should be changed to match the
   * new data.
   *
   * @return     Whether the data was set or not.
   */
  bool set_ptr(void* ptr, const BufferShape& shape, const std::string& _type,
               bool force = false) {
    if (!set_type(_type, force)) {
      return false;
    }
    if (!set_shape(shape, force)) {
      return false;
    }
    std::visit(
        [this, ptr](auto&& arg) {
          using T = std::remove_reference_t<decltype(arg[0])>;
          data = std::valarray<T>(static_cast<T*>(ptr), arg.size());
        },
        data);
    return true;
  }

  /**
   * @brief      Gets the description.
   *
   * @return     The description.
   */
  const BufferDescription& get_description() const { return description; }

  /**
   * @brief      Sets the description.
   *
   * If force is set, it will set the description and possibly reset the buffer
   * if the type or size have changed. If force is not set,
   * it won't allow changing shape to a different size or type to a different
   * type.
   *
   * @param[in]  value  The value
   * @param[in]  force  Whenever data may be reset.
   *
   * @return     Whether the description was set or not.
   */
  bool set_description(BufferDescription value, bool force = false) {
    if (!set_type(value.type, force) || !set_shape(value.shape, force)) {
      return false;
    }
    description = value;
    return true;
  }

  /**
   * @brief      Gets the lower bound.
   *
   * @return     The lower bound.
   */
  double get_low() const { return description.low; }

  /**
   * @brief      Gets the upper bound.
   *
   * @return     The upper bound.
   */
  double get_high() const { return description.high; }

  /**
   * @brief      Gets whether buffer is categorical.
   *
   * @return     Whether categorical or not.
   */
  bool get_categorical() const { return description.categorical; }

  /**
   * @brief      Gets the shape.
   *
   * @return     The shape.
   */
  const BufferShape& get_shape() const { return description.shape; }

  /**
   * @brief      Sets the lower bound.
   *
   * @param[in]  value  The value
   */
  void set_low(double value) { description.low = value; }

  /**
   * @brief      Sets the upper bound.
   *
   * @param[in]  value  The value
   */
  void set_high(double value) { description.high = value; }

  /**
   * @brief      Sets whether the buffer is categorical.
   *
   * @param[in]  value  The value
   */
  void set_categorical(bool value) { description.categorical = value; }

  /**
   * @brief      Sets the shape.
   *
   * If force is set, it will set the shape and possibly reset the buffer
   * if the size has changed. If force is not set,
   * it won't allow changing shape to a different size.
   *
   * @param[in]  value  The value
   * @param[in]  force  Whenever data may be reset.
   *
   * @return     Whether the shape was set or not.
   */
  bool set_shape(BufferShape value, bool force = true) {
    if (size() != get_size(value)) {
      if (force) {
        std::visit([n = get_size(value)](auto&& arg) { arg.resize(n); }, data);
      } else {
        std::cerr << "wrong size " << get_size(value) << ", expected " << size()
                  << std::endl;
        return false;
      }
    }
    description.shape = value;
    return true;
  }

  /**
   * @brief      Sets the type.
   *
   * If force is set, it will set the shape and possibly reset the buffer
   * if the type has changed. If force is not set,
   * it won't allow changing type.
   *
   * @param[in]  force  Whenever data may be reset.
   *
   * @tparam     T      The desired type
   *
   * @return     Whether the shape was set or not.
   */
  template <typename T>
  bool set_type(bool force = true) {
    if (!has_type<T>()) {
      if (force) {
        data = std::valarray<T>(T(), get_described_size());
      } else {
        std::cerr << "wrong type " << get_type<T>() << ", expected " << type()
                  << std::endl;
        return false;
      }
    }
    return true;
  }

  /**
   * @brief      Sets the type.
   *
   * If force is set, it will set the shape and possibly reset the buffer
   * if the type has changed. If force is not set,
   * it won't allow changing type.
   *
   * @param[in]  force  Whenever data may be reset.
   * @param[in]  value  The desired type code.
   *
   * @return     Whether the shape was set or not.
   */
  bool set_type(const std::string& value, bool force = true) {
    if (value.empty()) {
      return true;
    }
    if (value != type()) {
      auto zero = get_zero(value);
      if (force) {
        std::visit(
            [this](auto&& arg) {
              using T = std::decay_t<decltype(arg)>;
              data = std::valarray<T>(arg, get_described_size());
            },
            zero);
        description.type = value;
      } else {
        std::cerr << "wrong type " << value << ", expected " << type()
                  << std::endl;
        return false;
      }
    }
    return true;
  }

 private:
  BufferDescription description;
  BufferData data;

  size_t get_described_size() const { return get_size(description.shape); }
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BUFFER_HPP
