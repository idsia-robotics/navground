#ifndef NAVGROUND_CORE_STATES_SENSING_H
#define NAVGROUND_CORE_STATES_SENSING_H

#include <Eigen/Core>
#include <iostream>
#include <numeric>
#include <vector>

#include "navground/core/buffer.h"
#include "navground/core/common.h"
#include "navground/core/state.h"
#include "navground_core_export.h"

namespace navground::core {

/**
 * @brief      Generic state to hold data from sensors in keyed buffers.
 */
class NAVGROUND_CORE_EXPORT SensingState : virtual public EnvironmentState {
 public:
  /**
   * Construct an instance
   */
  SensingState() : EnvironmentState(), buffers() {}

  virtual ~SensingState() = default;

  /**
   * @brief      Gets the buffers.
   *
   * @return     The buffers.
   */
  const std::map<std::string, Buffer>& get_buffers() const { return buffers; }

  /**
   * @brief      Initializes a buffer with a description and a uniform value.
   *
   * @param[in]  key    The key
   * @param[in]  desc   The description of the buffer
   * @param[in]  value  The value to assign to the buffer
   *
   * @return     The buffer if successfully initialized else a null pointer
   */
  Buffer * init_buffer(const std::string& key, const BufferDescription & desc, BufferType value) {
    auto r = buffers.try_emplace(key, desc, value);
    if (r.second) {
      return &(r.first->second);
    }
    return nullptr;
  }

  /**
   * @brief      Initializes a buffer with description, with data set to zero.
   *
   * @param[in]  key    The key
   * @param[in]  desc   The description of the buffer
   *
   * @return     The buffer if successfully initialized else a null pointer
   */
  Buffer * init_buffer(const std::string& key, const BufferDescription & desc) {
    auto r = buffers.try_emplace(key, desc);
    if (r.second) {
      return &(r.first->second);
    }
    return nullptr;
  }

  /**
   * @brief      Initializes a flat buffer with data.
   *
   * @param[in]  key    The key
   * @param[in]  value  The data
   *
   * @return     The buffer if successfully initialized else a null pointer
   */
  Buffer * init_buffer(const std::string& key, const BufferData & value) {
    auto r = buffers.try_emplace(key, value);
    if (r.second) {
      return &(r.first->second);
    }
    return nullptr;
  }

  /**
   * @brief      Assign a buffer to a key.
   *
   * @param[in]  key    The new value
   * @param[in]  value  The value
   */
  void set_buffer(const std::string& key, const Buffer& value) {
    buffers.insert_or_assign(key, value);
  }

  /**
   * @brief      Gets the buffer assigned to a key
   *
   * @param[in]  key   The key
   *
   * @return     The buffer.
   */
  Buffer * get_buffer(const std::string& key) {
    if (!buffers.count(key)) {
      return nullptr;
    }
    return &buffers.at(key);
  }

 private:
  std::map<std::string, Buffer> buffers;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_STATES_SENSING_H