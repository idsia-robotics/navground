/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_STATE_H_
#define NAVGROUND_CORE_STATE_H_

namespace navground::core {

struct EnvironmentState {
  EnvironmentState() = default;
  virtual ~EnvironmentState() = default;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_STATE_H_
