/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior.h"

namespace navground::core {

class MinimalBehavior : public Behavior {
public:
  static const std::string type;
};

const std::string MinimalBehavior::type =
    register_type<MinimalBehavior>("Minimal", {});

} // namespace navground::core
