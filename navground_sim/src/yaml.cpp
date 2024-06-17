/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/yaml/sampling.h"

namespace YAML {

static bool _use_compact_samplers = true;

bool get_use_compact_samplers() { return _use_compact_samplers; }

void set_use_compact_samplers(bool value) { _use_compact_samplers = value; }

}  // namespace YAML
