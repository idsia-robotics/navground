.. _sim cpp yaml:

====
YAML
====

Similar as for `the core library <core cpp yaml>`_, we provide import/export 
from/to YAML through specializations of :py:class:`struct YAML::convert<T>` in ``navground_sim/yaml``.

In particular, we specialize the conversion from/to the following additional classes:

- :cpp:class:`navground::sim::Task`
- :cpp:class:`navground::sim::StateEstimation`
- :cpp:class:`navground::sim::Agent`
- :cpp:class:`navground::sim::Obstacle`
- :cpp:class:`navground::sim::Wall`
- :cpp:class:`navground::sim::World`
- :cpp:struct:`navground::sim::Sampler`
- :cpp:class:`navground::sim::BehaviorSampler`
- :cpp:class:`navground::sim::KinematicsSampler`
- :cpp:class:`navground::sim::StateEstimationSampler`
- :cpp:class:`navground::sim::TaskSampler`
- :cpp:class:`navground::sim::AgentSampler`
- :cpp:class:`navground::sim::Scenario`
- :cpp:class:`navground::sim::Experiment`

Schema
------

:cpp:expr:`navground::sim` schemas are more complex than :cpp:expr:`navground::core` schemas due to the presence of samplers. 

.. code-block:: cpp
   
   #include "navground/sim/yaml/schema_sim.h"

.. cpp:namespace:: navground::sim

.. doxygenfunction:: navground::sim::bundle_schema()