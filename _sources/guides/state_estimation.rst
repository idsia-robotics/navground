=================
State Estimations
=================


.. todo::

   Explain the relationship with the type of state. 
   
   - New state -> you need a new SE
   - Specifics SE -> you need a new SE
   - Sensors

:cpp:func:`navground::sim::StateEstimation::prepare`

:cpp:func:`navground::sim::StateEstimation::update`

:cpp:class:`navground::core::EnvironmentState`

.. code-block:: c++

   #include "navground/sim/state_estimation.h"

   namespace sim = navground::sim;
   namespace core = navground::core;

   struct MyStateEstimation : public sim::StateEstimation {
     // optionally override
     void prepare(sim::Agent *agent, sim::World *world) override {
       // setup the state estimation at the begin of a simulation
     }

     // optionally override
     void update(sim::Agent *agent, sim::World *world,
                 core::EnvironmentState *state) override {
       // update the environment state according to the agent and world
     }
   };

:py:meth:`navground.sim.StateEstimation.prepare`
:py:meth:`navground.sim.StateEstimation.update`
:py:class:`navground.core.EnvironmentState`

.. code-block:: python

   from navground import core, sim

   class MyStateEstimation(sim.PyStateEstimation):

       # optionally override
       def prepare(agent: sim.Agent, word: sim.World) -> None:
           # setup the state estimation at the begin of a simulation
           ...

       # optionally override
       def update(agent: sim.Agent, word: sim.World, state: core.EnvironmentState) -> None:
           # update the environment state according to the agent and world
           ...

======
Sensor
======

:cpp:func:`navground::sim::Sensor::get_description`

.. code-block:: c++

   #include "navground/sim/sensor.h"

   namespace sim = navground::sim;

   struct MySensor : public sim::Sensor {
     // in addition to the virtual StateEstimation methods
     // you MUST override
     virtual sim::Sensor::Description get_description() const { 
        return sim::Sensor::Description{...};
     }
   };


:py:meth:`navground.sim.Sensor.get_description`

.. code-block:: python

   from navground import core, sim

   class MyStateEstimation(sim.PyStateEstimation):

       # in addition to the virtual StateEstimation methods
       # you MUST override
       def get_description() -> Dict[str, core.BufferDescription]:
           return {...}

