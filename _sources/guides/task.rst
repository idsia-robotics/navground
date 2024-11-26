=====
Tasks
=====


.. todo::

   Explain the relationship with the target. Target are fixed, so you don't need
   to extend them when there is a new target. Tasks are control higher level than behaviors. Task can do anything, not just update the target.
   

:cpp:func:`navground::sim::Task::prepare`
:cpp:func:`navground::sim::Task::update`
:cpp:func:`navground::sim::Task::get_log_size`
:cpp:func:`navground::sim::Task::done`


:cpp:func:`navground::sim::Agent::get_controller`
:cpp:func:`navground::sim::Agent::get_behavior`
:cpp:func:`navground::core::Behavior::set_target`

.. code-block:: c++

   #include "navground/sim/tasks.h"

   namespace sim = navground::sim;

   struct MyTask : public sim::Task {
     // optionally override
     void prepare(sim::Agent *agent, sim::World *world) override {
       // setup at the beginning of a simulation
     }

     // optionally override
     void update(sim::Agent *agent, sim::World *world, ng_float_t time) override {
       // update during the simulation
     }
     // optionally override
     void get_log_size() override {
       // our log size
     }
     // optionally override
     bool done() override {
       // whether we are done
     }
   };


:py:meth:`navground.sim.Task.prepare`
:py:meth:`navground.sim.Task.update`
:py:meth:`navground.sim.Task.get_log_size`
:py:meth:`navground.sim.Task.done`

:py:attr:`navground.sim.Agent.controller`
:py:attr:`navground.sim.Agent.behavior`
:py:attr:`navground.core.Behavior.target`


.. code-block:: python

   from navground import core, sim

   class MyTask(sim.PyTask):

       # optionally override
       def prepare(agent: sim.Agent, word: sim.World) -> None:
           # setup at the beginning of a simulation
           ...

       # optionally override
       def update(agent: sim.Agent, word: sim.World, time: float) -> None:
           # update the during the simulation
           ...

       # optionally override
       def get_log_size() -> None:
           # our log size
           ...

       # optionally override
       def done() -> bool:
           # update the during the simulation
           ...