.. _scenario yaml:

========
Scenario
========

Behavior sampler
================

Sampler of behaviors, replaces types in :ref:`behavior_yaml` with samplers of the same type.

.. schema:: navground.sim.schema.bundle()["$defs"]["behavior_sampler"]

Behavior modulation sampler
===========================

Sampler of behavior modulations, replaces types in :ref:`behavior_modulation_yaml` with samplers of the same type.

.. schema:: navground.sim.schema.bundle()["$defs"]["behavior_modulation_sampler"]

Kinematics sampler
==================

Sampler of kinematics, replaces types in :ref:`kinematics_yaml` with samplers of the same type.


.. schema:: navground.sim.schema.bundle()["$defs"]["kinematics_sampler"]


State estimation sampler
========================

Sampler of state estimations, replaces types in :ref:`state_estimation_yaml` with samplers of the same type.


.. schema:: navground.sim.schema.bundle()["$defs"]["state_estimation_sampler"]

Task sampler
============

Sampler of tasks, replaces types in :ref:`task_yaml` with samplers of the same type.


.. schema:: navground.sim.schema.bundle()["$defs"]["task_sampler"]

.. _group_yaml:

Group
======

Sampler of agents: similar to :ref:`agent_yaml` but uses the sampler schemas to populate the agent components. 

.. schema:: navground.sim.schema.bundle()["$defs"]["group"]

.. note:: Almost all fields in these schemas are samplers with the exception of the ``type`` field of registered components, which is a scalar string.

Example
-------

.. code-block:: yaml

   type: my_agent_type
   name: my_group
   number: 4
   kinematics:
     type: Omni
     # implicit constant
     max_speed: 1.0  
   behavior:
     type: HL
     safety_margin: 
       # explicit constant
       sampler: constant
       value: 0.5    
     # implicit sequence
     tau: [0.1, 0.2, 0.2, 0.1]
   state_estimation:
     type: Bounded
     # explicit sequence
     range: 
       sampler: sequence
       value: [0.5, 1.0, 1.5, 2.0]
   # regular
   radius:
     sampler: regular
     from: 0.1
     to: 0.5
     number: 4
   # grid
   position:
     sampler: regular
     from: [0, 0]
     to: [10, 10]
     number: [2, 2]
   # step
   orientation:
     sampler: regular
     from: 0
     step: 0.1
   control_step:
     # uniform random
     sampler: uniform
     from: 0.1
     to: 0.2

Scenario
========

Scenarios are generators of worlds: similar to :ref:`world_yaml` but using :ref:`group_yaml` instead of a list of `agent_yaml`.

.. schema:: navground.sim.Scenario.schema()

Example
-------

.. code-block:: yaml

   walls:
     - line: [[-1.0, -1.0], [-1.0, 1.0]]
   obstacles:
     - 
       position: [2.0, 0.0]
       radius: 0.5
   groups:
     - type: my_type
       number: 2
       kinematics:
         type: Omni
         max_speed: 1.0
       behavior:
         type: Dummy
       radius: 0.1
       control_period: 0.1

Register
--------

Like all the other components, scenarios have a schema that includes all registered sub-classes: 

.. schema:: navground.sim.Scenario.register_schema()
