.. _scenario yaml:

========
Scenario
========

Group
======

Schema
------

.. schema:: navground.sim.schema()["$defs"]["group"]

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
--------

Schema
^^^^^^

.. schema:: navground.sim.Scenario.base_schema()

Example
^^^^^^^

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

.. schema:: navground.sim.Scenario.register_schema()
