.. _experiment yaml:

==========
Experiment
==========

Schema
======

RecordNeighborsConfig
---------------------

.. schema:: navground.sim.RecordNeighborsConfig.schema()

RecordSensignConfig
-------------------

.. schema:: navground.sim.RecordSensingConfig.schema()

Experiment
----------

.. schema:: navground.sim.Experiment.schema()

Example
=======

.. code-block:: yaml

   time_step: 0.1
   steps: 1000
   runs: 10
   save_directory: data
   record_pose: true
   record_collisions: true
   terminate_when_all_idle_or_stuck: true
   scenario:
     type: Cross
     agent_margin: 0.125
     side: 6
     tolerance: 0.25
     walls:
       - line: [[-1.0, -1.0], [-1.0, 1.0]]
     obstacles:
       - 
         position: [2.0, 0.0]
         radius: 0.5
     groups:
       - behavior:
           type: HL
           horizon: 1
         kinematics:
           type: 2WDiff
           wheel_axis: 0.125
           max_speed: 0.25
         radius: 0.15
         control_period: 0.1
         number: 4

