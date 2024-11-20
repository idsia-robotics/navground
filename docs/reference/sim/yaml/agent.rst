=====
Agent
=====

Schema
======

.. schema:: navground.sim.Agent.schema()

Example
=======

.. code-block:: yaml

   kinematics:
     type: Omni
     max_speed: 1.0
   behavior:
     type: Dummy
   task:
     type: Waypoints
     waypoints: [[1.0, 0.0]]
     tolerance: 0.1
   radius: 0.1
   control_period: 0.1
   type: my_agent
   id: 0




