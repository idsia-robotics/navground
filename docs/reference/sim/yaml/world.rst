.. _world_yaml:

=====
World
=====

Schema
======

Wall
----

.. schema:: navground.sim.Wall.schema()

Obstacle
--------

.. schema:: navground.sim.Obstacle.schema()

World
-----

.. schema:: navground.sim.World.schema()

Example
=======

.. code-block:: yaml

   walls:
     - line: [[-1.0, -1.0], [-1.0, 1.0]]
   obstacles:
     - position: [2.0, 0.0]
       radius: 0.5
   groups:
     - kinematics:
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

