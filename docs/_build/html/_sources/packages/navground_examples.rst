==================
navground_examples
==================

A packages with examples of using navground from C++.

Behaviors
=========

:directory:  `src/behavior <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/behavior>`_

Rely only on ``navground_core``.

.. _basics_behavior:

Basics 
------

:file:  `behavior.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/behavior/behavior.cpp>`_

How to instantiate and apply a behavior to avoid a circular obstacle.

Component 
---------

:file: `my_behavior.h <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/behavior/my_behavior.h>`_

How to define and register a new behavior.

Benchmark 
---------

:file: `benchmark.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/behavior/benchmark.cpp>`_

Measures the computation time is requires to move 20 agents in a crossings for 1000 steps.


YAML 
----

:file: `echo_yaml.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/behavior/echo_yaml.cpp>`_


How to load a behavior from YAML and how to dump it to YAML back.


Collisions
==========

:directory: `collision <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/collision>`_

Rely only on ``navground_core``.

Basics
------

:file:  `collision.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/collision/collision.cpp>`_


How to compute free distance to collision with a set of obstacles.

Cache
-----

:file: `cached_collision.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/collision/cached_collision.cpp>`_


How to compute free distance to collision with a set of obstacles and save it to a cache.

Controllers
===========

:directory: `controller <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/controller>`_

Rely only on ``navground_core``.

.. _basics_controller:

Basics
------

:file: `controller.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/controller/controller.cpp>`_


How to use the controller API to move to a target point.

Three dimensional
-----------------

:file: `controller_3d.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/controller/controller_3d.cpp>`_

How to use the 3D controller API to move to a 3D target point.

Asynchronous
------------

:file: `controller_async.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/controller/controller_async.cpp>`_

How to use the action callbacks to move back and forth between two way-points.

Scenarios
=========

:directory: `scenario <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/scenario>`_

Rely on ``navground_core`` and ``navground_sim``. 

Component
---------

:file: `my_scenario.h <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/scenario/my_scenario.h>`_

How to register a new scenario.

Register
--------

:file:  `scenario_registered.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/scenario/scenario_registered.cpp>`_

How to list registered scenarios and load one of them, and use it to sample a world.

YAML
----

:file:  `scenario_from_YAML.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/scenario/scenario_from_yaml.cpp>`_

How to load a scenario from YAML and use it to sample a world.

Groups
------

:file:  `scenario_with_groups.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/scenario/scenario_with_group.cpp>`_

How to define a scenario with groups.


Benchmark
---------

:file:  `benchmark_sim.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/scenario/benchmark_sim.cpp>`_

Measures the computation time is requires to move 20 agents in a crossings for 1000 steps in simulation.


World
=====

:directory:  `world <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/world>`_


Rely on ``navground_core`` and ``navground_sim``. 

Component
---------

:file:  `world.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/world/world.cpp>`_

How to instantiate a world with walls, obstacles and agents, and perform some simulation steps.

Register
--------

:file:   `world_from_yaml.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/world/world_from_yaml.cpp>`_

How to load a world from YAML and perform some simulation steps.

YAML 
----

:file:  `world_with_groups.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/world/world_with_groups.cpp>`_

How to instantiate a world using groups.

.. _custom_recordings:

Custom recordings 
=================

:file:  `experiment_with_custom_probes.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/experiment/experiment_with_custom_probes.cpp>`_ 
 
:file:  `run_with_custom_probes.cpp <https://github.com/idsia-robotics/navground/tree/main/navground_examples/src/experiment/run_with_custom_probes.cpp>`_

How to use the experiment API to record custom data.
