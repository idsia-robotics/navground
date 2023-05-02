============
Introduction
============

Navground, short **Nav**\ igation Play\ **ground**\ , is a playground to experiment with navigation algorithms.

Core library
============

At its core, there is a library of two-dimensional navigation algorithms implemented in C++.
that provides a common API to interact with multi-agent dynamic obstacle avoidance behaviors. Some behaviors are already implemented, more will be. Behaviors take the current state of the environment and of the agent, a target and output a control command. 

Features
--------

- navigation behaviors and modulations
- environment states
- targets
- kinematics
- 2D and 2.5D event-based controllers
- collision computation
- YAML serialization

Users can add their components (behaviors and kinematics) which are auto-discovered by the rest of the system.

The core library is designed to be integrated in real-time run times to provide obstacle avoidance to real or simulated robots or other kind of agents. We already integrated it into the following run-times

ROS 
---

We provide a ROS 2 node to perform 2.5D obstacle avoidance. The node wraps the core C++ library with a ROS2 compliant interface.


CoppeliaSim 
-----------

We also provide a coppaliaSim plugin that exposes a lua interface to the core library.

Python bindings
---------------

All core elements are also exposed as a Python 3 package. Moreover, components can be extended from Python too.
That is, users can implement a behavior or a kinematics which are then fully integrated with the rest of the system.

Simulation
==========

The simulation complements the navigation behaviors of core library:
- tasks that generates targets to behaviors
- state estimations that feeds the environment state of the  behaviors

It also provides all the infrastructure to run offline experiments.

Features
--------

- very fast kinematic simulation
- extensible tasks and state estimations
- generating world with agents and static obstacles from scenarios
- running experiments/benchmarks and recording data in HFD5 files
- using YAML to specify experiments
- Python bindings
- Real-time Python simulation with UI.


Acknowledgement and disclaimer
==============================

The work was supported in part by `REXASI-PRO <https://rexasi-pro.spindoxlabs.com>`_ H-EU project, call HORIZON-CL4-2021-HUMAN-01-01, Grant agreement no. 101070028.

.. image:: https://rexasi-pro.spindoxlabs.com/wp-content/uploads/2023/01/Bianco-Viola-Moderno-Minimalista-Logo-e1675187551324.png
  :width: 300
  :alt: REXASI-PRO logo

The work has been partially funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Commission. Neither the European Union nor the European Commission can be held responsible for them.



