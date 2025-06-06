# navground

navground, short for **Nav**igation Play**ground**, is a playground to experiment with navigation algorithms.

## Core C++ library

At its core there is a library of two-dimensional navigation algorithms implemented in C++, which
provides a common API to interact with dynamic obstacle avoidance behaviors for multi-agent systems. Some behaviors are already implemented, more will be. Behaviors take the current state of environment and agent together with a target, and output a control command. 

### Features

- navigation behaviors and modulations
- environment states
- targets
- kinematics
- 2D and 2.5D controllers with an event-based interface
- collisions computation
- YAML serialization

Users can add their components (behaviors and kinematics) which are then auto-discovered by the rest of the system.

The core library is designed to be integrated in real-time run-times of real or simulated robots or of other kind of agents. We already integrated it into the following run-times.

## Core Python library

All core elements are also exposed as a Python 3 library. Components can be extended from Python;
that is, users can implement a behavior or a kinematics which are then fully integrated with the rest of the system.

## Simulation

The simulation complements the navigation behaviors of the core library with:
- tasks that generate targets for behaviors, and
- state estimation components that feed a potentially noisy and partial representation of the environment state to the behaviors.

It also provides all the infrastructure to run offline experiments.

### Features

- very fast kinematic simulation
- extensible tasks and state estimations
- generating world with agents and static obstacles from scenarios
- running experiments/benchmarks and recording data in HFD5 files
- using YAML to specify experiments

## Installation

[Build from source](https://idsia-robotics.github.io/navground/installation/from_source.html) or install pre-build wheels from pip

```
pip install navground[all]
```

## Documentation

For more information, we refer to the [project documentation](https://idsia-robotics.github.io/navground) that contains also detailed installation instructions.

## Try the tutorials on binder

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/idsia-robotics/navground/HEAD?labpath=docs%2Ftutorials)

## License and copyright

This software is free for reuse according to the attached MIT license.


## Extensions

### ROS 

[**navground_ros**](https://github.com/idsia-robotics/navground_ros): A ROS 2 node to perform 2.5D obstacle avoidance, wrapping the core C++ library with a ROS 2 compliant interface.

### Simulators 

[**navground_coppeliasim**](https://github.com/idsia-robotics/navground_coppeliasim): A CoppeliaSim plugin that exposes a lua interface to the C++ library.

### Machine-Learning

[**navground_learning**](https://github.com/idsia-robotics/navground_learning): A Python package that interfaces navground with Gymnasium and PetttingZoo to apply IL and RL to navigation.

[**navground_onnx**](https://github.com/idsia-robotics/navground_onnx): A C++ package to speed up inference of ML policies in navground using ONNX. 

## Acknowledgement and disclaimer

The work was supported in part by [REXASI-PRO](https://rexasi-pro.spindoxlabs.com) H-EU project, call HORIZON-CL4-2021-HUMAN-01-01, Grant agreement no. 101070028.

<img src="https://rexasi-pro.spindoxlabs.com/wp-content/uploads/2023/01/Bianco-Viola-Moderno-Minimalista-Logo-e1675187551324.png"  width="300">

The work has been partially funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Commission. Neither the European Union nor the European Commission can be held responsible for them.



