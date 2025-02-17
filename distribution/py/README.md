# navground

Navground, short for **Nav**igation Play**ground**, is a playground to experiment with navigation algorithms. It is a modular library, primary implemented in C++.

This distribution provides the ``navground.core`` and ``navground.simulation`` Python packages and a CLI.

## Core

A library of two-dimensional navigation algorithms which
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

The core library is designed to be integrated in real-time run-times of real or simulated robots or of other kind of agents.

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

## Documentation

For more information, we refer to the [project documentation](https://idsia-robotics.github.io/navground) that contains also detailed installation instructions.

## Try the tutorials on binder

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/idsia-robotics/navground.git/HEAD?labpath=tutorials)

## License and copyright

This software is free for reuse according to the attached MIT license.

## Acknowledgement and disclaimer

The work was supported in part by [REXASI-PRO](https://rexasi-pro.spindoxlabs.com) H-EU project, call HORIZON-CL4-2021-HUMAN-01-01, Grant agreement no. 101070028.

<img src="https://rexasi-pro.spindoxlabs.com/wp-content/uploads/2023/01/Bianco-Viola-Moderno-Minimalista-Logo-e1675187551324.png"  width="300">

The work has been partially funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Commission. Neither the European Union nor the European Commission can be held responsible for them.