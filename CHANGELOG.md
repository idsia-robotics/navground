# Changelog

## [Unreleased]

### Added

### Fixed

### Changed

### Removed

## [0.0.3] - 2023-1-12

### Added

- All:
  - added cmake option and C macro `NAVGROUND_USES_DOUBLE` to use double instead of floats. It is disabled by default.

- Core:
  - added `Buffer` and `SensingState` to use homogeneous multi-dimensional array as enviromental states.
  - added deprecated names to properties
  - added deadlocked/stuck 
  - add efficacy

- Sim:
  - added `Lidar` and `Discs` State estimations
  - added agent `color`
  - added property `update_static_obstacles` to `BoundedStateEstimation`
  - added the possibility to record custom data during experiments
  - added support to run experiment in parallel.
  - added class to load recorded experiment.
  - added capability to render worlds and record videos
  - added `runs` and `run_index` params to the `run[_py]` executables
  - added optional `tqdm` progress bar to simulations.
  - added displaying deadlocks, collisions, and custom decoration in UI

- CoppeliaSim:
  - added support for CoppeliaSim 4.6

- Examples:
  - added examples for custom recordings
  - added example for custom decoration
  
### Fixed

- Sim:
  - competed `World::get_static_obstacles_in_region`

### Changed

- Core:
  - changed `Collisions` to use `std::valarray` and split angles/ranges

- Sim:
  - changed `range_of_view` to `range`
  - moved to optional seed for `Scenario::init_world`
  - refactored `Trace` into 2 separate classes, `ExperimentalRun` that holds the data, and `Probe` that records data.
  - `Experiment` can now store multiple runs, not just the last.
  - Each world has now its own random generator

### Removed

## [0.0.2] - 2023-1-12

### Added

- Sim:
  - `run_until` and callbacks to World
  - Task logging
  - replay
  - added `once` parameter to sample only once per run

- CoppeliaSim:
  - support for CoppeliaSim 4.5
  - recording experiments in CoppeliaSim
  - support for externally controlled agents in CoppeliaSim

- Docs:
  - experiment tutorial
  - scenarios' documentation

- Support for clang15

### Fixed

- HL relaxation.

### Changed

- HL behavior when overlapping safety margin
- Pass optional seed to `Scenario::init_world`
- An additional `state` argument to the protected virtual interface of `StateEstimation::update`.
- Various other changes to the internal interface.

### Removed

## [0.0.1] - 2023-05-05

### Added

- Initial release

