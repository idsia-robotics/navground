# Changelog

## [Unreleased]

### Added

### Fixed

### Changed

### Removed

## [0.0.4] - 2024-1-4

### Added

- All:
  - support for Windows and MSVC
  - simplified installation using VCS
  - colcon deps and config files
  - pickle serialization via YAML

- Docs
  - how to use probes

- Sim:
  - waypoints task now supports random order and dynamically changing waypoints
  - discs state estimation now supports more parameters
  - added an option to retrieve the runs when running a multi-process experiment and saving them to a single HFD5 file
  - datasets
  - methods to remove agents from a world
  - function to open an HTML view

- CoppeliaSim:
  - properties accessors
  - adding/removing agents


### Fixed

- YAML-CPP target
- CSS style so that real time HTML visualization works in all major browsers
- Snap twists to zero when dumping World to YAML, else the YAML may contain invalid numbers.
- Silenced moviepy
- Moved registration to compiled files to support MSVC which does not like complex inline initializations 

### Changed

- All:
  - `@registered_property` to  `@property + @register`
  - C++ plugins uses now the ament index and do not require setting env variables anymore.

- Core:
  - removed optionals from social margin API.

- Sim: 
  - moved random generator to `World`. 
  - separated data ("records") from data recorders ("probes")
  - we now have three classes of probes: `Probe`, `RecordProve` (1 dataset), `GroupRecordProbe` (a map of datasets). They are defined by three callbacks to prepare, update, finalize an experimental run.
  - moved notebook view functions to separate module.
 
- CoppeliaSim:
  - now uses CoppeliaSim handles instead of navground UID as keys

### Removed

- Sim: 
  - do not store anymore number and indices of agents in `ExperimentalRun` but compute them on the fly.


## [0.0.3] - 2024-12-1

### Added

- All:
  - added cmake option and C macro `NAVGROUND_USES_DOUBLE` to use double instead of floats. It is disabled by default.

- Core:
  - added `Buffer` and `SensingState` to use homogeneous multi-dimensional array as environmental states.
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

