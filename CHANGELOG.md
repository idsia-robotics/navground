# Changelog

## [Unreleased]

### Added

### Fixed

### Changed

### Removed


## [0.2.2]

### Added

### Fixed

- Python plugins are now correctly loaded in the CLI.

### Changed

### Removed


## [0.2.1] - 2024-05-11

### Added

- `get_step_to_collision` method to `ExperimentalRun` and `RecordedExperimentalRun`

### Fixed

- The scenario YAML encoding now uses the walls schema like the decoder, and not the line schema.

- Installing `navground_{core|sim}_py` without `ament` now include packages files. 

- `Dataset::as_tensor` now returns a tensor view and no more a copy.

### Changed

- It is now possible to set the stroke color of the Agents' SVG via a decorator.

### Removed


## [0.2.1] - 2024-29-10

### Added

- Targets now have an optional path to follow to reach a pose or position. Behaviors support this path be defining a new virtual method `cmd_twist_along_path` whose default implementation is a form of carrot planner that uses two new parameters `path_tau` (over which time length to converge to the path), and `path_look_ahead` (how far ahead to look when following the path). The equivalent controller actions now have an additional parameter `along_path`.

- Added `PathTask` Python class to follow a path composed by a list of points.

- Behaviors can now overwrite a new virtual method `cmd_twist_towards_pose` to implement more specific control strategies to reach a pose. The default implementation moves straight to the pose and then rotates in place.

- Added functions `to_{relative|absolute}_point` to `core/common.h` to convert the frame of reference of points (not just vectors).

- Exposed additional `core/common.h` functions to Python

- `Buffer` exposes the stride of their multi-dimensional arrays.

- Added `name` to prefix all fields of the data updated by a `Sensor`

- Added wheels for Python 3.13

- Added `no-plugins` option to the CLI commands to disable loading plugins.

- Added `echo` command to load from YAML and then print the YAML representation of an object.

- Added `plugins` command to list loaded plugins.

- Added `OdometryStateEstimation` sensor.

- Added simple `LimitTwistModulation` modulation to clip the twist command.

- Added manual control mode using `follow_manual_command`: the controller will ignore the actual behavior and return the manual command.
### Fixed

- Resetting a sampler with `once=true` now correctly sets its index to 0.

- Recording keys associated to agents (for tasks and sensing) do now correctly make use of the config `use_agent_uid_as_key`.

### Changed

- Renamed `normalize` to `normalize_angle`

- All kinematics now have default values in their constructors and uses infinite to represent unconstrained values, like, e.g., maximal angular speed. Negative values are also interpreted as unconstrained.

- `TwoWheelsDifferentialDriveKinematics` now supports additional (control) limitation in angular, forward and backward speed. It also supports moving backwards, although it is disabled by default for back compatibility.

- Removed const-ness from `Task::prepare` 

- Removed const-ness from `StateEstimation::prepare` and `StateEstimation::update`.

- `LidarStateEstimation` is now more realistic, supporting offset positions and (normal) rangings errors.

### Removed

## [0.1.0] - 2024-27-9

### Added

- This is the first version with wheels **released on PyPi** and github
- Core: add `__main__` to `navground.core` similar to `navground.sim`

### Fixed

### Changed

- Docs and docker files reflect now the different ways we have to install navground (pip and/or build from source) and the different ways the CLI commands are exposed.
- Binder now uses a `requirement.txt` file instead of an custom docker image.
- `navground_py` and `navground` CLI are now more similar. 

### Removed

## [0.0.6] - 2024-26-9

### Added

- The `navground` executable that groups together all `navground_sim` commands.
- The `navground_py` executable that groups together all `navground_sim_py` commands.
- Github actions to build the packages and distribute wheels.

### Fixed

- Switched from the deprecated `pkg_resources` to `importlib`.
- Compilation on Linux by linking to `Threads`
- Docstrings generation now works in Windows too and from virtual envs.
- Avoided that missing docstrings break the whole build.
- Core:
  - behaviors do now make use of agent id when computing the social margin
- Sim:
  - now handle empty datasets correctly.
  - videos frame are now constrained to have even dimensions as required to correctly render them.
  - recorded experiment now report collisions correctly.
### Changed

- Separated the python package `navground_sim_py` from the C++ package `navground_sim`.
- Changed package name from `navground_py` to `navground_core_py`.
- Ament is now optional.
- Update the installation instructions.
- All Python dependencies but `numpy` are now optional for `navground_sim_py`.
- Replaced of C math functions with their C++ stdlib equivalent.
- Core:
  - speeded up collision computation, avoiding unnecessary conversions.
- Sim:
  - recording the initial state of the world (as YAML) is now optional
  - using `multiprocessing` is now an opt-in.

### Removed

## [0.0.5] - 2024-20-8

### Added


- Core:

  - algebraic and geometric operators to Disc and Neighbor
  - added custom YAML serialization/deserialization to registered subclasses (C++ and Python)
  - behavior modulations
    - acceleration limiting modulation
    - motor PID modulation
  - behavior tag manipulation from Python
  - dynamic two-wheeled diff drive kinematics

- Sim:

  - Eigen tensor view of datasets
  - jumping to time steps in recorded runs
  - user customizable World termination criteria
  - `World` bounds property
  - grid, agent following, rotation and trimming to visualization/videos
  - support to make videos from simulated runs
  - python (sub) modules for notebook and pyplot
  - exposed random generator (seed) in Python
  - sensor combination and boundary sensor
  - recording sensor readings and neighbors
  - experiment init callback
  - method to extract collision "events"

- Python:
  - stubs for core and sim
  - increased pickle support

### Fixed

  - Corrected (registered) Python sub-classes pickling
  - Lines were sometimes ignored by ORCA
  - Replaced implicit zero eigen vectors with explicit `Vector2::Zero()`` calls

### Changed

- Core:

  - improved ORCA: switched to most recent library version, new options, exposed ORCA lines, neighbor computation aligned to RVO2 implementation
  - improved HRVO: new options and cleaned up
  - improved SocialForce: exposed options
  - it is now possible to instantiate "empty" behaviors from YAML
  - refactored `Behavior::compute_cmd` into public not virtual class and protected virtual internal implementation

- Sim:

  - group number is now a random variable
  - function to sample discs
  - lattice implementation does not use ghosts anymore
  - refactored World preparation and validity checks
  - requires now `multiprocess` instead of `multiprocessing`
  - added option to report nearest point instead of center  and id to `DiscsStateEstimation`
  - wall/obstacle YAML include Line/disc as an child now
  - corrected collision with walls
  - scenario properties are now random variables too.
  - experiments now stores Python probes
  - samplers reset seed is now optional

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

