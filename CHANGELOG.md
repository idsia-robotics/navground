# Changelog

## [0.7.0] unreleased

### Fixed

- `Twist3` constructor.
- `FourWheelsOmniDriveKinematics` mapping between wheel speeds and twist. 
- `core.Neighbor.load, `sim.Wall.load`, and `sim.Obstacle.load` now work as expected.
- Fixed calling `class_type` on a Python register. This make `help(...)` works again when called on classes that have a register. 
- Fixed type hints errors.

### Added
- World and scenario property `ignore_collisions` to ignore collisions between any entity.
- Entity (agents, obstacles, walls) property `ignore_collisions` to ignore collisions for a specific entity.
- Additional type of object accepted by the `echo` command.
- Exposed `Entity.reset_uid` in Python.
- Branch name to build infos (concatenated to the output of `git describe`).
- Added several type aliases.
- Added added `mypy` tests for `navground_core_py`, `navground_sim_py`, `navground_examples_py`, and `navground_minimal_plugin_py`.
- Argument `strict` to `get_entity` to raise an exception if the entity uid is not found.

### Changed
- Upgraded to Pybind11 v3. Among others changes, this brings more accurate type hints. For example, methods that accept a C++ float, are now annotated to accept `typing.SupportsFloat`. This requires users to change the type annotation in their code if they are overriding navground methods (only relevant for static type checking, has no impact when executing the code).
	- Replaced `py::enum_` with `py::native_enum` (a subclass of Python native enums `enum.Enum`).
	- Updated stubs.
- Added methods `BoundingBox.bounds` and `World.bounds` that replace functions in `navground.sim.bounds`.
- `core.FloaType` is now configured at build time and can be used as a type alias.
- Replaced `super().__init__(...)` with `Base.__init__(self, ...)` for classes that inherit from a Python trampoline, as specified by Pybind11 (https://pybind11.readthedocs.io/en/stable/advanced/classes.html)
- Added Python 3.14 and updated dependencies:
	- argparse v3.2
  - eigen v5.0.0
  - geos 3.14.1
  - hdf5 1.14.6
  - highfive 3.2.0

### Removed

- `navground_utils`
- `navground.sim.bounds`
- `SensorCombination` (deprecated in 0.5)

## [0.6.0] 2026-06-07

In this release we add two components (`Marker` sensor, `Bicycle` kinematics), a new base class for behaviors that are evaluated jointly (e.g., centralized controllers), dynamic attributes for agents and worlds (which are similar to properties but are not defined in advance), and several methods related to bounding boxes.

We partially expose samplers to Python: it is now possible to load them from YAML and query them to generate values. We also implement a few new samplers (`uniform_size` and `permutation` for lists, `binary` for numbers, `normal_2d` for vectors). It is now possible to manipulate groups of scenarios in Python.

The treatment of targets' spatial and angular components is now symmetric.
We can now set specific and default configurations for rendering worlds 
We also fix few errors and implemented a more solid registration of Python properties. 

### Added

- Added `Marker` sensor.
- Added `include_{x|y}` options to `Discs` sensor.
- Added  `load_sensor` to load a `sim.Sensor` (instead of a `sim.StateEstimation`).
- Added `background_extras` to render functions to render svg elements behind the world.
- Now rendering decorations support an additional `world` argument.
- Added conversion between `BoundingBox` and tuple.
- Added `BoundingBox` modifiers.
- Added picking support to `BoundingBox`.
- Added dynamic attributes to `Agent` and `World`.
- Added `record_scenario_properties` to `Experiment` to record (sampled) scenario properties as world attributes.
- Added keyword argument `include_properties_of` for registered classes (`__init_subclass__`) to include properties defined by a superclass.
- Added Python methods `Sensor.get_field_name` and `Sensor.get_or_init_buffer`.
- Added `register_abstract_type` to register abstract subtypes name and properties without registering a factory method.
- Added `BehaviorGroupMember`, an abstract `Behavior` subclass that delegates the computation of commands to their group (`BehaviorGroup`).
- Added `Bicycle` kinematics.
- Added `Target` angular direction.
- Added additional `Behavior` target helpers and extended `ignore_tolerance` argument to existing helpers: the treatment of the angular and planar components is now symmetric.
- Added (limited) Scenario accessors to property samplers.
- Enabled explicitly setting properties type name in Python.
- Added world-specific and default rendering configuration, which provides defaults for all rendering functions. This way, e.g., a scenario can specify world-specific extra for the worlds it creates.
- Added `World._repr_svg_` to implicitly display worlds as SVG in jupyter.
- Optional wait time[s] at waypoints for task `Waypoints`. 
- Added `probabilites` parameter to `choice` sampler
- Added `binary`, `normal_2d`, permutation`, and `uniform_size` samplers.
- Exposed (property) samplers in Python and in the CLI (`echo`, `sample`, and `validate`). 
- Exposed agent samplers in Python.
- Added a tutorial on attributes and another on samplers.
- Added argument `keep` to `Sampler::reset` to separate resetting samplers applied to groups of agents (`keep=false`) from samplers applied to the whole scenario (`keep=true`). 

### Fixed

- Fixed an error that wrongly marked `Direction` task as done.
- Fixed typo in wall-disc collision computation.
- Fixed `dtype` comparison to use `equal` instead of `is`.
- Fixed documentation errors.
- Fixed YAML serialization of `std::vector<bool>` which caused empty list to be serialized as a null node instead of an empty list.
- Fixed pickle protocol that linked `__dict__` to the original value.
- Fixed bug that caused Python (registered) properties of type `Vector2` to be
  configured as type `list[float]` instead, and similar mismatches between list of numbers.
- Fixed a bug that caused deterministic samplers of scenario properties to reset when initializing new worlds.

### Changed

- Now `Behavior::check_if_target_satisfied` returns `false` is the target direction is defined and target speed is not zero.
- Split the initialization of a world by a scenario in two steps: `Scenario::init_world()` and `Scenario::apply_inits()`. This way, the specialized `init_world` can create entities that are accessible by the initializers. Users should normally call `make_world`, which performs these two steps automatically.
- Initializers and groups are now private members of a `Scenario` that can be manipulated using accessors.
- PySensor is now registered as "Sensor".
- `Behavior::get_target_distance` returns now a float (vs `std::optional`), returning when previously it would return `std::nullopt`.
- Targets are now recorded as a 16 dimensional vector; reading older recording is still supported.
- Modified Scenario decoder so that it first tries to decode a properties and only after it fails, it decodes a sampler. This way, scenarios with properties that have no or const samplers are encoded using the property value, which can be changed with the accessors. For properties with non-const samplers, the encoded values still reflects the sampler value, not the property value.
- In Python, the argument of registered properties setters are now coerced by default. You can disable it by exporting `NAVGROUND_DISABLE_PY_PROPERTY_COERCION`. The return type of the generic `get` method is also coerced: when a conversion is not possible, it returns the default value. This ensure that properties in YAML representation always have the correct types. Supported conversions are:
	- between scalar numerical types (float, int, bool) and between the relative vector types,
	- between a list of two numbers and a vector.

## [0.5.2] 2025-04-30

Fixes few more bugs

### Fixed

- Fixes repetition in `plot_world`
- Fixes stopping criteria in `DirectionTask` 
- Fixes computing collisions between walls and agents 

### Changed

- `Behavior.check_if_target_satisfied` now returns false when target direction is defined and target speed is not zero.

## [0.5.1] 2025-02-28

Fixes few bugs went unnoticed when releasing 0.5.0. 

### Fixed

- Fixed typo in `plot_world`
- Fixed notebook `running_an_experiment_no_hdf5.ipynb`
- Fixed `LidarScan` import

## [0.5.0] 2025-02-28

In this release we add support for multiple state estimations, something that previously had limited support through `SensorCombination` (limited to sensors and not exposed to samplers). We keep backward-compatibility in the API and YAML: we can still configure agents to use a single state estimation using the old API and keywords. Agents with multiple state estimations evaluate them in constant order: subsequent state estimations evaluations have access to the state written by the previous state estimations. For example, we can use this to add noise to an otherwise perfect sensor or to combine the perception of two sensors into an higher level state estimation, like `LocalGridMap` does with odometry and lidar scans.

The second small but important addition are `prepare/close` virtual methods (called when a simulation starts and finishes) to `Behavior`, `StateEstimation`, and `Task`. The later two classes already had `prepare` but missed `close`. Users can specialize these methods to setup/teardown their classes after being initialized, for example to load a resource whose path is exposed as YAML property (i.e., only filled after initialization), or to setup coordination between group of objects, like when registering behaviors to a centralized group behavior to be evaluated at once for all agents.

The last set of important changes are structural: we moved packages to separate repositories to keep the focus on the navground libraries and CLI, which we are also the being tested and documented; we switched from push to pull-request & merge to update the main branch, running smoke tests before merging; and we now support building separated installers for `navground_core` and `navground_sim` and wheels for `navground_core_py` and `navground_sim_py`.


### Added

- Added `bidirectional` property to `Corridor` scenario to select unidirectional or bidirectional flows.
- Added virtual methods `prepare` and `close` to `Behavior`
- Added virtual method `close` to `StateEstimation`, `Task` and `World`
- Added tests and cpack and/or wheels to `navground_{core,sim,core_py,sim_py,examples,examples_py,minimal_plugin_cpp,minimal_plugin_py`.
- Added options to `python_install_namespace_package` to overwrite `setup.cfg`.
- Added wheel build/install to `python_install_namespace_package`.
- Added source dependencies to `generate_docstrings` to avoid rebuilding docstrings.h.
- Added "--version" to Python CLI.
- Added several methods/properties to Python `BoundingBox` wrapping `geos::geom::Envelope` methods: `__eq__`, `__hash__`, `contains`, `covers`, `distance`, `expand_by`, `expand_to_include`, `intersection`, `intersects`, `translate`, `height` , `width`.
- Added methods `get_buffer` and `get_number_of_items` to `Dataset`.
- Added options to `WaypointTask`: goal orientations and specific tolerances for different waypoints.
- Added task `GoToPose` as simplified interface to a `Waypoint` task with a single waypoint.
- Added argument `plot_last_pose` to `plot_trajectory` to force plotting the last pose.
- Added `zorder` argument to `plot_agent`, `plot_world`, `plot_trajectory` and `plot_run`.
- Added `velocity_arrow_alpha` in `plot_agent`.
- Added `angular_speed_tolerance` to `Controller`.

### Fixed

- Fixed version returned by C++ CLI.
- Fixed `Behavior::feasible_speed` and `Behavior::feasible_angular_speed`to clamp between `[-max, max]` (vs previous `[0, max]`).

### Changed

- Moved calling `world->prepare` from experiment to experimental run.
- Switched from a single to a sequence of state estimations. Single state estimation are still supported (YAML and API).
- Deprecated `SensorCombination`
- Renamed `velocity_arrow_color` to `velocity_arrow_edge_color` in `plot_agent`.
- `Controller::is_still` now uses `Behavior::is_stopped`.
- `Behavior::is_stopped` is now public and exposed to Python.
- In `Agent::prepare`, we now set the behavior pose and twist too.


### Removed

- Moved `navground_ros` and `navground_msgs` to their [own repository](https://github.com/idsia-robotics/navground_ros). 
- Moved `navground_coppeliasim` to its [own repository](https://github.com/idsia-robotics/navground_coppeliasim). 

## [0.4.0] 2025-02-11

This is the first release that includes binary installation packages for the navground C++ library. To support them, we slightly modified how we build the project. In particular, we added directories `distribution/{core|sim}` to build installers using `CPack` (C++) and `setuptools` (Python). 

Using the updated workflow, we still release self-contained wheels on PyPi, to which we add 

- DEB packages in Linux, an installer package for macOS, and a NSIS installer for Windows to install the C++ library (both core and sim);
- Python wheels to install a Python packages that loads shared libraries from the installed distribution, instead of containing all shared libraries like the versions available on PyPi. Users should install one of these wheels if they want to build C++ plugins against the released C++ library and use them from Python.

### Fixed

- Fixed several warnings raised by all tested C++ compilers (clang, gcc, msvc).
- `navground_sim_py`: moved importing dependency inside functions to allow executing `navground_py` without installing extra dependencies.

### Added

- Added packages `navground_minimal_{cpp|py}_plugin` with a minimal skeleton to write, build, install, and distribute a plugin (in this case a `Behavior`).
- Added docker files that install navground using the latest release.
- Added support to uniform samplers of type `Vector2` in `navground_sim`.
- Added option to `navground_{core|sim}` `CMakeLists.txt` to build executables.
- Added options to `navground_{core|sim}_py` `CMakeLists.txt` to install Python files and build python extension in custom directories.

### Changed

- Adapted `navground_{core|sim}{_py}` `CMakeLists.txt` to support inclusion in other projects. Added projects' version which are used, when building the projects, as a fallback when git is not available. Increased the required cmake version to 3.17.
- Restructured release workflow, splitting it in several workflow to be called from main workflow.
- Moved plugins to separate sub-directory in `navground_examples_cpp`.
- Added support in `navground_{core|sim}_py` to include DLL directories in Windows by exporting `NAVGROUND_DLL_PATH`: we don't need to copy shared libraries inside the Python package anymore.
- Moved `plugins.dsv` from `navground_core` to packages installing plugins. `navground_core` still exports a default `NAVGROUND_PLUGINS_PREFIX` set to the location where it is installed but now plugins packages can add the path where they are installed too if `ament_cmake` is available.
- Added `from __future__ import annotations`

## [0.3.5] 2025-02-03

This patch release corrects the documentation on custom YAML conversions.

### Fixed

- `Scenario::add_init` does not anymore overwrite initializers added with `Scenario::set_init`.
- `Agent::idle` now check that the behavior has no valid target.
- `Controller` invalidate the behavior target at the end of a move action.
- Corrected guide about custom YAML conversion in Python.
- Exposed all virtual methods in the trampoline `PySensor`.

### Added

- Added an example of custom YAML conversion in Python.
- Added `Behavior::has_target`

## [0.3.4] 2025-01-09

This patch release adds a few arguments to Navground CLI and fixes some issues when running an experiment over multiple processes.

### Added

- In `run`, added optional argument `--save_directory` that overrides the experiment config.
- In `run`, `sample`, `echo`, and `record_video`, added optional argument `--chdir` that changes the working directory to the parent of the file that is being loaded. This enable to specified file paths in the file fields that are relative to the path. 
- lua function `simNavground.dump`
- Added run callbacks to `Experiment` pickling.
- Added `Buffer::get_typed_ptr`

### Fixed

- `Experiment.run_mp` now load the plugins in every process, not just the main one.
- `Experiment.run_mp` now works with `multiprocess` when we need a queue.
- Kinematics default values in `navground_coppeliasim`
- Corrected order of `collisions` and `safety_violation` in Python `RecordConfig` constructor (and pickle).

### Changed

- `World::update` now first updates the state of all agents and then the control of all the agents. This simplifies using coupled/centralized behaviors because, at the time of the first agent behavior update, the state of all the other behaviors is already up-to-date.

## [0.3.3] 2024-16-12

This patch release adds more support for Python `Scenario` initializers and fixes few bugs.

### Added

- Experiments now pickle Python probes too.
- Scenarios now pickle Python initializers too. For this, `Scenario` gets a `__dict__` too.
- Added several accessors to `Scenario` groups and inits.
- Exposed `Experiment::RunConfig` to Python

### Fixed

- Corrected single run case in `plot_runs`
- `Experiment.add_group_record_probe` and `Experiment.add_record_probe` have now types that are coherent with accepting factories and do not requires `factory.dtype` anymore, instead reading it from instances once created.
- Fixed `Behavior::get_target_direction` in absolute frame.

### Changed

- `Behavior::get_efficacy` now returns 0 when the target velocity is null.
- `Scenario` inits are now stored as a string-keyed map. `Scenario::add_init` accepts the same argument, creating and returning an incremental (string) key.


## [0.3.2] 2024-29-11

Patch release to fix a bug in `PathTask`.

### Added

- `DummyBehavior` can now be configured to use any `EnvironmentState` to favor its usage to test arbitrary state estimations.

### Fixed

- Fixed an error in `PathTask` and added a warning when `shapely` is not installed.

## [0.3.1] 2024-29-11

Patch release that remove deprecated type hints and add support for the recently released version 2 of moviepy.

### Added

- Added support for latest numpy and moviepy.

### Fixed

- Updated type hints to removed deprecated `typing` import and resolved all errors reported by `mypy --strict` and `ruff` with `flake8-pep585` and `pyupgrade`.
- Fixed that `navground_py schema sim` outputted a schema without registered core classes by moving static definitions of registers to separate cpp files.
- Fixed `--register` argument of `navground_py schema`.
- Fixed `use_python` which relied on specific Python base classes (removed in v0.3); we now look for the presence of Python methods (i.e., variables with a `__code__` attribute).

### Changed

- `navground.sim.pyplot_helpers.plot_runs` now hides the axis by default.
- Modified `navground.core.property to use `typing.get_type` instead of eval.

## [0.3.0] 2024-26-11

This release improves usability when extending navground with new components and when validating code:
- we documented all protected virtual methods that are meant to be overriden by sub-classes,
- we simplified and documented how to register a new component,
- we added (JSON-) schemas for all YAML representations and CLI commands to validate YAML string/files.

### Added

- Exposed and documented all virtual methods meant to be specialized in `Python` , in particular all the different sub-methods of `Behavior::compute_cmd`
- Added `core::PI`, `core::HALF_PI` and `core::TWO_PI`
- Added `EnvironmentState` constructor to Python
- Added optional argument to force `Behavior::compute_cmd` to output feasible commands.
- Added `Task::log_event` to simplify sub-classing tasks.
- Added optional arguments to `HasRegister<T>::register_type` to register properties and YAML schema.
- Added build-time information to sim/core libraries and plugins. Libraries now expose when they where built, with which configuration (for now just which floating-point type), and from which git commit. Libraries and plugins now expose also the list of dependencies (limited to navground libraries), with build-time information and load-time information, to check if library/plugins are using newer versions of the navground libraries than the version they where compiled against.
- Added `Scenario::make_world` and `Scenario::bounding_box`
- Added custom svg renderers and support to render quadrotors and robomasters
- Added `start_angle` and `fov` to lidar sensing state
- Added a background doc on the difference between getting a reference or a copy from Pybind11.
- Added doc guides on how to extend navground
- Documented all currently implemented components with videos and examples

### Fixed

- Resolved missing references in the docs, adding also `intersphinx_mapping`.
- Fixed `Behavior::compute_cmd` in case of no target: now calls `cmd_twist_towards_stopping` instead of returning zero.
- `sim.Dataset.push` now accepts different types of scalars without losing precision
- `MotorPIDModulation` now requires `DynamicTwoWheelsDifferentialDriveKinematics`.
- Python navground properties now correctly infer and pass down the correct type and on the C++ side, setters also respect the type.
- Fixed missing arguments and references in the docs.

### Changed

- Restructured the docs (all components have their own file now)
- Renamed `Sensor::prepare` to `Sensor::prepare_state`
- Renamed `Experimental::get_duration_ns` to `Experimental::get_duration`
- Renamed `ExperimentalRun::get_duration_ns` to `ExperimentalRun::get_duration`
- Refactored `pyplot_helper` to increase code reuse.
- The (agent) controller now evaluates the behavior even when no action is active. This enable an (agent) task to operate on behavior targets without passing through the controller.
- Simplified Python interfaces of `core.Buffer`, `core.SensingState`, `sim.Dataset`, and `sim.ExperimentalRun.add_record`: they now group together constructors/methods by accepting optional arguments.  
- `Pose` integration is now exact (i.e., along arc of circles instead of line segments)
- `WheeledKinematics` is now a pure abstract class that adds few interfaces converting twists to wheel speeds but no data. `TwoWheelsDifferentialDriveKinematics` and `FourWheelsOmniDriveKinematics` get their own `wheel_axis` member (renamed from `axis`)
- `Kinematics::get_max_speed` is now virtual while `Kinematics::is_wheeled` is still virtual but no more pure.
- In C++, registered classes get automatically overridden `get_type` and `get_properties`. Replaced static members `type` and `properties` with methods `HasRegister<T>::get_type<C>` and `HasRegister<T>::get_properties<C>`.
- Replaced ``make_property` with several overloads of `Property::make` to simplify defining properties from class methods.
- Refactored the implementation of Python components (behaviors, kinematics, ...) between C++ (Pybind11) and Python
   - replaced Python-defined base-classes with richer Pybind11 base classes
      - added `__init_subclass__` to enable autoregistration
      - pickle protocol uses `__dict__` when available
   - added `<Component>.load` and `<Component>.dump` from Pybind11
   - moved `load_<component>` and `dump` to Python 
   - added `property.py` and `schema.py` with helper methods for registration
- Added Python YAML APIs for `Wall`, `Obstacle`, `Line`, `Disc`, and `Neighbor`.
- Added YAML schema generation and validation as API and CLI (`schema` and `validate`). To support operating with schemas on Python, we added `PyYAML` as required dependency.
- Simplified and uniformed the interfaces of `Scenario::Group` and `Scenario::Init` to use optional seeds as arguments.
- Refined the error models in `OdometryStateEstimation`
- `ExperimentalRun` now stores all fields of target except paths and record the agent own sensing state, not just external sensors. `ExperimentalRun` and `RecordedExperimentalRun`,  `go_to_step` can now set target and sensing state.
- `Agent::update` now uses `Kinematics::feasible_from_current` to compute feasible commands and  check if the agent is stuck only after evaluating the behavior.
- `Agent` exposes `Controller::speed_tolerance` (also as YAML field)
- In most examples in the docs, we now generate the output automatically (like schemas and CLI commands)
- New types of `EnvironmentState` can be now defined (also) in Python.
- Updated Python stubs

### Removed

- Removed `Frame` argument from `Behavior::compute_cmd` (virtual) sub-methods: they are now free to compute a command in any frame. `Behavior::compute_cmd` takes now care of returning a command in the requested frame. 


## [0.2.2] - 2024-07-11

### Added

- Added to `core::Pose2` methods related to rigid transform.
- Exported `NAVGROUND_USES_DOUBLE` cmake option.

### Fixed

- Python plugins are now correctly loaded in the CLI.
- Fixed errors in the documentation.

### Changed

- Separated `Vector2Like` and `Vector2` in the Python stubs

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


## [0.2.0] - 2024-29-10

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

