from __future__ import annotations
import datetime
from navground import core
from navground.core._navground import HasProperties
import numpy
import os
import tqdm
import typing
import pathlib

Vector2Like = numpy.ndarray | tuple[float, float] | list[float]
ScenarioInitCallback = typing.Callable[['Scenario', int], None]

def uses_doubles() -> bool:
    ...

def use_compact_samplers(value: bool) -> None:
    ...

__all__ = ['Agent', 'AntipodalScenario', 'BoundedStateEstimation', 'BoundingBox', 'CorridorScenario', 'CrossScenario', 'CrossTorusScenario', 'Dataset', 'DiscsStateEstimation', 'Entity', 'Experiment', 'ExperimentalRun', 'GroupRecordProbe', 'LidarStateEstimation', 'Obstacle', 'Probe', 'RecordConfig', 'RecordProbe', 'Scenario', 'ScenarioRegister', 'Sensor', 'SimpleScenario', 'StateEstimation', 'StateEstimationRegister', 'Task', 'TaskRegister', 'Wall', 'DirectionTask', 'WaypointsTask', 'World', 'dump', 'load_agent', 'load_experiment', 'load_scenario', 'load_state_estimation', 'load_task', 'load_world']
class Agent(NativeAgent, Entity):
    """
    This class describes an agent.
    
    The agent navigates in the environment using a task, a state
    estimation, a kinematic and a behavior, and a controller.
    
    Agents have a circular shape which should match the shape of their
    navigation :py:class:`Behavior`.
    
    The role of task and state estimation is to provide goals and
    environment state (perception) to the behavior.
    
    Agents have a public identifies :py:attr:`id` that is accessible by the
    other agents' state estimation and may be passed to their behavior as
    :py:attr:`Neighbor.id`. This identifier may not be unique
    (e.g., may be used to identifies *groups* of agents).
    
    Agents runs their update at the rate set by :py:attr:`control_period` even
    if the world is updated at a faster rate.
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, radius: float = 0, behavior: core.Behavior | None = None, kinematics: core.Kinematics | None = None, task: Task | None = None, state_estimation: StateEstimation | None = None, control_period: float = 0, id: int = 0) -> None:
        """
        Constructs a new instance.
        
        :param radius:
            The radius of the agent
        
        :param behavior:
            The behavior
        
        :param kinematics:
            The kinematics
        
        :param task:
            The task
        
        :param estimation:
            The estimation
        
        :param control_period:
            The control period
        
        :param id:
            The public identifier
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def behavior(self) -> core.Behavior:
        ...
    @behavior.setter
    def behavior(self, arg1: core.Behavior) -> None:
        ...
    @property
    def controller(self) -> core.Controller:
        ...
    @property
    def kinematics(self) -> core.Kinematics:
        ...
    @kinematics.setter
    def kinematics(self, arg1: core.Kinematics) -> None:
        ...
    @property
    def state_estimation(self) -> StateEstimation:
        ...
    @state_estimation.setter
    def state_estimation(self, arg1: StateEstimation) -> None:
        ...
    @property
    def task(self) -> Task:
        ...
    @task.setter
    def task(self, arg1: Task) -> None:
        ...
class AntipodalScenario(Scenario):
    """
    A scenario that place the agents around a circle at regular intervals
    and task them to reach the opposite ("antipode") side.
    
    *Registered properties*:
    
    - `radius` (float, :py:attr:`radius`)
    
    - `tolerance` (float, :py:attr:`tolerance`)
    
    - `position_noise` (float, :py:attr:`position_noise`)
    
    - `orientation_noise` (float, :py:attr:`orientation_noise`)
    
    - `shuffle` (bool, :py:attr:`shuffle`)
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, radius: float = 1.0, tolerance: float = 0.1, position_noise: float = 0.0, orientation_noise: float = 0.0, shuffle: bool = False) -> None:
        """
        Constructs a new instance.
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def orientation_noise(self) -> float:
        """
        The orientation.
        """
    @orientation_noise.setter
    def orientation_noise(self, arg1: float) -> None:
        ...
    @property
    def position_noise(self) -> float:
        """
        The position_noise.
        """
    @position_noise.setter
    def position_noise(self, arg1: float) -> None:
        ...
    @property
    def radius(self) -> float:
        """
        The circle radius.
        """
    @radius.setter
    def radius(self, arg1: float) -> None:
        ...
    @property
    def tolerance(self) -> float:
        """
        The goal tolerance.
        """
    @tolerance.setter
    def tolerance(self, arg1: float) -> None:
        ...
class BoundedStateEstimation(StateEstimation):
    """
    Perfect state estimation within a range from the agent.
    
    *Registered properties*:
    
    - `range` (float, :py:attr:`range`), deprecated synonym `range_of_view`
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, range: float = 0.0, update_static_obstacles: bool = False) -> None:
        """
        Constructs a new instance.
        
        :param range_:
            The range of view
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def _neighbors_of_agent(self, agent: Agent, world: World) -> list[core.Neighbor]:
        """
        Gets the neighbors that lie within :py:attr:`range` from the agent.
        
        :param agent:
            The agent
        
        :param world:
            The world the agent is part of.
        
        :return:
            A list of neighbors around the agent
        """
    @property
    def range(self) -> float:
        """
        The maximal range of view.
        """
    @range.setter
    def range(self, arg1: float) -> None:
        ...
    @property
    def update_static_obstacles(self) -> bool:
        """
        Whether to set the static obstacles in ``prepare`` (ignoring the
        range) or in ``update``.
        """
    @update_static_obstacles.setter
    def update_static_obstacles(self, arg1: bool) -> None:
        ...
class BoundingBox:
    """
    A rectangular region
    """
    @typing.overload
    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float) -> None:
        """
        Creates a rectangular region
        
        :param min_x:
            Minimal x coordinates
        :param max_x:
            Maximal x coordinate
        :param min_y:
            Minimal y coordinate
        :param max_y:
            Maximal y coordinate
        """
    @typing.overload
    def __init__(self, p1: Vector2Like, p2: Vector2Like) -> None:
        """
        Creates a rectangular region
        
        :param p1: Bottom-left corner
        :param max_x: Top-right corner
        """
    def __repr__(self) -> str:
        ...
    @property
    def max_x(self) -> float:
        ...
    @property
    def max_y(self) -> float:
        ...
    @property
    def min_x(self) -> float:
        ...
    @property
    def min_y(self) -> float:
        ...
    @property
    def p1(self) -> numpy.ndarray:
        ...
    @property
    def p2(self) -> numpy.ndarray:
        ...
class CorridorScenario(Scenario):
    """
    A scenario where agents travel along an infinite corridor in opposite
    directions. Agents are initialize at non-overlapping random poses.
    
    *Registered properties*:
    
    - `width` (float, :py:attr:`width`)
    
    - `length` (float, :py:attr:`length`)
    
    - `agent_margin` (float, :py:attr:`agent_margin`)
    
    - `add_safety_to_agent_margin` (bool, :py:attr:`add_safety_to_agent_margin`)
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, width: float = 1.0, length: float = 10.0, agent_margin: float = 0.1, add_safety_to_agent_margin: bool = True) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def add_safety_to_agent_margin(self) -> bool:
        """
        Whenever the agent's safety margin should be considered in addition to
        :py:attr:`agent_margin` when initializing the agents' poses.
        """
    @add_safety_to_agent_margin.setter
    def add_safety_to_agent_margin(self, arg1: bool) -> None:
        ...
    @property
    def agent_margin(self) -> float:
        """
        The initial minimal distance between agents.
        """
    @agent_margin.setter
    def agent_margin(self, arg1: float) -> None:
        ...
    @property
    def length(self) -> float:
        """
        The length of the simulated portion of corridor. agents
        experience an infinite corridor, as it wraps around.
        """
    @length.setter
    def length(self, arg1: float) -> None:
        ...
    @property
    def width(self) -> float:
        """
        The width of the corridor.
        """
    @width.setter
    def width(self, arg1: float) -> None:
        ...
class CrossScenario(Scenario):
    """
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, side: float = 2.0, tolerance: float = 0.25, agent_margin: float = 0.1, add_safety_to_agent_margin: bool = True, target_margin: float = 0.5) -> None:
        """
        A scenario where agents move between two waypoints, one half of the
        agents vertically and the other horizontally. Agents are initialize at
        non-overlapping random poses in a box.
        
        *Registered properties*:
        
        - `side` (float, :py:attr:`side`)
        
        - `tolerance` (float, :py:attr:`tolerance`)
        
        - `agent_margin` (float, :py:attr:`agent_margin`)
        
        - `add_safety_to_agent_margin` (bool, :py:attr:`add_safety_to_agent_margin`)
        
        - `target_margin` (float, :py:attr:`target_margin`)
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def add_safety_to_agent_margin(self) -> bool:
        """
        Whenever the agent's safety margin should be considered in addition to
        :py:attr:`agent_margin` when initializing the agents' poses.
        """
    @add_safety_to_agent_margin.setter
    def add_safety_to_agent_margin(self, arg1: bool) -> None:
        ...
    @property
    def agent_margin(self) -> float:
        """
        The initial minimal distance between agents.
        """
    @agent_margin.setter
    def agent_margin(self, arg1: float) -> None:
        ...
    @property
    def side(self) -> float:
        """
        The half-length of the squared arena. waypoints are placed at
        (+/-side, 0) and (0, +/-side)
        """
    @side.setter
    def side(self, arg1: float) -> None:
        ...
    @property
    def target_margin(self) -> float:
        """
        The minimal distance between agents and targets at
        initialization.
        """
    @target_margin.setter
    def target_margin(self, arg1: float) -> None:
        ...
    @property
    def tolerance(self) -> float:
        """
        The task goal tolerance (i.e., agents will change target when
        they arrive closer than this to their current target).
        """
    @tolerance.setter
    def tolerance(self, arg1: float) -> None:
        ...
class CrossTorusScenario(Scenario):
    """
    A scenario where agents move crosses in a middle zone, one half of the
    agents moving vertically and the other half horizontally. This world
    is period in both directions.
    
    *Registered properties*:
    
    - `side` (float, :py:attr:`side`)
    
    - `agent_margin` (float, :py:attr:`agent_margin`)
    
    - `add_safety_to_agent_margin` (bool, :py:attr:`add_safety_to_agent_margin`)
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, side: float = 2.0, agent_margin: float = 0.1, add_safety_to_agent_margin: bool = True) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def add_safety_to_agent_margin(self) -> bool:
        """
        Whenever the agent's safety margin should be considered in addition to
        :py:attr:`agent_margin` when initializing the agents' poses.
        """
    @add_safety_to_agent_margin.setter
    def add_safety_to_agent_margin(self, arg1: bool) -> None:
        ...
    @property
    def agent_margin(self) -> float:
        """
        The initial minimal distance between agents.
        """
    @agent_margin.setter
    def agent_margin(self, arg1: float) -> None:
        ...
    @property
    def side(self) -> float:
        """
        The side of simulate cell of the infinite lattice.
        """
    @side.setter
    def side(self, arg1: float) -> None:
        ...
class Dataset:
    """
    Dynamic homogeneous multi-dimensional numerical data stored in
    ``std::vector``. Used to record data collected during an :py:class:`ExperimentalRun`.
    """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, data: numpy.ndarray) -> None:
        """
        Instantiate a dataset.
        
        :param data: Copies shape, data and dtype from this numpy array
        :type data: :py:class:`numpy.ndarray`
        """
    @typing.overload
    def __init__(self, dtype: typing.Any, item_shape: list[int] = []) -> None:
        """
        Instantiate a dataset.
        
        :param dtype: The type of data to store
        :type dtype: Any object that is convertible to a :py:class:`numpy.dtype`.
        
        :param item_shape: The shape of all axis except the first.
                           Leave empty to instantiate a flat dataset.
        :type item_shape:  List[int]
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def append(self, values: numpy.ndarray, reset: bool = False) -> None:
        """
        Append items from a numpy array.
        
        :param values: Append data and dtype from this numpy array
        :type values: :py:class:`numpy.ndarray`
        :param reset: Whether to replace the data instead of appending. 
        :type reset: bool
        """
    @typing.overload
    def append(self, values: list[float] | list[float] | list[int] | list[int] | list[int] | list[int] | list[int] | list[int] | list[int] | list[int]) -> None:
        """
        Add items.
        
        The items will be implicitly converted to the current data type, see
        :py:attr:`dtype`.
        
        :param values:
            The values to add
        
        Template parameter ``T``:
            The type of the items. Must be convertible to the current data
            type.
        """
    def push(self, value: float | float | int | int | int | int | int | int | int | int) -> None:
        """
        Add an item.
        
        The item will be implicitly converted to the current data type, see
        :py:attr:`dtype`.
        
        :param value:
            The value to add
        
        Template parameter ``T``:
            The type of the item. Must be convertible to the current data
            type.
        """
    def reset(self) -> None:
        """
        Clear all data.
        """
    @property
    def dtype(self) -> numpy.dtype[typing.Any]:
        """
        The type of the dataset.
        
        Can be set to any object that is convertible to a :py:class:`numpy.dtype`.
        """
    @dtype.setter
    def dtype(self, arg1: typing.Any) -> None:
        ...
    @property
    def is_valid(self) -> bool:
        """
        Determines if valid.
        """
    @property
    def item_shape(self) -> list[int]:
        """
        The shape of all axis except the first.
        """
    @item_shape.setter
    def item_shape(self, arg1: list[int]) -> None:
        ...
    @property
    def shape(self) -> list[int]:
        """
        Returns the shape of the multi-dimensional dataset
        """
    @property
    def size(self) -> int:
        ...
class DiscsStateEstimation(Sensor, StateEstimation):
    """
    Perceive a fixed number of nearest neighbors and obstacles
    
    Empty places are filled with zeros
    
    *Registered properties*:
    
    - `range` (float, :py:attr:`range`)
    
    - `number` (int, :py:attr:`number`)
    
    - `max_radius` (int, :py:attr:`number`)
    
    - `max_speed` (int, :py:attr:`number`)
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, range: float = 1.0, number: int = 1, max_radius: float = 0, max_speed: float = 0, include_valid: bool = True, use_nearest_point: bool = True, max_id: int = 0) -> None:
        """
        Constructs a new instance.
        
        :param range_:
            The range of view
        
        :param number_:
            Number of discs
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def max_radius(self) -> float:
        """
        The maximal neighbor radius.
        """
    @max_radius.setter
    def max_radius(self, arg1: float) -> None:
        ...
    @property
    def max_speed(self) -> float:
        """
        The maximal neighbor speed.
        """
    @max_speed.setter
    def max_speed(self, arg1: float) -> None:
        ...
    @property
    def number(self) -> int:
        """
        The number of discs.
        """
    @number.setter
    def number(self, arg1: int) -> None:
        ...
    @property
    def range(self) -> float:
        """
        The range of view.
        """
    @range.setter
    def range(self, arg1: float) -> None:
        ...
    @property
    def include_valid(self) -> bool:
        ...
    @include_valid.setter
    def include_valid(self, arg1: bool) -> None:
        ...
    @property
    def use_nearest_point(self) -> bool:
        ...
    @use_nearest_point.setter
    def use_nearest_point(self, arg1: bool) -> None:
        ...
    @property
    def max_id(self) -> int:
        ...
    @max_id.setter
    def max_id(self, arg1: int) -> None:
        ...
class Entity:
    """
    Super-class that adds a unique ID to world entities.
    
    This unique ID should not be fed to navigation behaviors, but only
    used internally by the simulation, for instance, to identify entities
    in a UI.
    """
    @property
    def _uid(self) -> int:
        """
        Unique identifier
        """
    @property
    def last_collision_time(self) -> float:
        ...
class Experiment:
    """
    An experiment supervises the execution of several runs of simulation.
    
    It initializes simulations sharing the same :py:class:`Scenario` and run them
    while collecting the desired data through :py:class:`ExperimentalRun`.
    
    - use :py:meth:`run_once` to perform a single run.
    
    - use :py:meth:`run` to perform all runs, optionally saving the data to a
      HDF5 dataset.
    
    - use :py:meth:`start`, :py:meth:`stop`, :py:meth:`start_run`, :py:meth:`stop_run`, :py:meth:`update_run` to record data without launching a simulation, for
      instance if you are using a different run-loop.
    """
    _group_record_probes: dict[str, typing.Callable[[], GroupRecordProbe]]
    _probes: list[typing.Callable[[], Probe]]
    _record_probes: dict[str, typing.Callable[[], RecordProbe]]
    @staticmethod
    def run_mp(experiment: Experiment, number_of_processes: int, keep: bool = False, number_of_runs: typing.Optional[int] = None, start_index: typing.Optional[int] = None, callback: typing.Optional[typing.Callable[[int], None]] = None, bar: typing.Optional[tqdm.tqdm] = None) -> None:
        """
        
        
            Run an experiment distributing its runs in parallel over multiple processes.
        
            Use this to parallelize experiments that contains Python classes, see :py:fun:``uses_python``,
            :py:attr:`sim.Experiment.run` parallelizes over multiple threads instead and cannot
            be used to run such experiments because of the GIL.
        
            If ``keep=True``,  the experiment will query the runs from the different processes and hold them in memory.
            If it is configured to save the data, it will save a single HDF5 file.
        
            If ``keep=False``, the experiment won't keep the runs in memory. If it is configured to save the data,
            it will save one HDF5 file per process (``"data_<i>.h5"``) and one "data.h5" linking all the runs together.
            To access the data, you will need to load the HFD5 file.
        
            :param      experiment:           The experiment
            :param      number_of_processes:  The number of processes
            :param      keep:                 Whether to keep runs in memory
            :param      number_of_runs:       The number of runs
            :param      start_index:          The index of the first run
            :param      callback:             An optional callback to run after each run is completed
            :param      bar:                  An optional tqdm bar to display the progresses.
        
            
        """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, time_step: float = 0.1, steps: int = 1000) -> None:
        """
        Constructs a new instance.
        
        :param time_step:
            The default simulation time step
        
        :param steps:
            The default number of simulation steps
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add_group_record_probe(self, key: str, probe_cls: typing.Type[GroupRecordProbe]) -> None:
        """
        Register a probe to record a group of data to during all runs.
        
        :param key: the name associated to the group
        :type key: str
        :param probe_cls: the class of the probe.
        :type probe_cls: Type[sim.GroupRecordProbe]
        """
    def add_probe(self, factory: typing.Callable[[], Probe]) -> None:
        """
        Register a probe to be added to all runs
        
        :param factory:
            A function that generate the probe.
        """
    def add_record_probe(self, key: str, probe_cls: typing.Callable[[], RecordProbe]) -> None:
        """
        Register a probe to record data to during all runs.
        
        :param key: the name associated to the record
        :type key: str
        :param probe_cls: the class of the probe.
        :type probe_cls: Type[sim.RecordProbe]
        """
    def add_run(self, seed: int, run: ExperimentalRun) -> None:
        ...
    def add_run_callback(self, callback: typing.Callable[[ExperimentalRun], None], at_init: bool = False) -> None:
        """
        Adds a callback to be executed after each run.
        
        :param value:
            The callback
        
        :param value:
            Whether the callback should be called when initializing the run.
            If not set, it will be called at the completion of a run.
        """
    def clear_run_callbacks(self) -> None:
        """
        Remove all run callbacks
        
        :param value:
            The callback
        """
    def get_run(self, arg0: int) -> ExperimentalRun:
        ...
    def init_run(self, seed: int, world: World | None = None) -> ExperimentalRun:
        """
        Initializes a run
        
        :param seed:
            The random seed
        
        :param world:
            The world to simulate. If null, it will initialize a new world.
        
        See :py:meth:`start`. This is only need when using an external run-loop to
        simulate or when manually calling :py:meth:`ExperimentalRun.run` later.
        Else use :py:meth:`run_once` (or :py:meth:`run` for all runs) to initialize and
        run at once.
        """
    def remove_all_runs(self) -> None:
        """
        Clear the recording
        """
    def remove_run(self, seed: int) -> None:
        """
        Removes a recorded run.
        
        :param seed:
            The seed/index of the run
        """
    def run(self, keep: bool = True, number_of_threads: int = 1, start_index: int | None = None, number_of_runs: int | None = None, data_path: os.PathLike | None = None) -> None:
        """
        Perform several runs and optionally record the data in a HFD5 file.
        
        The number of runs is specified by the default :py:attr:`number_of_runs` if
        not specified.
        
        Runs will be indexed sequentially and their index used as a random
        seed.
        
        If :py:attr:`save_directory` not empty but points to an existing directory,
        it creates a HDF5 file ``<name>_<hash>_<timestamp>/data.h5`` with
        attributes
        
        - ``begin_time`` [``string``], ISO 8601 formatted string of the time
          when the experiment is run, see :py:attr:`begin_time`;
        
        - ``duration_ns`` [``unsigned``], total duration in nanoseconds, see
          :py:attr:`duration_ns`.
        
        - ``experiment`` [``string``], YAML serialization of the experiment;
        
        Moreover, at the end of each run, it saves a group ``run_<index>``
        with attributes:
        
        - ``duration_ns`` [``unsigned``], total duration in nanoseconds, see
          :py:attr:`run_duration_ns`;
        
        - ``seed`` [``unsigned``];
        
        - ``steps`` [``unsigned``], actual number of steps performed;
        
        - ``maximal_steps`` [``unsigned``], maximal number of steps that could
          have been performed;
        
        - ``final_sim_time`` [``float``], the simulated time at the end of the
          run;
        
        - ``world`` [``string``], YAML serialization of the world at the begin
          of the experiment.
        
        datasets:
        
        - ``times`` [``float``] (if :py:attr:`RecordConfig.time` is set);
        
        - ``poses`` [``float``] (if :py:attr:`RecordConfig.pose` is set);
        
        - ``twists`` [``float``] (if :py:attr:`RecordConfig.twist` is set);
        
        - ``cmds`` [``float``] (if :py:attr:`RecordConfig.cmd` is set);
        
        - ``targets`` [``float``] (if :py:attr:`RecordConfig.target` is set);
        
        - ``collisions`` [``unsigned``] (if :py:attr:`RecordConfig.collisions` is
          set);
        
        - ``deadlocks`` [``float``] (if :py:attr:`RecordConfig.deadlocks` is set);
        
        - ``efficacy`` [``float``] (if :py:attr:`RecordConfig.efficacy` is set);
        
        and groups:
        
        - ``task_events`` (if :py:attr:`RecordConfig.task_events` is set), where
          each agents logs, in dataset ``<uid>`` [``float``], the events
          emitted by their task.
        
        Apart from saving data to the HDF5 file, each run is performed
        similarly to :py:meth:`run_once`.
        
        :param keep:
            Whether to keep runs in memory
        
        :param number_of_threads:
            How many threads to use. When more than one, runs will be
            distributed in parallel over the threads.
        
        :param start_index:
            The index/seed of the first run. If unspecified, it will use the
            experiment's :py:attr:`run_index`
        
        :param number_of_runs:
            The number of runs. If unspecified, it will use the experiment's
            :py:attr:`number_of_runs`
        
        :param data_path:
            A path to set optionally as :py:attr:`file_path`. If set, it will save
            an HDF5 file, but no YAML, to this path. If not set, :py:attr:`file_path` will be automatically set to
            ``<save_directory>/data.h5`` if :py:attr:`save_directory` is set.
        """
    def run_once(self, seed: int) -> ExperimentalRun:
        """
        Perform a single run
        
        1. it initializes a world from its scenario by calling :py:meth:`Scenario.init_world`
        
        2. it runs the simulation, step by step, collecting data in a :py:class:`ExperimentalRun` by calling :py:meth:`ExperimentalRun.run`
        
        :param seed:
            The index (and random seed) of the run
        
        :return:
            The recorded run.
        """
    def save(self, directory: os.PathLike | None = None, path: os.PathLike | None = None) -> None:
        """
        Save all recorded runs.
        
        :param directory:
            A path to set optionally as :py:attr:`save_directory` where to save YAML
            and HDF5 file.
        
        :param path:
            A path to set optionally as :py:attr:`file_path`. If set, it will save
            an HDF5 file, but no YAML, to this path. If not set, :py:attr:`file_path` will be automatically set to
            ``<save_directory>/data.h5`` if :py:attr:`save_directory` is set.
        """
    def setup_tqdm(self, bar: tqdm.tqdm, number_of_runs: typing.Optional[int] = None) -> None:
        """
        
                Configure a tqdm object that displays the progress of an experiment
        
                :param bar: a tqdm progress bar
        
                :param number_of_runs: the number of runs to track. If not provided,
                                       it will use :py:attr:`sim.Experiment.number_of_runs`.
        
            
        """
    def start(self, path: os.PathLike | None = None) -> None:
        """
        Signal to start an experiment
        
        Note that this won't neither execute any simulation nor record data.
        It will just record time stamp and change the state to running.
        
        This is only needed when using an external run-loop for the
        simulation. In this case:
        
        1. call :py:meth:`start`
        
        2. For each run
        
        i. Call :py:meth:`init_run` and :py:meth:`start_run` ii. from your run-loop, call
        :py:meth:`update_run` to record the current state of the simulated world
        iii. Call :py:meth:`stop_run`
        
        3. Call :py:meth:`stop`
        
        Use :py:meth:`run` instead to perform the simulations and record them all at
        once.
        
        Calling :py:meth:`start` is only effective once per experiment.
        
        :param path:
            A path to set optionally as :py:attr:`file_path`. If set, it will save
            an HDF5 file, but no YAML, to this path. If not set, :py:attr:`file_path` will be automatically set to
            ``<save_directory>/data.h5`` if :py:attr:`save_directory` is set.
        """
    def start_run(self, run: ExperimentalRun) -> None:
        """
        Start recording a run
        
        See :py:meth:`start`. This is only need when using an external run-loop to
        simulate.
        
        Call :py:meth:`start_run` only once per run.
        
        :param run:
            The run being recorded
        """
    def stop(self, save_runs: bool = False) -> None:
        """
        Signal to stop an experiment
        
        :param save_runs:
            Whether to save the runs before closing
        
        See :py:meth:`start`. This is only need when using an external run-loop to
        simulate.
        """
    def stop_run(self, run: ExperimentalRun) -> None:
        """
        Stop the run recording
        
        See :py:meth:`start_run`. This is only need to use an external run-loop.
        
        :param run:
            The run being recorded
        """
    def update_run(self, run: ExperimentalRun) -> None:
        """
        { function_description }
        
        See :py:meth:`start`. This is only need when using an external run-loop to
        simulate.
        
        Call :py:meth:`update_run` every time you need to record the current state
        of the world, typically after each simulation step.
        
        :param run:
            The run being recorded
        """
    @property
    def begin_time(self) -> datetime.datetime:
        """
        The system time when the experiment began.
        """
    @property
    def duration(self) -> datetime.timedelta:
        """
        The duration required to perform the whole experiment.
        """
    @property
    def has_finished(self) -> bool:
        """
        Determines if the experiment has finished.
        """
    @property
    def is_running(self) -> bool:
        """
        Determines if the experiment is running.
        """
    @property
    def name(self) -> str:
        """
        The name of the experiment
        """
    @name.setter
    def name(self, arg0: str) -> None:
        ...
    @property
    def number_of_runs(self) -> int:
        """
        Default number of runs to perform
        """
    @number_of_runs.setter
    def number_of_runs(self, arg0: int) -> None:
        ...
    @property
    def path(self) -> pathlib.Path | None:
        """
        The path where the experimental data has been saved.
        """
    @property
    def record_config(self) -> RecordConfig:
        """
        Which data to record
        """
    @record_config.setter
    def record_config(self, arg0: RecordConfig) -> None:
        ...
    @property
    def run_index(self) -> int:
        """
        The seed/index of the next run
        """
    @run_index.setter
    def run_index(self, arg0: int) -> None:
        ...
    @property
    def runs(self) -> dict[int, ExperimentalRun]:
        """
        The map of recorded runs.
        
        runs are associated to the seed used to initialize them.
        """
    @property
    def save_directory(self) -> os.PathLike:
        """
        Where to save the results
        """
    @save_directory.setter
    def save_directory(self, arg0: os.PathLike) -> None:
        ...
    @property
    def scenario(self) -> Scenario:
        """
        The scenario used to initialize a world
        """
    @scenario.setter
    def scenario(self, arg1: Scenario) -> None:
        ...
    @property
    def steps(self) -> int:
        """
        The default maximal number of steps to simulate during each run.
        """
    @steps.setter
    def steps(self, arg1: float) -> None:
        ...
    @property
    def terminate_when_all_idle_or_stuck(self) -> bool:
        """
        Whether to terminate when all agents are idle or stuck.
        """
    @terminate_when_all_idle_or_stuck.setter
    def terminate_when_all_idle_or_stuck(self, arg1: bool) -> None:
        ...
    @property
    def time_step(self) -> float:
        """
        The default time step used for simulation during each run
        """
    @time_step.setter
    def time_step(self, arg1: float) -> None:
        ...
    @property
    def scenario_init_callback(self) -> typing.Optional[ScenarioInitCallback]:
        ...
    @scenario_init_callback.setter
    def scenario_init_callback(self, value: typing.Optional[ScenarioInitCallback]) -> None:
        ...
class ExperimentalRun:
    """
    Simulates a world and collects data.
    """
    def reset(self) -> None:
        ...
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, world: World, time_step: float = 0.1, steps: int = 1000, terminate_when_all_idle_or_stuck: bool = True, record_config: RecordConfig = ..., seed: int = 0) -> None:
        """
        @private
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add_group_record_probe(self, key: str, probe_cls: typing.Callable[[], GroupRecordProbe]) -> GroupRecordProbe:
        """
        Adds a group record probe.
        
        :param key: the key of the group to be created
        :type key: str
        
        :param probe_cls: the probe class
        :type key: Type[sim.GroupRecordProbe]
        """
    def add_probe(self, probe: Probe) -> None:
        """
        Adds a probe
        
        :param probe:
            The probe
        """
    @typing.overload
    def add_record(self, key: str, data: numpy.ndarray) -> Dataset:
        """
        Adds a record.
        
        :param key: the record key
        :type key: str
        
        :param data: Copies shape, data and dtype from this numpy array
        :type data: :py:class:`numpy.ndarray`
        """
    @typing.overload
    def add_record(self, key: str, dtype: typing.Any, item_shape: list[int] = []) -> Dataset:
        """
        Adds a record.
        
        :param key: The record key
        :type key: str
        
        :param dtype: The type of data to store
        :type dtype: Any object that is convertible to a :py:class:`numpy.dtype`.
        
        :param item_shape: The shape of all axis except the first.
                           Leave empty to instantiate a flat dataset.
        :type item_shape:  List[int]
        """
    def add_record_probe(self, key: str, probe_cls: typing.Type[RecordProbe]) -> RecordProbe:
        """
        Adds a record probe.
        
        :param key: the key of the record to be created
        :type key: str
        
        :param probe_cls: the probe class
        :type key: Type[sim.RecordProbe]
        """
    def get_collisions_at_step(self, step: int) -> set[tuple[Entity, Entity]]:
        """
        Gets recorded collisions at a given step.
        
        :param step:
            The step. Negative steps are interpreted as relative to the last
            registered step, i.e., -1 is the last step.
        
        :return:
            The set of colliding entity pairs.
        """
    def get_record(self, key: str = '') -> Dataset:
        """
        Gets the record associated to a given key.
        
        :param key:
            The key
        
        :return:
            The record or null if none is found.
        """
    def get_record_names(self, group: str = '') -> set[str]:
        """
        Gets the names of records.
        
        :param group:
            An optional group. If specified, the looks for record associated
            to keys ``<group>/...``.
        
        :return:
            The record names (relative to the group if specified).
        """
    def get_records(self, group: str = '') -> dict[str, Dataset]:
        """
        Gets the records.
        
        :param group:
            If specified, limits to records in a given group.
        
        :return:
            The records
        """
    @typing.overload
    def get_task_events(self, agent: Agent) -> numpy.ndarray:
        """
        The recorded events logged by the task of an agent as a numpy array of shape 
        ``(number events, size of event log)`` and dtype ``float``::
        
          [[data_0, ...], 
           ...]
        
        The array is empty if the agent's task has not been recorded in the run.
        
        :param agent: The agent
        
        :return: The events logged by the agent task
        """
    @typing.overload
    def get_task_events(self, uid: int) -> numpy.ndarray:
        """
        The recorded events logged by the task of an agent as a numpy array of shape 
        ``(number events, size of event log)`` and dtype ``float``::
        
          [[data_0, ...], 
           ...]
        
        The array is empty if the agent's task has not been recorded in the run.
        
        :param uid: The uid of the agent
        
        :return: The events logged by the agent task
        """
    def go_to_step(self, step: int, ignore_collisions: bool = False, ignore_twists: bool = False, ignore_cmds: bool = False) -> bool:
        """
        Try to advance the world to a given recorded step.
        
        Depending if the data has been recorded, it will update:
        
        - poses - twists - cmds - time - collisions
        
        :param step:
            The step. Negative steps are interpreted as relative to the last
            registered step, i.e., -1 is the last step.
        
        :param ignore_twists:
            Whether to skip setting twists
        
        :param ignore_cmds:
            Whether to skip setting [last] commands
        
        :param ignore_collisions:
            Whether to skip setting collisions
        
        :return:
            True if the operation was possible and false otherwise.
        """
    def index_of_agent(self, agent: Agent) -> int | None:
        """
        Associate an index to a given agent.
        
        Data related to agents is stored in this order. For example, poses at
        a given time step are stored as ``[[x_0, y_0, z_0], [x_1, y_1, z_1],
        ...]``
        
        where the pose of agent a is at the index returned by this function.
        
        :param agent:
            The agent
        
        :return:
            The index of data related to this agent.
        """
    def run(self) -> None:
        """
        Runs a simulation and automatically record data step by step
        
        You don't need to call :py:meth:`start`, :py:meth:`update`, :py:meth:`stop` beside :py:meth:`run` as :py:meth:`run` manages all the simulation and recording.
        
        You don't need to call :py:meth:`run` when running an :py:class:`Experiment` (see
        :py:meth:`Experiment.run` instead) but only if you are want to perform a
        run outside of an :py:class:`Experiment`.
        """
    @property
    def collisions(self) -> numpy.ndarray:
        """
        The recorded collisions between pairs of entities as
        as a numpy array of shape ``(number of collisions, 3)``
        and dtype ``np.uint32``::
        
          [[time_step, uid_0, uid_1], 
           ...]
        
        The array is empty if collisions have not been recorded in the run.
        """
    @property
    def commands(self) -> numpy.ndarray:
        """
        The recorded commands of the agents as a numpy array of shape 
        ``(simulation steps, number of agents, 3)`` and dtype ``float``::
        
          [[[vx_0, vy_0, omega_0], 
            [vx_1, vy_1, omega_1], 
            ...], 
           ...]
          
        The array is empty if commands have not been recorded in the run.
        """
    @property
    def actuated_commands(self) -> numpy.ndarray:
        ...
    @property
    def deadlocks(self) -> numpy.ndarray:
        """
        The time since agents are deadlocked as a numpy array of shape 
        ``(number of agents, )`` and dtype ``float``::
        
          [time_0, time_1, ...]
        
        If ``time_i`` is negative, the i-th agent is not stuck at the end of the recording.
        Else, it has been stuck since ``time_i``.
        
        The array is empty if deadlocks have not been recorded in the run.
        """
    @property
    def duration(self) -> datetime.timedelta:
        """
        The real-time duration of the run.
        """
    @property
    def efficacy(self) -> numpy.ndarray:
        """
        The recorded agents' efficacy as a numpy array of shape 
        ``(simulation steps, number of agents)`` and dtype ``float``::
        
          [[efficacy_0, efficacy_1, ...],
           ...]
        
        The array is empty if efficacy has not been recorded in the run.
        """
    @property
    def final_sim_time(self) -> float:
        """
        The last recorded simulation time
        """
    @property
    def has_finished(self) -> bool:
        """
        Whether the run is finished.
        """
    @property
    def maximal_steps(self) -> int:
        """
        The number of steps execute
        """
    @property
    def poses(self) -> numpy.ndarray:
        """
        The recorded poses of the agents as a numpy array of shape 
        ``(simulation steps, number of agents, 3)`` and dtype ``float``::
        
          [[[x_0, y_0, theta_0], 
            [x_1, y_1, theta_1], 
            ...], 
           ...]
          
        The array is empty if poses have not been recorded in the run.
        """
    @property
    def record_config(self) -> RecordConfig:
        """
        Which data to record
        """
    @property
    def record_names(self) -> set[str]:
        """
        The names of records.
        
        :param group:
            an optional group. if specified, the looks for record associated
            to keys ``<group>/...``.
        """
    @property
    def recorded_steps(self) -> int:
        """
        The number of recorded steps.
        """
    @property
    def records(self) -> dict[str, Dataset]:
        """
        The records.
        
        :param group:
            if specified, limits to records in a given group.
        """
    @property
    def safety_violations(self) -> numpy.ndarray:
        """
        The recorded amounts of safety violation as a numpy array of shape 
        ``(simulation steps, number of agents)`` and dtype ``float``::
        
          [[violation_0, violation_1, ...],
           ...]
        
        where a value of 0 represents no violations.
        
        The array is empty if safety violations have not been recorded in the run.
        """
    @property
    def seed(self) -> int:
        """
        The seed used to initialize the simulation.
        """
    @property
    def targets(self) -> numpy.ndarray:
        """
        The recorded targets of the agents as a numpy array of shape 
        ``(simulation steps, number of agents, 3)`` and dtype ``float``::
        
          [[[x_0, y_0, theta_0], 
            [x_1, y_1, theta_1], 
            ...], 
           ...]
          
        The array is empty if targets have not been recorded in the run.
        """
    @property
    def task_events(self) -> dict[int, numpy.ndarray]:
        """
        The recorded events logged by the tasks all agents as a dictionart of numpy array of shape 
        ``(number events, size of event log)`` and dtype ``float``::
        
          {<agent_uid>: [[data_0, ...], ...]}
        
        The array are empty if the agent's task has not been recorded in the run.
        
        :return: The events logged by the tasks all agents
        """
    @property
    def terminate_when_all_idle_or_stuck(self) -> bool:
        """
        Whether to terminate when all agents are idle or stuck.
        """
    @property
    def time_step(self) -> float:
        """
        The time step used for simulation.
        """
    @property
    def times(self) -> numpy.ndarray:
        """
        The recorded simulation times as a numpy array of shape
        ``(simulation steps)`` and dtype ``float``::
        
          [t_0, t_1, ...]
          
        The array is empty if times have not been recorded in the run.
        """
    @property
    def twists(self) -> numpy.ndarray:
        """
        The recorded twists of the agents as a numpy array of shape 
        ``(simulation steps, number of agents, 3)`` and dtype ``float``::
        
          [[[vx_0, vy_0, omega_0], 
            [vx_1, vy_1, omega_1], 
            ...], 
           ...]
          
        The array is empty if twist have not been recorded in the run.
        """
    @property
    def world(self) -> World:
        """
        Returns the simulated world.
        """
    def get_collision_events(self, min_interval: int) -> numpy.ndarray:
        ...
    @property
    def bounding_box(self) -> BoundingBox:
        """
        The bounding box.
        """
class GroupRecordProbe(Probe):
    """
    Base class for probes that record a group of datasets, possibly to be
    saved in HDF5. Records are keyed by strings.
    
    Subclasses are expected to overwrite :py:meth:`Probe.update`, :py:attr:`shapes` and to redefine :py:class:`Type`.
    """
    def __init__(self, factory: typing.Callable[[str], Dataset] | None = None) -> None:
        """
        Constructor
        
        @private
        
        :param factory:
            The datasets generator
        """
    def get_data(self, key: str) -> Dataset:
        """
        Gets the recorded data, possibly after instanting a dataset if none is
        yet associated to the key.
        
        :param key:
            The key
        
        :return:
            The data.
        """
class LidarStateEstimation(Sensor, StateEstimation):
    """
    A distance scanner.
    
    *Registered properties*:
    
    - `range` (float, :py:attr:`range`)
    
    - `start_angle` (float, :py:attr:`start_angle`)
    
    - `field_of_view` (float, :py:attr:`field_of_view`)
    
    - `resolution` (float, :py:attr:`resolution`)
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, range: float = 0.0, start_angle: float = -3.141592653589793, field_of_view: float = 6.283185307179586, resolution: int = 100) -> None:
        """
        Constructs a new instance.
        
        :param range_:
            The maximal range of the sensor
        
        :param start_angle_:
            The starting angle
        
        :param field_of_view_:
            The field of view
        
        :param resolution_:
            The number of ranging measurements per scan
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def field_of_view(self) -> float:
        """
        The range of view.
        """
    @field_of_view.setter
    def field_of_view(self, arg1: float) -> None:
        ...
    @property
    def range(self) -> float:
        """
        The range of view.
        """
    @range.setter
    def range(self, arg1: float) -> None:
        ...
    @property
    def resolution(self) -> int:
        """
        The range of view.
        """
    @resolution.setter
    def resolution(self, arg1: int) -> None:
        ...
    @property
    def start_angle(self) -> float:
        """
        The range of view.
        """
    @start_angle.setter
    def start_angle(self, arg1: float) -> None:
        ...
class BoundarySensor(Sensor, StateEstimation):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, range: float = 0.0, min_x: float = -numpy.inf, min_y: float = -numpy.inf, max_x: float = numpy.inf, max_y: float = numpy.inf) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def min_x(self) -> float:
        ...
    @min_x.setter
    def min_x(self, arg1: float) -> None:
        ...
    @property
    def min_y(self) -> float:
        ...
    @min_y.setter
    def min_y(self, arg1: float) -> None:
        ...
    @property
    def max_x(self) -> float:
        ...
    @max_x.setter
    def max_x(self, arg1: float) -> None:
        ...
    @property
    def max_y(self) -> float:
        ...
    @max_y.setter
    def max_y(self, arg1: float) -> None:
        ...
    @property
    def range(self) -> float:
        ...
    @range.setter
    def range(self, arg1: float) -> None:
        ...
class SensorCombination(Sensor, StateEstimation):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, sensors: list[Sensor] = []) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def sensors(self) -> list[Sensor]:
        ...
    @sensors.setter
    def sensors(self, arg1: list[Sensor]) -> None:
        ...
class NativeAgent(Entity):
    """
    This class describes an agent.
    
    The agent navigates in the environment using a task, a state
    estimation, a kinematic and a behavior, and a controller.
    
    Agents have a circular shape which should match the shape of their
    navigation :py:class:`Behavior`.
    
    The role of task and state estimation is to provide goals and
    environment state (perception) to the behavior.
    
    Agents have a public identifies :py:attr:`id` that is accessible by the
    other agents' state estimation and may be passed to their behavior as
    :py:attr:`Neighbor.id`. This identifier may not be unique
    (e.g., may be used to identifies *groups* of agents).
    
    Agents runs their update at the rate set by :py:attr:`control_period` even
    if the world is updated at a faster rate.
    """
    def actuate(self, cmd: core.Twist2, time_step: float) -> None:
        """
        Actuate the current agent control command.
        
        :param dt:
            The time step
        """
    @property
    def angular_speed(self) -> float:
        """
        Angular speed
        """
    @angular_speed.setter
    def angular_speed(self, arg1: float) -> None:
        ...
    @property
    def behavior(self) -> core.Behavior:
        """
        The navigation behavior.
        """
    @behavior.setter
    def behavior(self, arg1: core.Behavior) -> None:
        ...
    @property
    def color(self) -> str:
        """
        The color of the agent.
        
        A valid CSS color to fill the agent in the UI or empty to use the
        default color.
        """
    @color.setter
    def color(self, arg0: str) -> None:
        ...
    @property
    def control_period(self) -> float:
        """
        The control period
        """
    @control_period.setter
    def control_period(self, arg0: float) -> None:
        ...
    @property
    def controller(self) -> core.Controller:
        """
        The navigation controller.
        """
    @property
    def id(self) -> int:
        """
        The agent public identifier
        """
    @id.setter
    def id(self, arg0: int) -> None:
        ...
    @property
    def idle(self) -> bool:
        """
        Returns whether the task is done and the control is idle.
        
        :return:
            False if it has an active task or if the control is running
        """
    @property
    def kinematics(self) -> core.Kinematics:
        """
        The kinematics.
        """
    @kinematics.setter
    def kinematics(self, arg1: core.Kinematics) -> None:
        ...
    @property
    def last_cmd(self) -> core.Twist2:
        """
        The last control command
        """
    @last_cmd.setter
    def last_cmd(self, arg0: core.Twist2) -> None:
        ...
    @property
    def orientation(self) -> float:
        """
        Orientation
        """
    @orientation.setter
    def orientation(self, arg1: float) -> None:
        ...
    @property
    def pose(self) -> core.Pose2:
        """
        The current pose
        """
    @pose.setter
    def pose(self, arg0: core.Pose2) -> None:
        ...
    @property
    def position(self) -> numpy.ndarray:
        """
        Position
        """
    @position.setter
    def position(self, arg1: Vector2Like) -> None:
        ...
    @property
    def radius(self) -> float:
        """
        The agent radius
        """
    @radius.setter
    def radius(self, arg0: float) -> None:
        ...
    @property
    def state_estimation(self) -> StateEstimation:
        """
        The state estimation.
        """
    @state_estimation.setter
    def state_estimation(self, arg1: StateEstimation) -> None:
        ...
    @property
    def tags(self) -> set[str]:
        """
        A set of tags used to label the agent.
        
        Tags are mainly used to add meta-information about an agent, for
        instance by the :py:class:`Group` that generated it
        (if any), to simplify analysis.
        """
    @tags.setter
    def tags(self, value: set[str]) -> None:
        ...
    def add_tag(self, value: str) -> None:
        ...
    def remove_tag(self, value: str) -> None:
        ...
    @property
    def task(self) -> Task:
        """
        The task.
        """
    @task.setter
    def task(self, arg1: Task) -> None:
        ...
    @property
    def twist(self) -> core.Twist2:
        """
        The current twist
        """
    @twist.setter
    def twist(self, arg0: core.Twist2) -> None:
        ...
    @property
    def type(self) -> str:
        """
        The type of the agent.
        
        The agent type should not used by the neighbors state estimation. It
        is mainly used internally to draw the agents in the UI.
        """
    @type.setter
    def type(self, arg0: str) -> None:
        ...
    @property
    def velocity(self) -> numpy.ndarray:
        """
        Velocity
        """
    @velocity.setter
    def velocity(self, arg1: Vector2Like) -> None:
        ...
    def has_been_stuck_since(self, arg1: float) -> bool:
        ...

class NativeWorld:
    """
    """
    def __init__(self) -> None:
        """
        Constructs a new instance.
        """
    def _prepare(self) -> None:
        ...
    def actuate(self, time_step: float) -> None:
        """
        Actuate then controllers and perform collisions resolutions.
        
        :param time_step:
            The duration of each time step
        """
    def add_agent(self, agent: Agent) -> None:
        """
        Adds an agent to the world.
        
        :param agent:
            The agent
        """
    def add_callback(self, callback: typing.Callable[[], None]) -> None:
        """
        Adds a callback to be executed after each simulation step.
        
        :param value:
            The callback
        """
    @typing.overload
    def add_obstacle(self, disc: core.Disc) -> None:
        """
        Adds a disc the world as a static obstacle
        
        :param disc:
            The disc
        """
    @typing.overload
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """
        Adds a static obstacle the world
        
        :param obstacle:
            The obstacle
        """
    def add_random_obstacles(self, number: int, min_radius: float, max_radius: float, margin: float = 0.0, max_tries: int = 1000) -> None:
        """
        Adds a random obstacles in the world bounding box
        
        It iteratively try to sample a disc that is far enough of any other
        item. If the sampling fails, it counts as a try and the process stop
        after either enough obstacles have been added or enough tries
        performed.
        
        :param number:
            The number of obstacles
        
        :param min_radius:
            The minimum radius of obstacles
        
        :param max_radius:
            The maximum radius of obstacles
        
        :param margin:
            The minimal distance to other obstacles or agents. For agents,
            it's additional to their safety margin.
        
        :param max_tries:
            The maximum tries before terminating.
        """
    @typing.overload
    def add_wall(self, line: core.LineSegment) -> None:
        """
        Adds a line to the world as a wall
        
        :param line:
            The line
        """
    @typing.overload
    def add_wall(self, wall: Wall) -> None:
        """
        Adds a wall to the world
        
        :param wall:
            The wall
        """
    def agents_are_idle(self) -> bool:
        """
        Check if all agents are idle (i.e., their tasks are done and their
        controller are idle).
        
        :return:
            True if all agents are idle
        """
    def agents_are_idle_or_stuck(self) -> bool:
        """
        Check if all agents are idle or stuck (i.e., they are no moving
        because they task is done or they are deadlocked)
        
        :return:
            True if all agents are idle or stuck
        """
    def compute_safety_violation(self, agent: Agent, safety_margin: float | None = None) -> float:
        """
        Calculates the safety violation, i.e. the maximal penetration of a
        neighbor or obstacle in the safety margin of the agent.
        
        :param agent:
            The agent
        
        :return:
            The safety violation or 0 if no violation.
        """
    def copy_random_generator(self, world: World) -> None:
        """
        Copy the random generator from another world
        
        :param world:
            The world
        """
    def get_agents_in_collision(self, duration: float = 0.0) -> list[Agent]:
        """
        Gets the agents that had a collision after ``now - duration``.
        
        :param duration:
            The duration
        
        :return:
            The agents in collision.
        """
    def get_agents_in_deadlock(self, duration: float = 0.0) -> list[Agent]:
        """
        Gets the agents that are in stuck since ``now - duration``.
        
        :param duration:
            The duration
        
        :return:
            The agents in deadlock.
        """
    def get_agents_in_region(self, bounding_box: BoundingBox) -> list[Agent]:
        """
        Gets all agents in a bounding box.
        
        :param bb:
            The bounding box specified in world-fixed coordinates
        
        :return:
            All agents that lie in a bounding box.
        """
    def get_discs_in_region(self, bounding_box: BoundingBox, ignore_lattice: bool = False) -> list[core.Disc]:
        """
        Gets all agents in a bounding box.
        
        :param bb:
            The bounding box specified in world-fixed
        
        :param ignore_lattice:
            Whether to ignore the lattice when computing neighbors
        
        :return:
            All obstacles that lie in a bounding box
        """
    def get_entity(self, uid: int) -> Entity:
        """
        Find an entity by identifier
        
        :param uid:
            The entity uid
        
        :return:
            The entity or nullptr if not found.
        """
    def get_lattice(self, axis: int) -> tuple[float, float] | None:
        """
        Gets the periodic lattice.
        
        :param axis:
            The axis (0 for x, 1 for y)
        
        :return:
            An optional tuple of points that define a periodic lattice that
            wraps the selected axis.
        """
    def get_lattice_grid(self, include_zero: bool = True, c8: bool = True) -> list[numpy.ndarray]:
        """
        The N=0, 1, or 2 vectors that define the lattice, e.g., ``{delta_x,
        -delta_x}`` if only the axis=0 lattice is set.
        
        :param Whether:
            to include the zero vector
        
        :param Whether:
            to use 8-connectivity instead of 4-connectivity
        
        :return:
            A vector of 2D vectors
        """
    def get_line_obstacles_in_region(self, bounding_box: BoundingBox) -> list[core.LineSegment]:
        """
        Gets all walls in a bounding box.
        
        :param bb:
            The bounding box specified in world-fixed
        
        :return:
            All walls that lie in a bounding box
        """
    def get_neighbors(self, agent: Agent, distance: float, ignore_lattice: bool = False) -> list[core.Neighbor]:
        """
        Gets all neighbor of an agent
        
        :param agent:
            The agent
        
        :param distance:
            The radius of the neighborhood
        
        :param ignore_lattice:
            Whether to ignore the lattice when computing neighbors
        
        :return:
            All neighbor within a circle of radius ``radius`` centered around
            the agent.
        """
    def get_obstacles_in_region(self, bounding_box: BoundingBox) -> list[Obstacle]:
        """
        Gets all obstacles in a bounding box.
        
        :param bb:
            The bounding box specified in world-fixed coordinates
        
        :return:
            All obstacles that lie in a bounding box.
        """
    def in_collision(self, e1: Entity, e2: Entity) -> bool:
        """
        Check if two entities are currently in collision
        
        :param e1:
            The first entity
        
        :param e2:
            The second entity
        
        :return:
            True if they are in collision.
        """
    def record_collision(self, e1: Entity, e2: Entity) -> None:
        ...
    def clear_collisions(self) -> None:
        ...
    def index_of_agent(self, agent: Agent) -> int | None:
        """
        Searches for the index of an agent.
        
        :param agent:
            The agent
        
        :return:
            The index of this agent in the world agents list or null if not
            found.
        """
    def reset_callbacks(self) -> None:
        """
        Clear all the callbacks
        """
    def run(self, steps: int, time_step: float) -> None:
        """
        Updates the world for one or more time steps
        
        :param steps:
            The number of steps
        
        :param time_step:
            The duration of each time step
        """
    def run_until(self, condition: typing.Callable[[], bool], time_step: float) -> None:
        """
        Updates the world until a condition is satisfied
        
        :param condition:
            The condition
        
        :param time_step:
            The duration of each time step
        """
    def set_lattice(self, axis: int, value: tuple[float, float] | None) -> None:
        """
        Sets the periodic lattice.
        
        :param axis:
            The axis (0 for x, 1 for y)
        
        :param value:
            An optional tuple of points that define a periodic lattice that
            wraps the selected axis. Pass none to unset the lattice and remove
            wrapping.
        """
    def set_termination_condition(self, condition: typing.Callable[[World], bool] | None) -> None:
        """
        Sets a condition to terminate simulations
        
        :param value:
            The desired condition.
        """
    def snap_twists_to_zero(self, epsilon: float = 1e-06) -> None:
        """
        Snap agents' twists smaller than epsilon to zero.
        
        :param epsilon:
            The tolerance
        """
    def space_agents_apart(self, minimal_distance: float = 0, with_safety_margin: bool = False, max_iterations: int = 10) -> None:
        """
        Move agents so that they do not overlap anymore with themselves or
        with any obstacle
        
        :param minimal_distance:
            The minimal distance
        
        :param with_safety_margin:
            Whether the safety margin should be added to the minimal distance
        
        :param max_iterations:
            The maximal number of iterations to perform.
        """
    def subdivide_bounding_box(self, arg0: BoundingBox, arg1: bool) -> list[tuple[numpy.ndarray, BoundingBox]]:
        ...
    def update(self, time_step: float) -> None:
        """
        Updates world for a single time step.
        
        :param time_step:
            The time step
        """
    def update_dry(self, time_step: float, advance_time: bool = True) -> None:
        """
        Updates world for a single time step without actuation and collisions
        resolution.
        
        :param time_step:
            The time step
        
        :param advance_time:
            Whenever to advance time too.
        """
    @property
    def agents(self) -> list[Agent]:
        """
        All agents in this world.
        """
    @property
    def bounding_box(self) -> BoundingBox:
        """
        The bounding box.
        """
    @bounding_box.setter
    def bounding_box(self, arg1: BoundingBox | None) -> None:
        ...
    @property
    def collisions(self) -> set[tuple[Entity, Entity]]:
        """
        The colliding pairs computed during the last simulation step.
        """
    @collisions.setter
    def collisions(self, arg1: set[tuple[Entity, Entity]]) -> None:
        ...
    @property
    def discs(self) -> list[core.Disc]:
        """
        All disc shaped static obstacles in this world.
        
        :param ignore_lattice:
            whether to ignore the lattice
        """
    @property
    def has_termination_condition(self) -> bool:
        """
        Returns whether there is a termination condition set.
        
        :return:
            True if a termination condition has been set.
        """
    @property
    def line_obstacles(self) -> list[core.LineSegment]:
        """
        All line obstacles in this world.
        """
    @property
    def minimal_bounding_box(self) -> BoundingBox:
        """
        The bounding box that contains all agents, obstacles and walls.
        """
    @property
    def obstacles(self) -> list[Obstacle]:
        """
        All obstacles in this world.
        """
    @property
    def seed(self) -> int:
        """
        The random seed.
        """
    @seed.setter
    def seed(self, arg1: int) -> None:
        ...
    @property
    def step(self) -> int:
        """
        The simulation step.
        """
    @step.setter
    def step(self, arg1: int) -> None:
        ...
    @property
    def time(self) -> float:
        """
        The simulation time.
        """
    @time.setter
    def time(self, arg1: float) -> None:
        ...
    @property
    def walls(self) -> list[Wall]:
        """
        All walls in this world.
        """
    def should_terminate(self) -> bool:
        ...

class Obstacle(Entity):
    """
    A static obstacle with circular shape
    """
    @typing.overload
    def __init__(self, position: Vector2Like, radius: float) -> None:
        """
        Constructs a new instance.
        
        :param position:
            The position of the circle
        
        :param radius:
            The radius of the circle
        """
    @typing.overload
    def __init__(self, disc: core.Disc) -> None:
        """
        Constructs a new instance.
        
        :param disc:
            A disc
        """
    @property
    def disc(self) -> core.Disc:
        """
        The disc.
        """
    @disc.setter
    def disc(self, arg0: core.Disc) -> None:
        ...
class Probe:
    """
    The base class for all probes.
    
    Probes are callbacks called at the begin, at the end, and during each
    simulation step of an :py:class:`ExperimentalRun`.
    
    Concrete classes should overwrite :py:meth:`prepare`, :py:meth:`update`, and/or
    :py:attr:`finalize` as the base class does nothing.
    """
    def __init__(self) -> None:
        ...
    def _prepare(self, run: ExperimentalRun) -> None:
        ...
    def _update(self, run: ExperimentalRun) -> None:
        ...
    def _finalize(self, run: ExperimentalRun) -> None:
        ...
class RecordConfig:
    """
    Holds which data to record during an experimental run
    """
    @staticmethod
    def all(value: bool) -> RecordConfig:
        """
        Sets all to record or not record.
        
        :return:
            The configuration.
        """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, time: bool = False, pose: bool = False, twist: bool = False, cmd: bool = False, target: bool = False, safety_violation: bool = False, collisions: bool = False, task_events: bool = False, deadlocks: bool = False, efficacy: bool = False) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def set_all(self, value: bool) -> None:
        """
        Sets all to record or not record.
        
        :param value:
            The desired value
        """
    @property
    def cmd(self) -> bool:
        """
        Whether to record the agents control commands
        """
    @cmd.setter
    def cmd(self, arg0: bool) -> None:
        ...
    @property
    def actuated_cmd(self) -> bool:
        ...
    @actuated_cmd.setter
    def actuated_cmd(self, arg0: bool) -> None:
        ...
    @property
    def collisions(self) -> bool:
        """
        Whether to record collisions
        """
    @collisions.setter
    def collisions(self, arg0: bool) -> None:
        ...
    @property
    def deadlocks(self) -> bool:
        """
        Whether to record the time since when agents are stuck
        """
    @deadlocks.setter
    def deadlocks(self, arg0: bool) -> None:
        ...
    @property
    def efficacy(self) -> bool:
        """
        Whether to record efficacy (i.e., fraction of actual velocity vs
        optimal velocity)
        """
    @efficacy.setter
    def efficacy(self, arg0: bool) -> None:
        ...
    @property
    def pose(self) -> bool:
        """
        Whether to record the agents poses
        """
    @pose.setter
    def pose(self, arg0: bool) -> None:
        ...
    @property
    def safety_violation(self) -> bool:
        """
        Whether to record safety violations
        """
    @safety_violation.setter
    def safety_violation(self, arg0: bool) -> None:
        ...
    @property
    def target(self) -> bool:
        """
        Whether to record the agents targets
        """
    @target.setter
    def target(self, arg0: bool) -> None:
        ...
    @property
    def task_events(self) -> bool:
        """
        Whether to record data from task events
        """
    @task_events.setter
    def task_events(self, arg0: bool) -> None:
        ...
    @property
    def time(self) -> bool:
        """
        Whether to record the simulation time
        """
    @time.setter
    def time(self, arg0: bool) -> None:
        ...
    @property
    def twist(self) -> bool:
        """
        Whether to record the agents twists
        """
    @twist.setter
    def twist(self, arg0: bool) -> None:
        ...
    @property
    def world(self) -> bool:
        ...
    @world.setter
    def world(self, arg0: bool) -> None:
        ...
class RecordProbe(Probe):
    """
    Base class for probes that records numerical data on a single dataset,
    possibly to be saved in HDF5.
    
    Subclasses are expected to overwrite :py:meth:`Probe.update`, :py:attr:`shape` and to redefine :py:class:`Type`.
    """
    def __init__(self, record: Dataset | None = None) -> None:
        """
        The constructor
        
        @private
        
        :param data:
            The data
        """
    @property
    def data(self) -> Dataset:
        """
        The recorded data
        """
class Scenario(ScenarioRegister, HasProperties):
    """
    A scenario describes a distribution of :py:class:`World` that can be sampled
    to perform an experiment.
    """
    class Group:
        """
        A group of agents that can be generated and added to the world.
        """
        def __init__(self) -> None:
            ...
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add_init(self, initializer: typing.Callable[[World], None]) -> None:
        """
        Adds a world initializer.
        
        :param f:
            The initializer
        """
    def init_world(self, world: World, seed: int | None = None) -> None:
        """
        Initializes the world.
        
        :param world:
            The world
        
        :param seed:
            The random seed
        """
    def set_yaml(self, arg0: str) -> None:
        ...
    @property
    def obstacles(self) -> list[core.Disc]:
        """
        Obstacles
        """
    @obstacles.setter
    def obstacles(self, arg0: list[core.Disc]) -> None:
        ...
    @property
    def walls(self) -> list[core.LineSegment]:
        """
        Walls
        """
    @walls.setter
    def walls(self, arg0: list[core.LineSegment]) -> None:
        ...
class ScenarioRegister:
    type_properties: typing.ClassVar[dict]  # value = {'Antipodal': {'orientation_noise': <core.Property object>, 'position_noise': <core.Property object>, 'radius': <core.Property object>, 'shuffle': <core.Property object>, 'tolerance': <core.Property object>}, 'Corridor': {'add_safety_to_agent_margin': <core.Property object>, 'agent_margin': <core.Property object>, 'length': <core.Property object>, 'width': <core.Property object>}, 'Cross': {'add_safety_to_agent_margin': <core.Property object>, 'agent_margin': <core.Property object>, 'side': <core.Property object>, 'target_margin': <core.Property object>, 'tolerance': <core.Property object>}, 'CrossTorus': {'add_safety_to_agent_margin': <core.Property object>, 'agent_margin': <core.Property object>, 'side': <core.Property object>}, 'Simple': {}}
    types: typing.ClassVar[list] = ['Antipodal', 'Corridor', 'Cross', 'CrossTorus', 'Simple']
    @staticmethod
    def _add_property(arg0: str, arg1: str, arg2: typing.Any, arg3: bool | int | float | str | numpy.ndarray | list[bool] | list[int] | list[float] | list[str] | list[numpy.ndarray], arg4: str, arg5: list[str]) -> None:
        ...
    @staticmethod
    def _register_type(arg0: str, arg1: typing.Any) -> None:
        ...
    @staticmethod
    def make_type(name: str) -> typing.Any:
        """
        Create an object of a sub-class selected by name.
        
        :param type:
            The associated type name.
        
        :return:
            An object of a registered sub-class or ``None`` in case the desired name is not found.
        """
class Sensor(StateEstimation):
    """
    Base class for agents using a :py:class:`SensingState`.
    """
    def __init__(self) -> None:
        """
        Constructs a new instance.
        """
    def prepare(self, arg0: core.SensingState) -> None:
        """
        @private
        """
    @property
    def description(self) -> dict[str, core.BufferDescription]:
        """
        The description of the buffers set by the sensors.
        """
class SimpleScenario(Scenario):
    """
    Simple scenario that serves as an example.
    
    The scenario add a single agent with a waypoints task, dummy behavior,
    and holonomic kinematics.
    
    *Registered properties*: none
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
class StateEstimation(StateEstimationRegister, HasProperties):
    """
    This class describe a generic state estimation that should update the
    environment state used by the agent :py:class:`Behavior`.
    
    As the environment state is specialized by sub-classes of :py:class:`Behavior` like :py:class:`GeometricState`,
    concrete sub-classes have to target one or more of them.
    
    In particular, the agent should use a state estimation compatible with
    it's state representation.
    
    :py:class:`StateEstimation` holds a pointer to the :py:class:`World` containing the
    agent, which it queries to get the relevant entities (located nearby
    the agent).
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        """
        Constructs a new instance. @private
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def update(self, agent: Agent, world: World, state: core.EnvironmentState) -> None:
        """
        Updates an environment state with respect to a given agent.
        
        :param agent:
            The agent owning the state estimation
        
        :param world:
            The world that the agent is part of
        
        :param state:
            The environment state to be updated
        """
    @property
    def type(self) -> str:
        """
        The name associated to the type of an object
        """
class StateEstimationRegister:
    type_properties: typing.ClassVar[dict]  # value = {'Bounded': {'range': <core.Property object>, 'update_static_obstacles': <core.Property object>}, 'Discs': {'include_valid': <core.Property object>, 'max_radius': <core.Property object>, 'max_speed': <core.Property object>, 'number': <core.Property object>, 'range': <core.Property object>}, 'Lidar': {'field_of_view': <core.Property object>, 'range': <core.Property object>, 'resolution': <core.Property object>, 'start_angle': <core.Property object>}, 'pyLidar': {'length': <core.Property object>, 'max_distance': <core.Property object>, 'resolution': <core.Property object>, 'start_angle': <core.Property object>}}
    types: typing.ClassVar[list] = ['Bounded', 'Discs', 'Lidar', 'pyLidar']
    @staticmethod
    def _add_property(arg0: str, arg1: str, arg2: typing.Any, arg3: bool | int | float | str | numpy.ndarray | list[bool] | list[int] | list[float] | list[str] | list[numpy.ndarray], arg4: str, arg5: list[str]) -> None:
        ...
    @staticmethod
    def _register_type(arg0: str, arg1: typing.Any) -> None:
        ...
    @staticmethod
    def make_type(name: str) -> typing.Any:
        """
        Create an object of a sub-class selected by name.
        
        :param type:
            The associated type name.
        
        :return:
            An object of a registered sub-class or ``None`` in case the desired name is not found.
        """
class Task(TaskRegister, HasProperties):
    """
    This class describe the high-level task control that provides
    navigation goals.
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add_callback(self, callback: typing.Callable[[list[float]], None]) -> None:
        """
        Adds a callback called to log task events.
        
        :param value:
            The desired callback
        """
    def done(self) -> bool:
        """
        Returns whether the task is done.
        
        :return:
            True if the task has finished.
        """
    @property
    def log_size(self) -> int:
        """
        The size of the data passed to callbacks when events occur, see :py:class:`TaskCallback` and :py:meth:`add_callback`.
        """
    @property
    def type(self) -> str:
        """
        The name associated to the type of an object
        """
class TaskRegister:
    type_properties: typing.ClassVar[dict]  # value = {'Waypoints': {'loop': <core.Property object>, 'random': <core.Property object>, 'tolerance': <core.Property object>, 'waypoints': <core.Property object>}}
    types: typing.ClassVar[list] = ['Waypoints']
    @staticmethod
    def _add_property(arg0: str, arg1: str, arg2: typing.Any, arg3: bool | int | float | str | numpy.ndarray | list[bool] | list[int] | list[float] | list[str] | list[numpy.ndarray], arg4: str, arg5: list[str]) -> None:
        ...
    @staticmethod
    def _register_type(arg0: str, arg1: typing.Any) -> None:
        ...
    @staticmethod
    def make_type(name: str) -> typing.Any:
        """
        Create an object of a sub-class selected by name.
        
        :param type:
            The associated type name.
        
        :return:
            An object of a registered sub-class or ``None`` in case the desired name is not found.
        """
class Wall(Entity):
    """
    A static wall.
    
    Currently, only line segment are valid shapes of walls.
    """
    @typing.overload
    def __init__(self, p1: Vector2Like, p2: Vector2Like) -> None:
        """
        Constructs a new instance.
        
        :param p1:
            The line segment start vertex
        
        :param p2:
            The line segment end vertex
        """
    @typing.overload
    def __init__(self, line: core.LineSegment) -> None:
        """
        Constructs a new instance.
        
        :param ls:
            A line segment
        """
    @property
    def line(self) -> core.LineSegment:
        """
        The line segment
        """
    @line.setter
    def line(self, arg0: core.LineSegment) -> None:
        ...
class DirectionTask(Task):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, direction: Vector2Like = (1, 0)) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def direction(self) -> numpy.ndarray:
        ...
    @direction.setter
    def direction(self, arg1: Vector2Like) -> None:
        ...
class WaypointsTask(Task):
    """
    This class implement a task that makes the agent reach a sequence of
    waypoints, calling :py:attr:`Controller.go_to_position`
    for the next waypoint after the current has been reached within a
    tolerance.
    
    The task notifies when a new waypoint is set by calling a callback.
    
    *Registered properties*:
    
    - `waypoints` (list of :py:class:`Vector2`, :py:attr:`waypoints`)
    
    - `loop` (bool, :py:attr:`loop`)
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, waypoints: list[Vector2Like] = [], loop: bool = True, tolerance: float = True) -> None:
        """
        Constructs a new instance.
        
        :param waypoints_:
            The waypoints
        
        :param loop_:
            Whether it should start from begin after reaching the last
            waypoint
        
        :param tolerance_:
            The goal tolerance applied to each waypoint.
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @property
    def log_size(self) -> int:
        """
        The size of the data passed to callbacks when events occur, see :py:class:`TaskCallback` and :py:meth:`add_callback`.
        
        The data is composed of 4 numbers: ``[time, target_x, target_y,
        target_theta]``
        """
    @property
    def loop(self) -> float:
        """
        Whether it should start from begin after reaching the last
        waypoint.
        """
    @loop.setter
    def loop(self, arg1: bool) -> None:
        ...
    @property
    def tolerance(self) -> float:
        """
        The goal tolerance applied to each waypoint.
        """
    @tolerance.setter
    def tolerance(self, arg1: float) -> None:
        ...
    @property
    def waypoints(self) -> list[numpy.ndarray]:
        """
        The waypoints.
        """
    @waypoints.setter
    def waypoints(self, arg1: list[Vector2Like]) -> None:
        ...
class World(NativeWorld):
    """
    """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        """
        Constructs a new instance.
        """
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add_agent(self, agent: Agent) -> None:
        """
        Adds an agent to the world.
        
        :param agent:
            The agent
        """

    @property
    def random_generator(self) -> numpy.random.Generator:
        ...

@typing.overload
def dump(task: Task) -> str:
    """
    Dump a task to a YAML-string
    """
@typing.overload
def dump(state_estimation: StateEstimation) -> str:
    """
    Dump a state_estimation to a YAML-string
    """
@typing.overload
def dump(world: World) -> str:
    """
    Dump a world to a YAML-string
    """
@typing.overload
def dump(scenario: Scenario) -> str:
    """
    Dump a scenario to a YAML-string
    """
@typing.overload
def dump(agent: Agent) -> str:
    """
    Dump an agent to a YAML-string
    """
@typing.overload
def dump(experiment: Experiment) -> str:
    """
    Dump an experiment to a YAML-string
    """
@typing.overload
def dump(behavior: core.Behavior) -> str:
    """
    Dump a behavior to a YAML-string
    """
@typing.overload
def dump(kinematics: core.Kinematics) -> str:
    """
    Dump a kinematics to a YAML-string
    """
def load_agent(value: str) -> Agent:
    """
    Load an agent from a YAML string.
    
    :return:
      The loaded agent or ``None`` if loading fails.
    """
def load_experiment(value: str) -> Experiment:
    """
    Load an experiment from a YAML string.
    
    :return:
      The loaded experiment or ``None`` if loading fails.
    """
def load_scenario(value: str) -> Scenario:
    """
    Load a scenario from a YAML string.
    
    :return:
      The loaded scenario or ``None`` if loading fails.
    """
def load_state_estimation(value: str) -> StateEstimation:
    """
    Load a state estimation from a YAML string.
    
    :return:
      The loaded state estimation or ``None`` if loading fails.
    """
def load_task(value: str) -> Task:
    """
    Load a task from a YAML string.
    
    :return:
      The loaded task or ``None`` if loading fails.
    """
def load_world(value: str) -> World:
    """
    Load a world from a YAML string.
    
    :return:
      The loaded world or ``None`` if loading fails.
    """
class RecordSensingConfig:
    def __init__(self, name: str = "", sensor: typing.Optional[Sensor] = None, agent_indices: list[int] = []) -> None:
        ...
    @property
    def name(self) -> str:
        ...
    @name.setter
    def name(self, value: str) -> None:
        ...
    @property
    def sensor(self) -> typing.Optional[Sensor]:
        ...
    @sensor.setter
    def sensor(self, value: typing.Optional[Sensor]) -> None:
        ...
    @property
    def agent_indices(self) -> list[int]:
        ...
    @agent_indices.setter
    def agent_indices(self, value: list[int]) -> None:
        ...
class SensingProbe(Probe):
    def __init__(self, name: str = "sensing", sensor: typing.Optional[Sensor] = None, agent_indices: list[int] = []) -> None:
        ...
    def get_data(self) -> typing.Dict[str, Dataset]:
        ...
