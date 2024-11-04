import datetime
import pathlib
import re
import sys
import warnings
from typing import TYPE_CHECKING, Dict, List, Optional, Set, Tuple, Union

import numpy as np
from navground import core

from . import (Agent, BoundingBox, RecordConfig, World, _Scenario,
               load_experiment, load_world)

if TYPE_CHECKING:
    import h5py  # type: ignore


def _timedelta_from_ns(ns: int):
    return datetime.timedelta(microseconds=ns / 1e3)


def _get_all_datasets(group: 'h5py.Group',
                      ns: str) -> Dict[str, 'h5py.Dataset']:
    import h5py
    rs: Dict[str, h5py.Dataset] = {}
    for k, v in group.items():
        if isinstance(v, h5py.Group):
            cs = _get_all_datasets(v, ns)
            rs.update(**cs)
        else:
            i = v.name.find(ns)
            n = v.name[i + len(ns):]
            rs[n] = v
    return rs


class RecordedExperimentalRun:
    """
    A recorded experimental run.

    All data is loaded from a HDF5 group
    and served with the same getter methods and attributes
    as :py:class:`navground.sim.ExperimentalRun` but limited to read-only.
    All setters are explicitly blocked.

    It holds a :py:class:`navground.sim.World`.
    At initialization, the world is set to the state at the begin of the recorded run.
    By repeatedly calling :py:meth:`do_step`, the world can be advanced,
    setting the state according to the data stored in the HDF5 group.
    """

    _frozen = False
    _mutable = {"_step"}
    world: World
    """The world that has been simulated"""

    def __init__(self,
                 group: 'h5py.Group',
                 scenario: _Scenario | None = None,
                 record_config: RecordConfig | None = None):
        """
        Constructs a new instance.

        :param      group:  The HDF5 group recorded by a :py:class:`navground.sim.ExperimentalRun`
        """
        self.record_config = record_config
        self._group = group
        self._scenario = scenario
        self.reset()
        self.collisions: Optional['h5py.Dataset'] = group.get('collisions')
        """The recorded collisions"""
        self.deadlocks: Optional['h5py.Dataset'] = group.get('deadlocks')
        """The recorded deadlocks"""
        self.efficacy: Optional['h5py.Dataset'] = group.get('efficacy')
        """The recorded efficacy"""
        self.commands: Optional['h5py.Dataset'] = group.get('cmds')
        """The recorded commands"""
        self.poses: Optional['h5py.Dataset'] = group.get('poses')
        """The recorded poses"""
        self.twists: Optional['h5py.Dataset'] = group.get('twists')
        """The recorded twists"""
        self.times: Optional['h5py.Dataset'] = group.get('times')
        """The recorded times"""
        self.targets: Optional['h5py.Dataset'] = group.get('targets')
        """The recorded targets"""
        self.task_events: Dict[int, 'h5py.Dataset'] = {
            int(k): v
            for k, v in group.get('task_events', {}).items()
        }
        """The recorded task events"""
        self.safety_violations: Optional['h5py.Dataset'] = group.get(
            'safety_violations')
        """The recorded safety violations"""
        self.seed: int = group.attrs['seed']
        """The seed used during to initialize the world and during the run"""
        self.duration: datetime.timedelta = _timedelta_from_ns(
            group.attrs['duration_ns'])
        """The duration of the run"""
        self.number_of_agents: int = len(self.world.agents)
        """The number of agents recorded"""
        # self.probes_names: List[str] = list(group.keys())
        # """The names of all probes active during recording"""
        self.final_sim_time = group.attrs['final_sim_time']
        self.maximal_steps: int = group.attrs['maximal_steps']
        """The maximal steps that could have been performed"""
        self.recorded_steps: int = group.attrs['steps']
        """The actual number of steps that have been performed"""
        self.time_step: float = group.attrs['time_step']
        """The time step used by the simulation"""
        self._indices: Dict[int, int] = {
            agent._uid: i
            for i, agent in enumerate(self.world.agents)
        }
        """A map between agents' uids and their index in the records"""

    def reset(self) -> None:
        """
        Reset the run to the state at the begin of the recorded run.
        """
        seed = self._group.attrs['seed']
        if 'world' in self._group.attrs:
            world = load_world(self._group.attrs['world'])
            if world:
                self.world = world
                self.world.seed = seed
            else:
                raise RuntimeError("Could not load world")
        elif self._scenario:
            warnings.warn(
                'HDF5 group does not store a world ... sampling from '
                'the scenario may not be correct')
            self.world = World()
            self._scenario.init_world(self.world, seed=seed)
        else:
            raise RuntimeError("World not stored and no scenario")
        self._step = -1

    @property
    def root(self) -> 'h5py.Group':
        """The run root HDF5 group"""
        return self._group

    @property
    def records(self) -> Dict:
        """All recorded datasets"""
        return self.get_records()

    @property
    def record_names(self) -> Set[str]:
        """All recorded dataset names"""

        return self.get_record_names()

    def get_record_names(self, group: str = '') -> Set[str]:
        """
         Gets the names of records.

        :param      group:  An optional group. If specified, the looks for record
                            associated to keys ``<group>/...``.
        :type       group:  str

        :returns:   The record names (relative to the group if specified).
        """
        return set(self.get_records(group=group).keys())

    def get_record(self, key: str = '') -> Optional['h5py.Dataset']:
        """
        Gets recorded data.

        :param group: the name of the record
        :type group: str
        :return: recorded dataset or None if no data
                 has been recorded for the given key
        """
        import h5py

        value = self._group.get(key)
        if isinstance(value, h5py.Dataset):
            return value
        return None

    def get_records(self, group: str = '') -> Dict[str, 'h5py.Dataset']:
        """
        Gets recorded data map.

        :param group: if specified, limits to records in a given group.
        :type group: str
        :return: recorded datasets indexed by their key
        (relative to the group if specified).
        """
        if group:
            g = self._group.get(group)
        else:
            g = self._group
        return _get_all_datasets(g, f'{g.name}/')

    def get_task_events(self, agent: Agent):
        """
        The recorded events logged by the task of an agent
        as a HDF5 dataset of shape
        ``(number events, size of event log)`` and dtype ``float``::

          [[data_0, ...],
           ...]

        The dataset is empty if the agent's task has not been recorded in the run.

        :param agent: The agent
        :return: The events logged by the agent task
        """
        te = self.task_events
        if te:
            return te[agent._uid]
        return np.array([])

    def index_of_agent(self, agent: Agent) -> int:
        """
        Associate an index to a given agent.

        Data related to agents is stored in this order.
        For example, poses at a given time step are stored
        as ``[[x_0, y_0, z_0], [x_1, y_1, z_1], ...]``

        where the pose of agent a is at the index returned by this function.

        :param  agent:  The agent
        :return:    The index of data related to this agent or -1 if not found.
        """
        return self._indices.get(agent._uid, -1)

    def go_to_step(self, step: int) -> bool:
        """
        Try to advance the world to a given recorded simulation step.

        Depending if the data has been recorded, it will update:

        - :py:attr:`navground.sim.World.collisions`,
        - :py:attr:`navground.sim.World.time`
        - :py:attr:`navground.sim.Agent.pose`
        - :py:attr:`navground.sim.Agent.twist`
        - :py:attr:`navground.sim.Agent.last_cmd`

        :return: True if the operation was possible and False otherwise.

        """
        if step >= 0 and step < self.recorded_steps:
            self._step = step
            self.world.step = self._step
            if self.poses:
                for ps, agent in zip(self.poses[self._step],
                                     self.world.agents):
                    agent.pose = core.Pose2(ps[:2], ps[2])
            if self.twists:
                for ps, agent in zip(self.twists[self._step],
                                     self.world.agents):
                    agent.twist = core.Twist2(ps[:2], ps[2])
            if self.commands:
                for ps, agent in zip(self.commands[self._step],
                                     self.world.agents):
                    agent.last_cmd = core.Twist2(ps[:2], ps[2])
            # if self.targets:
            #     for ps, agent in zip(self.targets[self._step],
            #                          self.world.agents):
            #         if agent.behavior:
            #             # TODO(Jerome): we are ignoring that values may be optional
            #             agent.behavior.task.position = ps[:2]
            #             agent.behavior.task.orientation = ps[2]

            if self.times:
                self.world.time = self.times[self._step]
            else:
                self.world.time = self.time_step * (self._step + 1)
            self.world.clear_collisions()
            if self.collisions:
                cs = self.collisions[(self.collisions[:, 0] == self._step)]
                ucs = [(self.world.get_entity(e1), self.world.get_entity(e2))
                       for _, e1, e2 in cs]
                for e1, e2 in ucs:
                    self.world.record_collision(e1, e2)
            return True
        return False

    def do_step(self) -> bool:
        """
        Try to advance the world by one recorded simulation step,
        see :py:meth:`go_to_step`.

        :return: True if the operation was possible and False otherwise.

        """
        return self.go_to_step(self._step + 1)

    @property
    def has_finished(self) -> bool:
        """
        Determines if play back has finished:
        ``True`` if there are no more steps to play back, ``False`` otherwise
        """
        return self._step >= self.recorded_steps - 1

    def __setattr__(self, attr, value):
        if self._frozen and attr not in self._mutable:
            raise AttributeError("RecordedExperiment are frozen")
        return super().__setattr__(attr, value)

    @property
    def bounds(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Computes the rectangle in which agents are contained during the run:
        ``(lower-left corner, top-right corner)``
        or ``None`` if no poses has been recorded
        """
        if not self.poses:
            return None
        a = np.min(self.poses, axis=(0, 1))[:2]
        b = np.max(self.poses, axis=(0, 1))[:2]
        return a, b

    @property
    def bounding_box(self) -> BoundingBox:
        """
        Gets the rectangle that contains the world during the whole run.
        """
        bb = self.world.bounding_box
        if self.poses:
            min_over_steps = np.min(self.poses, axis=(0, ))[:2]
            max_over_steps = np.max(self.poses, axis=(0, ))[:2]
            radii = np.array([a.radius for a in self.world.agents])
            min_over_steps -= radii
            max_over_steps += radii
            min_over_agents = np.min(min_over_steps, axis=0)
            max_over_agents = np.max(max_over_steps, axis=0)
            return BoundingBox(min(bb.min_x, min_over_agents[0]),
                               max(bb.max_x, max_over_agents[0]),
                               min(bb.min_y, min_over_agents[1]),
                               max(bb.max_y, max_over_agents[1]))
        return bb

    def get_collision_events(self, min_interval: int = 0) -> np.ndarray:
        """
        Gets the recorded collisions events, i.e.
        collisions separated by more than min_interval steps

        :param      min_interval:  The minimal interval between collision among
                                   the same pair to be considered a new event

        :returns:   A dataset of shape {`#events`, 4} of events `<begin, end, e1, e2>`
        """
        if self.collisions is None:
            print("Collisions not recorded", file=sys.error)
            return np.array([], dtype=int)
        collision_events: List[Tuple[int, int, int, int]] = []
        # (e1, e2) -> (begin, end)
        ts: Dict[Tuple[int, int], tuple[int, int]] = {}
        for t, *es in self.collisions:
            es = tuple(es)
            if es not in ts:
                ts[es] = (t, t)
            else:
                a, b = ts[es]
                if t > b + min_interval:
                    collision_events.append((a, b, *es))
                    ts[es] = (t, t)
                else:
                    ts[es] = (a, t)
        for es, (a, b) in ts.items():
            collision_events.append((a, b, *es))
        return np.asarray(collision_events)

    def get_steps_to_collision(self, min_interval: int = 0) -> np.ndarray:
        """
        Gets the steps to the next recorded collision
        for each agent at each simulation step.

        :param      min_interval:  The minimal interval between collision among
                                   the same pair to be considered a new event

        :returns:   An array of shape `{#steps, #agents}`
        """
        agents = self.world.agents
        use_uid = self.record_config.use_agent_uid_as_key
        first_agent_id = 0
        if agents and use_uid:
            first_agent_id = agents[0]._uid

        n = len(agents)
        m = np.iinfo(np.uint32).max
        events = self.get_collision_events(min_interval=min_interval)
        vs = np.full((self.recorded_steps, n), m)
        for (a, b, e1, e2) in events:
            for i in range(a, b + 1):
                vs[i, e1 - first_agent_id] = 0
                vs[i, e2 - first_agent_id] = 0

        for i in range(vs.shape[0] - 2, -1, -1):
            for j in range(vs.shape[1]):
                if vs[i, j] > 0 and vs[i + 1, j] < m:
                    vs[i, j] = vs[i + 1, j] + 1

        return vs


class RecordedExperiment:
    """
    A recorded experiment.

    All data is loaded from a HDF5
    recorded by :py:class:`navground.sim.Experiment`,
    and served with the same getter methods and attributes
    as :py:class:`navground.sim.Experiment` but limited to read-only.

    All setters are explicitly blocked.
    """

    _frozen = False

    def __init__(self,
                 path: Union[str, pathlib.Path] = '',
                 file: Optional['h5py.File'] = None):
        """
        Constructs a new instance.

        :param      path:  An optional path to the HDF5 file to load
        :param      file:  An optional HDF5 file. When not specified,
                           the file is loaded from the path.
        """

        if file:
            self._file = file
        else:

            import h5py

            self._file = h5py.File(path)

        self.has_finished = True
        """Set to ``True``: the experiment is no more running"""
        self.is_running = False
        """Set to ``False``: the experiment is no more running"""
        self.path = pathlib.Path(path)
        """The path of the HDF5 file"""

        experiment = load_experiment(self._file.attrs['experiment'])
        if not experiment:
            raise RuntimeError("Could not load world")
        self._experiment = experiment
        self.record_config = self._experiment.record_config
        """The record config used by the experiment"""

        self.runs: Dict[int, RecordedExperimentalRun] = {
            int(re.match(r"run_(\d+)", k).groups()[0]):  # type: ignore
            RecordedExperimentalRun(v, self._experiment.scenario,
                                    record_config=self.record_config)
            for k, v in self._file.items()
        }
        """The recorded runs, indexed by their seed"""

        self.name = self._experiment.name
        """The experiment name"""
        self.number_of_runs = self._experiment.number_of_runs
        """The number of runs that the experiment should have performed"""
        self.run_index = self._experiment.run_index
        """The first seed/index"""
        self.save_directory = self._experiment.save_directory
        """Where the experiment saved its result"""
        self.steps = self._experiment.steps
        """The maximal steps performed by the experiment"""
        self.terminate_when_all_idle_or_stuck = self._experiment.terminate_when_all_idle_or_stuck
        self.scenario = self._experiment.scenario
        """The scenario of the experiment"""
        self.begin_time = datetime.datetime.fromisoformat(
            self._file.attrs['begin_time'])
        """The time stamp of the experiment"""
        self.duration = _timedelta_from_ns(self._file.attrs['duration_ns'])
        """The duration of the experiment"""
        self._frozen = True

    def __setattr__(self, attr, value):
        if self._frozen:
            raise AttributeError("RecordedExperiment are frozen")
        return super().__setattr__(attr, value)
