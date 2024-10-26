import functools
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    import tqdm

import navground.core
import importlib.metadata
from navground.core import _register, load_cpp_plugins
from navground.core import load_py_plugins as _load_py_core_plugins
from navground.core import get_loaded_plugins as _get_loaded_core_plugins
from navground.core import get_loaded_py_plugins as _get_loaded_py_core_plugins
from navground.core import register

from ._navground_sim import (Agent, BoundingBox, Dataset, Entity, Experiment,
                             ExperimentalRun, GroupRecordProbe, Obstacle,
                             Probe, RecordConfig, RecordProbe,
                             RecordSensingConfig)
from ._navground_sim import Scenario as _Scenario
from ._navground_sim import SensingProbe
from ._navground_sim import Sensor as _Sensor
from ._navground_sim import StateEstimation as _StateEstimation
from ._navground_sim import Task as _Task
from ._navground_sim import (Wall, World, dump, load_agent, load_experiment,
                             load_scenario, load_state_estimation, load_task,
                             load_world, use_compact_samplers, uses_doubles)
from .recorded_experiment import RecordedExperiment, RecordedExperimentalRun
from .run_mp import run_mp


class Scenario(_Scenario):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Scenario, cls, name)

    def __init__(self):
        _Scenario.__init__(self)

    def __getstate__(self):
        return (self.__dict__, _Scenario.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _Scenario.__setstate__(self, value[1])


class StateEstimation(_StateEstimation):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_StateEstimation, cls, name)

    def __init__(self, *args, **kwargs):
        _StateEstimation.__init__(self, *args, **kwargs)

    def __getstate__(self):
        return (self.__dict__, _StateEstimation.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _StateEstimation.__setstate__(self, value[1])


class Sensor(_Sensor):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_StateEstimation, cls, name)

    def __init__(self, *args, **kwargs):
        _Sensor.__init__(self, *args, **kwargs)

    def __getstate__(self):
        return (self.__dict__, _Sensor.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _Sensor.__setstate__(self, value[1])


class Task(_Task):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Task, cls, name)

    def __init__(self):
        _Task.__init__(self)

    def __getstate__(self):
        return (self.__dict__, _Task.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _Task.__setstate__(self, value[1])


from . import scenarios, state_estimations, tasks

TaskCallback = Callable[[List[float]], None]


def load_py_plugins():
    _load_py_core_plugins()
    for group in ('navground_tasks', 'navground_state_estimations',
                  'navground_scenarios'):
        for entry_point in importlib.metadata.entry_points(group=group):
            entry_point.load()


def load_plugins() -> None:
    """
    Loads registered Python and C++ plugins.

    Python plugins extend
    :py:class:`navground.core.Behavior`,
    :py:class:`navground.core.Kinematics`,
    :py:class:`Scenario`,
    :py:class:`StateEstimation`, and
    :py:class:`Task` registers
    """
    load_cpp_plugins()
    load_py_plugins()


def get_loaded_py_plugins(
    kinds: List[str] = [
        'behaviors', 'kinematics', 'modulations', 'state_estimations', 'tasks',
        'scenarios'
    ]
) -> Dict[str, Dict[str, List[str]]]:
    """
    Returns all plugins implemented in Python

    :param      kinds:  The kinds of components

    :returns:   A dictionary {pkg name: {kind: [registered types]}}
    """
    return _get_loaded_py_core_plugins(kinds)


def get_loaded_plugins(
    kinds: List[str] = [
        'behaviors', 'kinematics', 'modulations', 'state_estimations', 'tasks',
        'scenarios'
    ]
) -> Dict[str, Dict[str, List[Tuple[str, str]]]]:
    """
    Returns all plugins

    :param      kinds:  The kinds of components

    :returns:   A dictionary {pkg name: {kind: [(registered type, language)]}}
    """
    return _get_loaded_core_plugins(kinds)


def setup_tqdm(self,
               bar: 'tqdm.tqdm',
               number_of_runs: Optional[int] = None) -> None:
    """
        Configure a tqdm object that displays the progress of an experiment

        :param bar: a tqdm progress bar

        :param number_of_runs: the number of runs to track. If not provided,
                               it will use :py:attr:`sim.Experiment.number_of_runs`.

    """

    bar.total = number_of_runs if number_of_runs is not None else self.number_of_runs
    self.add_run_callback(lambda _: bar.update(1))


Experiment.setup_tqdm = setup_tqdm
Experiment.run_mp = run_mp


@functools.singledispatch
def uses_python(item) -> bool:
    """
    Check whether this item uses any Python-class
    as behavior/kinematics/state estimation/task.

    :param      item:  The item
    """
    return False


@uses_python.register
def _(agent: Agent) -> bool:
    return any((isinstance(agent.behavior, navground.core.Behavior),
                isinstance(agent.kinematics, navground.core.Kinematics),
                isinstance(agent.state_estimation,
                           StateEstimation), isinstance(agent.task, Task)))


@uses_python.register
def _(world: World) -> bool:
    return any(uses_python(agent) for agent in world.agents)


@uses_python.register
def _(scenario: _Scenario) -> bool:
    world = World()
    scenario.init_world(world)
    return uses_python(world)


@uses_python.register
def _(experiment: Experiment) -> bool:
    return uses_python(experiment.scenario)


__all__ = [
    'Entity', 'Obstacle', 'Wall', 'World', 'Agent', 'Experiment', 'Scenario',
    'StateEstimation', 'Task', 'BoundingBox', 'dump', 'TaskCallback',
    'load_agent', 'load_state_estimation', 'load_task', 'load_world',
    'load_scenario', 'load_experiment', 'load_plugins', 'register', 'Sensor',
    'ExperimentalRun', 'RecordSensingConfig', 'RecordConfig', 'RecordProbe',
    'GroupRecordProbe', 'Probe', 'Dataset', 'RecordedExperiment',
    'RecordedExperimentalRun', 'SensingProbe', 'use_compact_samplers',
    'uses_doubles', 'get_loaded_plugins'
]
