import functools
from typing import (TYPE_CHECKING, Callable, Dict, List, Optional, Tuple,
                    TypeAlias, Union)

if TYPE_CHECKING:
    import tqdm

import importlib.metadata

import navground.core
from navground.core import get_loaded_plugins as _get_loaded_core_plugins
from navground.core import get_loaded_py_plugins as _get_loaded_py_core_plugins
from navground.core import get_plugins_dependencies, load_cpp_plugins
from navground.core import load_py_plugins as _load_py_core_plugins
from navground.core import register

from . import schema
from ._navground_sim import (Agent, BoundingBox, Dataset, Entity, Experiment,
                             ExperimentalRun, GroupRecordProbe, Obstacle,
                             Probe, RecordConfig, RecordNeighborsConfig,
                             RecordProbe, RecordSensingConfig, Scenario,
                             SensingProbe, Sensor, StateEstimation, Task, Wall,
                             World, get_build_dependencies, get_build_info,
                             use_compact_samplers, uses_doubles)

SUPPORT_YAML: TypeAlias = Union[navground.core.SUPPORT_YAML, Task,
                                StateEstimation, Scenario, Experiment, Agent,
                                World, Wall, Obstacle]


def load_state_estimation(value: str) -> Optional[StateEstimation]:
    return StateEstimation.load(value)


load_state_estimation.__doc__ = StateEstimation.load.__doc__


def load_task(value: str) -> Optional[Task]:
    return Task.load(value)


load_task.__doc__ = Task.load.__doc__


def load_scenario(value: str) -> Optional[Scenario]:
    return Scenario.load(value)


load_scenario.__doc__ = Scenario.load.__doc__


def load_obstacle(value: str) -> Optional[Obstacle]:
    return Obstacle.load(value)


load_obstacle.__doc__ = Obstacle.load.__doc__


def load_wall(value: str) -> Optional[Wall]:
    return Wall.load(value)


load_wall.__doc__ = Wall.load.__doc__


def load_agent(value: str) -> Optional[Agent]:
    return Agent.load(value)


load_agent.__doc__ = Agent.load.__doc__


def load_world(value: str) -> Optional[World]:
    return World.load(value)


load_world.__doc__ = World.load.__doc__


def load_experiment(value: str) -> Optional[Experiment]:
    return Experiment.load(value)


load_experiment.__doc__ = Experiment.load.__doc__


def dump(obj: SUPPORT_YAML) -> str:
    """
    Dumps the object to a YAML-string.

    :return: The YAML representation
    :rtype: str
    """
    return obj.dump()


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


from .recorded_experiment import RecordedExperiment, RecordedExperimentalRun
from .run_mp import run_mp

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
def _(scenario: Scenario) -> bool:
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
    'ExperimentalRun', 'RecordNeighborsConfig', 'RecordSensingConfig',
    'RecordConfig', 'RecordProbe', 'GroupRecordProbe', 'Probe', 'Dataset',
    'RecordedExperiment', 'RecordedExperimentalRun', 'SensingProbe',
    'use_compact_samplers', 'uses_doubles', 'get_loaded_plugins', 'schema',
    'get_build_info', 'get_build_dependencies', 'get_plugins_dependencies'
]
