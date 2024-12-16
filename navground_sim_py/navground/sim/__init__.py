import functools
from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING, Any, TypeAlias

if TYPE_CHECKING:
    import tqdm

import importlib.metadata

import navground.core
from navground.core import Vector2
from navground.core import get_loaded_plugins as _get_loaded_core_plugins
from navground.core import get_loaded_py_plugins as _get_loaded_py_core_plugins
from navground.core import get_plugins_dependencies, load_cpp_plugins
from navground.core import load_py_plugins as _load_py_core_plugins
from navground.core import register

from . import schema
from ._navground_sim import (
    Agent, BoundingBox, Dataset, Entity, Experiment, ExperimentalRun,
    GroupRecordProbe, Obstacle, Probe, RecordConfig, RecordNeighborsConfig,
    RecordProbe, RecordSensingConfig, RunConfig, Scenario, SensingProbe,
    Sensor, StateEstimation, Task, Wall, World, get_build_dependencies,
    get_build_info, use_compact_samplers, uses_doubles)

# isort: split

from . import scenarios, state_estimations, tasks
from .run_mp import run_mp

Bounds = tuple[Vector2, Vector2]


def bounds_of_bounding_box(bb: BoundingBox) -> Bounds:
    return bb.p1, bb.p2


def bounds_for_world(world: World) -> Bounds:
    return bounds_of_bounding_box(world.bounding_box)


SUPPORT_YAML: TypeAlias = (navground.core.SUPPORT_YAML | Task | StateEstimation
                           | Scenario | Experiment | Agent | World | Wall
                           | Obstacle)


def load_state_estimation(value: str) -> StateEstimation | None:
    return StateEstimation.load(value)


load_state_estimation.__doc__ = StateEstimation.load.__doc__


def load_task(value: str) -> Task | None:
    return Task.load(value)


load_task.__doc__ = Task.load.__doc__


def load_scenario(value: str) -> Scenario | None:
    return Scenario.load(value)


load_scenario.__doc__ = Scenario.load.__doc__


def load_obstacle(value: str) -> Obstacle | None:
    return Obstacle.load(value)


load_obstacle.__doc__ = Obstacle.load.__doc__


def load_wall(value: str) -> Wall | None:
    return Wall.load(value)


load_wall.__doc__ = Wall.load.__doc__


def load_agent(value: str) -> Agent | None:
    return Agent.load(value)


load_agent.__doc__ = Agent.load.__doc__


def load_world(value: str) -> World | None:
    return World.load(value)


load_world.__doc__ = World.load.__doc__


def load_experiment(value: str) -> Experiment | None:
    return Experiment.load(value)


load_experiment.__doc__ = Experiment.load.__doc__


def dump(obj: SUPPORT_YAML) -> str:
    """
    Dumps the object to a YAML-string.

    :return: The YAML representation
    :rtype: str
    """
    return obj.dump()


TaskCallback = Callable[[list[float]], None]


def load_py_plugins() -> None:
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


def get_loaded_py_plugins(kinds: Iterable[str] = (
    'behaviors', 'kinematics', 'modulations', 'state_estimations', 'tasks',
    'scenarios')) -> dict[str, dict[str, list[str]]]:
    """
    Returns all plugins implemented in Python

    :param      kinds:  The kinds of components

    :returns:   A dictionary {pkg name: {kind: [registered types]}}
    """
    return _get_loaded_py_core_plugins(kinds)


def get_loaded_plugins(kinds: Iterable[str] = (
    'behaviors', 'kinematics', 'modulations', 'state_estimations', 'tasks',
    'scenarios')) -> dict[str, dict[str, list[tuple[str, str]]]]:
    """
    Returns all plugins

    :param      kinds:  The kinds of components

    :returns:   A dictionary {pkg name: {kind: [(registered type, language)]}}
    """
    return _get_loaded_core_plugins(kinds)


@functools.singledispatch
def uses_python(item: Any) -> bool:
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


def setup_tqdm(self: Experiment,
               bar: 'tqdm.tqdm[Any]',
               number_of_runs: int | None = None) -> None:
    """
        Configure a tqdm object that displays the progress of an experiment

        :param bar: a tqdm progress bar

        :param number_of_runs: the number of runs to track. If not provided,
                               it will use :py:attr:`sim.Experiment.number_of_runs`.

    """

    bar.total = number_of_runs if number_of_runs is not None else self.number_of_runs

    def bar_update(run: ExperimentalRun) -> None:
        bar.update(1)

    self.add_run_callback(bar_update)


Experiment.setup_tqdm = setup_tqdm  # type: ignore[method-assign]
Experiment.run_mp = run_mp  # type: ignore[method-assign]

# isort: stop
from .recorded_experiment import (  # noqa: E402
    RecordedExperiment, RecordedExperimentalRun)

__all__ = [
    'Entity', 'Obstacle', 'Wall', 'World', 'Agent', 'Experiment', 'Scenario',
    'StateEstimation', 'Task', 'BoundingBox', 'dump', 'TaskCallback',
    'load_agent', 'load_state_estimation', 'load_task', 'load_world',
    'load_scenario', 'load_experiment', 'load_plugins', 'register', 'Sensor',
    'ExperimentalRun', 'RecordNeighborsConfig', 'RecordSensingConfig',
    'RecordConfig', 'RecordProbe', 'GroupRecordProbe', 'Probe', 'Dataset',
    'RecordedExperiment', 'RecordedExperimentalRun', 'SensingProbe',
    'use_compact_samplers', 'uses_doubles', 'get_loaded_plugins', 'schema',
    'get_build_info', 'get_build_dependencies', 'get_plugins_dependencies',
    'scenarios', 'state_estimations', 'tasks', 'RunConfig'
]
