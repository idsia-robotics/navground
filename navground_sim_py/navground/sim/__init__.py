from __future__ import annotations

import os

if hasattr(os, "add_dll_directory") and "NAVGROUND_DLL_PATH" in os.environ:
    os.add_dll_directory(os.environ["NAVGROUND_DLL_PATH"])

import functools
from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING, Annotated, Any, TypeAlias

if TYPE_CHECKING:
    import tqdm

import importlib.metadata

import navground.core
import numpy
import numpy.typing
from navground.core import _copy_doc
from navground.core import get_loaded_plugins as _get_loaded_core_plugins
from navground.core import get_loaded_py_plugins as _get_loaded_py_core_plugins
from navground.core import get_plugins_dependencies, load_cpp_plugins
from navground.core import load_py_plugins as _load_py_core_plugins
from navground.core import register

from . import schema
from ._navground_sim import (Agent, AgentSampler, BoundingBox, Dataset, Entity,
                             Experiment, ExperimentalRun, GroupRecordProbe,
                             Obstacle, Probe, RecordConfig,
                             RecordNeighborsConfig, RecordProbe,
                             RecordSensingConfig, RunConfig, Sampler, Scenario,
                             SensingProbe, Sensor, StateEstimation, Task, Wall,
                             World, get_build_dependencies, get_build_info,
                             use_compact_samplers, uses_doubles)
from .run_mp import ScenarioInitCallback, run_mp
from .ui.to_svg import svg_for_world

# isort: split

SUPPORT_YAML: TypeAlias = (navground.core.SUPPORT_YAML | Task | StateEstimation
                           | Scenario | Experiment | Agent | World | Wall
                           | Obstacle | Sampler)
TaskCallback: TypeAlias = Callable[[list[float]], None]
Bounds: TypeAlias = tuple[navground.core.Vector2, navground.core.Vector2]

from . import scenarios, state_estimations, tasks


def load_state_estimation(value: str) -> StateEstimation | None:
    return StateEstimation.load(value)


_copy_doc(load_state_estimation, StateEstimation)


def load_sensor(value: str) -> Sensor | None:
    return Sensor.load(value)


_copy_doc(load_state_estimation, StateEstimation)


def load_task(value: str) -> Task | None:
    return Task.load(value)


_copy_doc(load_task, Task)


def load_scenario(value: str) -> Scenario | None:
    return Scenario.load(value)


_copy_doc(load_scenario, Scenario)


def load_obstacle(value: str) -> Obstacle | None:
    return Obstacle.load(value)


_copy_doc(load_obstacle, Obstacle)


def load_wall(value: str) -> Wall | None:
    return Wall.load(value)


_copy_doc(load_wall, Wall)


def load_agent(value: str) -> Agent | None:
    return Agent.load(value)


_copy_doc(load_agent, Agent)


def load_world(value: str) -> World | None:
    return World.load(value)


_copy_doc(load_world, World)


def load_experiment(value: str) -> Experiment | None:
    return Experiment.load(value)


_copy_doc(load_experiment, Experiment)


def load_sampler(value: str, type_name: str) -> Sampler | None:
    return Sampler.load(value, type_name)


_copy_doc(load_sampler, Sampler)


def load_group(value: str) -> Scenario.Group | None:
    return AgentSampler.load(value)


_copy_doc(load_group, Scenario.Group)


def dump(obj: SUPPORT_YAML) -> str:
    """
    Dumps the object to a YAML-string.

    :return: The YAML representation
    :rtype: str
    """
    return obj.dump()


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
               bar: tqdm.tqdm[Any],
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


def repr_svg(world: World) -> str:
    return svg_for_world(world, **world.render_kwargs)


World.render_kwargs = {}
"""World-specific rendering configuration. Specified fields override
:py:data:`navground.sim.ui.render_default_config`"""
World._repr_svg_ = repr_svg

# isort: stop
from .recorded_experiment import RecordedExperiment  # noqa: E402
from .recorded_experiment import RecordedExperimentalRun

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
    'scenarios', 'state_estimations', 'tasks', 'RunConfig', 'AgentSampler',
    'Sampler', 'load_sampler', 'Bounds', 'ScenarioInitCallback'
]
