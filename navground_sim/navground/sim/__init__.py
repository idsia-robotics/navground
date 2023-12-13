from typing import Callable, List

import pkg_resources
from navground.core import _register
from navground.core import load_py_plugins as _load_py_plugins
from navground.core import registered_property

from ._navground_sim import Agent, BoundingBox, Entity, Experiment, Obstacle
from ._navground_sim import Scenario as _Scenario
from ._navground_sim import Sensor
from ._navground_sim import StateEstimation as _StateEstimation
from ._navground_sim import Task as _Task
from ._navground_sim import (Trace, Wall, World, dump, load_agent,
                             load_experiment, load_scenario,
                             load_state_estimation, load_task, load_world)


class Scenario(_Scenario):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Scenario, cls, name)

    def __init__(self):
        _Scenario.__init__(self)


class StateEstimation(_StateEstimation):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_StateEstimation, cls, name)

    def __init__(self, *args, **kwargs):
        _StateEstimation.__init__(self, *args, **kwargs)


class Task(_Task):

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Task, cls, name)

    def __init__(self):
        _Task.__init__(self)


from . import scenarios, state_estimations, tasks

TaskCallback = Callable[[List[float]], None]


def load_py_plugins():
    _load_py_plugins()
    for name in ('navground_tasks', 'navground_state_estimations',
                 'navground_scenarios'):
        for entry_point in pkg_resources.iter_entry_points(name):
            entry_point.load()


def _experiment_tqdm(self):
    """
        Return a tqdm object that displays experiment progresses
    """
    from tqdm.notebook import tqdm as tqdm_n

    bar = tqdm_n(total=self.runs)
    self.add_run_callback(lambda: bar.update(1))
    return bar


Experiment.tqdm = _experiment_tqdm

__all__ = [
    'Entity', 'Obstacle', 'Wall', 'World', 'Agent', 'Experiment', 'Scenario',
    'StateEstimation', 'Task', 'BoundingBox', 'dump', 'TaskCallback',
    'load_agent', 'load_state_estimation', 'load_task', 'load_world',
    'load_scenario', 'load_experiment', 'load_py_plugins',
    'registered_property', 'Sensor', 'Trace'
]
