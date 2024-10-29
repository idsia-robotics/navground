from __future__ import annotations

import importlib.metadata
from typing import (Any, Callable, Dict, List, Tuple, Type, TypeAlias, TypeVar,
                    Union)

import numpy

from ._navground import Action
from ._navground import Behavior as _Behavior
from ._navground import BehaviorModulation as _BehaviorModulation
from ._navground import (Buffer, BufferDescription, BufferMap,
                         CachedCollisionComputation, CollisionComputation,
                         Controller, Disc, EnvironmentState, Frame,
                         GeometricState)
from ._navground import Kinematics as _Kinematics
from ._navground import (LineSegment, Neighbor, Path, Pose2, SensingState,
                         SocialMargin, Target, Twist2, clamp_norm, dump)
from ._navground import get_loaded_plugins as get_loaded_cpp_plugins
from ._navground import (load_behavior, load_behavior_modulation,
                         load_kinematics)
from ._navground import load_plugins as load_cpp_plugins
from ._navground import (normalize_angle, orientation_of, rotate, to_absolute,
                         to_absolute_point, to_relative, to_relative_point,
                         unit, uses_doubles)

# TODO(Jerome): Add vector shape = (2, )
# numpy.ndarray[numpy.float32[2, 1]]

# TODO(Jerome): how to define an alias that depends on `uses_doubles`?
# Vector2: TypeAlias = "numpy.ndarray[numpy.float64, Any]"
Vector2: TypeAlias = numpy.ndarray
PropertyField = Union[bool, int, float, str, Vector2, List[bool], List[int],
                      List[float], List[str], List[Vector2]]

T = TypeVar('T', bound=Any)


def register(default_value: PropertyField,
             description: str = "",
             deprecated_names: list[str] = []) -> Callable[[T], T]:
    """
    A decorator to register a property.
    It must be used below the ``@property`` decorator.

    For example, the following code adds a boolean valued
    registered property to a registered sub-class ``C`` of class ``T``:

    .. code-block:: python

        class C(T, name="C"):

            def __init__(self):
                super().__init__()
                self._my_field = True

            @property
            @register(True, "...")
            def my_property(self) -> bool:
                return self._my_field;

            @my_property.setter
            def my_property(self, value: bool) -> None:
                self._my_field = value;


    :param default_value:
        The default value of the property
        when the object is initialized

    :param description: The description of the property

    :param deprecated_names: A list of alternative deprecated names
    """

    def g(f: T) -> T:
        f.__default_value__ = default_value
        f.__desc__ = description
        f.__deprecated_names__ = deprecated_names
        return f

    return g


def _register(super_cls: Type, cls: Type, name: str):
    if not name:
        return
    super_cls._register_type(name, cls)
    cls._type = name
    for k, v in vars(cls).items():
        if isinstance(v, property) and v.fget and hasattr(
                v.fget, "__default_value__"):
            return_type = v.fget.__annotations__['return']
            if return_type == Vector2:
                return_type = numpy.array
            else:
                try:
                    if return_type._name == "List":
                        return_type = list
                except AttributeError:
                    pass
            default_value = return_type(v.fget.__default_value__)
            desc = v.fget.__desc__  # type: ignore
            deprecated_names = v.fget.__deprecated_names__  # type: ignore
            super_cls._add_property(name, k, v, default_value, desc,
                                    deprecated_names)


class Behavior(_Behavior):

    __doc__ = _Behavior.__doc__

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Behavior, cls, name)

    def __init__(self, kinematics=None, radius=0.0):
        _Behavior.__init__(self, kinematics, radius)

    def __getstate__(self):
        return (self.__dict__, _Behavior.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _Behavior.__setstate__(self, value[1])


class Kinematics(_Kinematics):

    __doc__ = _Kinematics.__doc__

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Kinematics, cls, name)

    def __init__(self, max_speed=0.0, max_angular_speed=0.0):
        _Kinematics.__init__(self, max_speed, max_angular_speed)

    def __getstate__(self):
        return (self.__dict__, _Kinematics.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _Kinematics.__setstate__(self, value[1])


class BehaviorModulation(_BehaviorModulation):

    __doc__ = _BehaviorModulation.__doc__

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_BehaviorModulation, cls, name)

    def __init__(self):
        _BehaviorModulation.__init__(self)

    def __getstate__(self):
        return (self.__dict__, _BehaviorModulation.__getstate__(self))

    def __setstate__(self, value):
        self.__dict__ = value[0]
        _BehaviorModulation.__setstate__(self, value[1])


from . import behavior_modulations, behaviors, kinematics


def load_py_plugins() -> None:
    for group in ('navground_behaviors', 'navground_kinematics',
                  'navground_modulations'):
        for entry_point in importlib.metadata.entry_points(group=group):
            entry_point.load()


def load_plugins() -> None:
    """
    Loads registered Python and C++ plugins.

    Python plugins extend :py:class:`Behavior`
    and :py:class:`Kinematics` registers
    """
    load_cpp_plugins()
    load_py_plugins()


def get_loaded_py_plugins(
    kinds: List[str] = ['behaviors', 'kinematics', 'modulations']
) -> Dict[str, Dict[str, List[str]]]:
    """
    Returns all plugins implemented in Python

    :param      kinds:  The kinds of components

    :returns:   A dictionary {pkg name: {kind: [registered types]}}
    """
    rs: Dict[str, Dict[str, List[str]]] = {}
    for kind in kinds:
        eps = importlib.metadata.entry_points(group=f'navground_{kind}')
        for e in eps:
            pkg = e.module.split('.')[0]
            if pkg not in rs:
                rs[pkg] = {k: [] for k in kinds}
            cls = e.load()
            try:
                rs[pkg][kind].append(cls._type)
            except AttributeError:
                print(f'{cls} has not been registered')
    return rs


def get_loaded_plugins(
    kinds: List[str] = ['behaviors', 'kinematics', 'modulations']
) -> Dict[str, Dict[str, List[Tuple[str, str]]]]:
    """
    Returns all plugins

    :param      kinds:  The kinds of components

    :returns:   A dictionary {pkg name: {kind: [(registered type, language)]}}
    """
    py_rs = get_loaded_py_plugins(kinds)
    cpp_rs = get_loaded_cpp_plugins()
    rs = {
        pkg: {
            kind: [(name, 'Python') for name in names]
            for kind, names in vs.items()
        }
        for pkg, vs in py_rs.items()
    }
    for pkg, vs in cpp_rs.items():
        if pkg not in rs:
            rs[pkg] = {
                kind: [(name, 'C++') for name in names]
                for kind, names in vs.items() if kind in kinds
            }
        else:
            for kind, names in vs.items():
                if kind in kinds:
                    rs[pkg][kind].extend([(name, 'C++') for name in names])
    return rs


__all__ = [
    'Behavior', 'Path', 'BehaviorModulation', 'Pose2', 'Twist2', 'Target',
    'Disc', 'Neighbor', 'LineSegment', 'Kinematics', 'Action', 'Controller',
    'CollisionComputation', 'CachedCollisionComputation', 'Frame',
    'GeometricState', 'SensingState', 'dump', 'load_behavior',
    'load_behavior_modulation', 'load_kinematics', 'load_plugins', 'Buffer',
    'BufferMap', 'BufferDescription', 'SocialMargin',
    'CachedCollisionComputation', 'EnvironmentState', 'CollisionComputation',
    'behaviors', 'behavior_modulations', 'kinematics', 'clamp_norm', 'rotate',
    'unit', 'orientation_of', 'normalize_angle', 'to_absolute_point',
    'to_relative_point', 'to_absolute', 'to_relative', 'uses_doubles',
    'get_loaded_plugins'
]
