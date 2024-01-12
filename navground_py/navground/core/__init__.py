from __future__ import annotations

from typing import Any, Callable, List, Type, TypeAlias, TypeVar, Union

import numpy
import pkg_resources

from ._navground import Action
from ._navground import Behavior as _Behavior
from ._navground import (Buffer, BufferDescription, BufferMap,
                         CachedCollisionComputation, CollisionComputation,
                         Controller, Disc, EnvironmentState, Frame,
                         GeometricState)
from ._navground import Kinematics as _Kinematics
from ._navground import (LineSegment, Neighbor, Pose2, SensingState,
                         SocialMargin, Target, Twist2, dump, load_behavior,
                         load_kinematics, to_absolute, to_relative)

# TODO(Jerome): Add vector shape = (2, )
# numpy.ndarray[numpy.float32[2, 1]]
Vector2: TypeAlias = "numpy.ndarray[numpy.float32, Any]"
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
        if isinstance(v, property) and v.fget and hasattr(v.fget, "__default_value__"):
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
            deprecated_names = v.fget.__deprecated_names__ # type: ignore
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


class Kinematics(_Kinematics):

    __doc__ = _Kinematics.__doc__

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        _register(_Kinematics, cls, name)

    def __init__(self, max_speed=0.0, max_angular_speeed=0.0):
        _Kinematics.__init__(self, max_speed, max_angular_speeed)


from . import behaviors, kinematics


def load_py_plugins():
    for name in ('navground_behaviors', 'navground_kinematics'):
        for entry_point in pkg_resources.iter_entry_points(name):
            entry_point.load()


__all__ = [
    'Behavior', 'Pose2', 'Twist2', 'Target', 'Disc', 'Neighbor', 'LineSegment',
    'Kinematics', 'Action', 'Controller', 'CollisionComputation'
    'CachedCollisionComputation', 'Frame', 'GeometricState', 'SensingState',
    'dump', 'load_behavior', 'load_kinematics', 'load_py_plugins',
    'to_absolute', 'to_relative', 'Buffer', 'BufferMap', 'BufferDescription'
]
