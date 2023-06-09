from __future__ import annotations

from typing import Any, Callable, List, ParamSpec, TypeVar, Union

import numpy
import pkg_resources

from ._navground import Action
from ._navground import Behavior as _Behavior
from ._navground import (CachedCollisionComputation, CollisionComputation,
                             Controller, Disc, Frame, GeometricState)
from ._navground import Kinematics as _Kinematics
from ._navground import (EnvironmentState, LineSegment, Neighbor, Pose2, SocialMargin,
                             Target, Twist2, dump, load_behavior,
                             load_kinematics)

Vector2 = 'numpy.ndarray[numpy.float32[2, 1]]'
PropertyField = Union[bool, int, float, str, Vector2, List[bool], List[int],
                      List[float], List[str], List[Vector2]]

T = TypeVar('T')
P = Callable[[], T]


def registered_property(default_value: PropertyField,
                        description: str = "") -> Callable[[P], P]:
    """
    A decorator that define and automatically register a property.
    The use it similar to :py:func:`property`, except for the
    two additional arguments

    For example, the following code adds a boolean valued auto
    registered property to a registered sub-class ``C`` of class ``T``:

    .. code-block:: python

        class C(T, name="C"):

            def __init__(self):
                super().__init__()
                self._my_field = True

            @registered_property(True, "...")
            def my_property(self) -> bool:
                return self._my_field;

            @my_property.setter
            def my_property(self, value: bool) -> None:
                self._my_field = value;


    :param default_value:
        The default value of the property
        when the object is initialized

    :param description: The description of the property
    """

    def g(f):
        f.__default_value__ = default_value
        f.__desc__ = description
        return property(f)

    return g


class Behavior(_Behavior):

    __doc__ = _Behavior.__doc__

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _Behavior._register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _Behavior._add_property(name, k, v, default_value, desc)

    def __init__(self, kinematics=None, radius=0.0):
        _Behavior.__init__(self, kinematics, radius)


class Kinematics(_Kinematics):

    __doc__ = _Kinematics.__doc__

    def __init_subclass__(cls, /, **kwargs):
        name = kwargs.pop('name', '')
        super().__init_subclass__(**kwargs)
        if name:
            _Kinematics._register_type(name, cls)
            cls._type = name
            for k, v in vars(cls).items():
                if isinstance(v, property) and hasattr(v.fget,
                                                       "__default_value__"):
                    return_type = v.fget.__annotations__['return']
                    default_value = return_type(v.fget.__default_value__)
                    desc = v.fget.__desc__
                    _Kinematics._add_property(name, k, v, default_value, desc)

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
    'CachedCollisionComputation', 'Frame', 'GeometricState', 'dump',
    'load_behavior', 'load_kinematics', 'load_py_plugins'
]
