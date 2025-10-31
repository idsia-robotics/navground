from __future__ import annotations

import os

if hasattr(os, "add_dll_directory") and "NAVGROUND_DLL_PATH" in os.environ:
    os.add_dll_directory(os.environ["NAVGROUND_DLL_PATH"])

import importlib.metadata
import pathlib as pl
from collections.abc import Callable, Iterable
from typing import Annotated, Any, SupportsFloat, TypeAlias, cast

import numpy
import numpy.typing

from . import schema
from ._navground import (
    Action, Behavior, BehaviorGroup, BehaviorGroupMember, BehaviorModulation,
    BehaviorModulationRegister, BehaviorRegister, Buffer, BufferDescription,
    BufferMap, BuildInfo, CachedCollisionComputation, CollisionComputation,
    Controller, DependencyInfo, Disc, EnvironmentState, Frame, GeometricState,
    GridMap, HasAttributes, HasProperties, Kinematics, KinematicsRegister,
    LineSegment, Neighbor, Path, Pose2, Property, SensingState, SocialMargin,
    SocialMarginConstantModulation, SocialMarginLinearModulation,
    SocialMarginLogisticModulation, SocialMarginModulation,
    SocialMarginQuadraticModulation, SocialMarginZeroModulation, Target,
    Twist2, clamp_norm, convert, get_build_dependencies, get_build_info)
from ._navground import get_loaded_plugins as get_loaded_cpp_plugins
from ._navground import (get_plugins_dependencies, get_scalar_type_name,
                         get_type_name_with_scalar)
from ._navground import load_plugins as load_cpp_plugins
from ._navground import (normalize_angle, orientation_of, rotate, to_absolute,
                         to_absolute_point, to_relative, to_relative_point,
                         unit, uses_doubles)
from .types import FloatType
from .property import PropertyField, PropertyFieldLike, register

Vector2: TypeAlias = Annotated[numpy.typing.NDArray[FloatType], '[2, 1]']
Vector2Like: TypeAlias = Annotated[numpy.typing.ArrayLike, FloatType, '[2, 1]']


def zeros2() -> Vector2:
    """Creates a zero vector of size 2.

       :returns: A :py:type:`Vector2` with fields initialized to zero.
    """
    return cast(Vector2, numpy.zeros(2, dtype=FloatType))


Attribute: TypeAlias = bool | int | float | str | Vector2 | list[bool] | list[
    int] | list[float] | list[str] | list[Vector2]
BuildDependencies: TypeAlias = dict[str, DependencyInfo]
PkgDependencies: TypeAlias = dict[str, dict[pl.Path, BuildDependencies]]
Curve: TypeAlias = Callable[[float], tuple[Vector2, SupportsFloat,
                                           SupportsFloat]]
Projection: TypeAlias = Callable[[Vector2Like, SupportsFloat, SupportsFloat],
                                 float]
Cell: TypeAlias = Annotated[numpy.typing.NDArray[numpy.int32], '[2, 1]']
CellLike: TypeAlias = Annotated[numpy.typing.ArrayLike, numpy.int32, '[2, 1]']
Map: TypeAlias = Annotated[numpy.typing.NDArray[numpy.uint8], '[m, n]']

# isort: split

from . import behavior_modulations, behaviors, kinematics


def _copy_doc(fn: Any, cls: Any) -> None:
    if cls.load.__doc__:
        fn.__doc__ = '\n'.join(cls.load.__doc__.split('\n')[2:])


def load_behavior(value: str) -> Behavior | None:
    return Behavior.load(value)


_copy_doc(load_behavior, Behavior)


def load_behavior_modulation(value: str) -> BehaviorModulation | None:
    return BehaviorModulation.load(value)


_copy_doc(load_behavior_modulation, BehaviorModulation)


def load_kinematics(value: str) -> Kinematics | None:
    return Kinematics.load(value)


_copy_doc(load_kinematics, Kinematics)


def load_disc(value: str) -> Disc | None:
    return Disc.load(value)


_copy_doc(load_disc, Disc)


def load_line_segment(value: str) -> LineSegment | None:
    return LineSegment.load(value)


_copy_doc(load_line_segment, LineSegment)


def load_neighbor(value: str) -> Neighbor | None:
    return Neighbor.load(value)


_copy_doc(load_neighbor, Neighbor)

SUPPORT_YAML: TypeAlias = Behavior | BehaviorModulation | Kinematics | Disc | LineSegment | Neighbor


def dump(obj: SUPPORT_YAML) -> str:
    """
    Dumps the object to a YAML-string.

    :return: The YAML representation
    :rtype: str
    """
    return obj.dump()


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


def get_loaded_py_plugins(kinds: Iterable[str] = (
    'behaviors', 'kinematics',
    'modulations')) -> dict[str, dict[str, list[str]]]:
    """
    Returns all plugins implemented in Python

    :param      kinds:  The kinds of components

    :returns:   A dictionary ``{pkg name: {kind: [registered types]}}``
    """
    rs: dict[str, dict[str, list[str]]] = {}
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


def get_loaded_plugins(kinds: Iterable[str] = (
    'behaviors', 'kinematics',
    'modulations')) -> dict[str, dict[str, list[tuple[str, str]]]]:
    """
    Returns all plugins

    :param      kinds:  The kinds of components

    :returns:   A dictionary ``{pkg name: {kind: [(registered type, language)]}}``
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
    'get_loaded_plugins', 'schema', "Property",
    "SocialMarginConstantModulation", "SocialMarginLinearModulation",
    "SocialMarginLogisticModulation", "SocialMarginModulation",
    "SocialMarginQuadraticModulation", "SocialMarginZeroModulation",
    "PropertyField", "PropertyFieldLike", "Vector2", "Vector2Like", "register",
    "get_build_info", "BuildInfo", "get_build_dependencies",
    "get_plugins_dependencies", "DependencyInfo", "load_cpp_plugins",
    "BehaviorModulationRegister", "BehaviorRegister", "HasProperties",
    "KinematicsRegister", "FloatType", "GridMap", "HasAttributes",
    "BehaviorGroup", "BehaviorGroupMember", "convert", "get_scalar_type_name",
    "get_type_name_with_scalar", "zeros2"
]
