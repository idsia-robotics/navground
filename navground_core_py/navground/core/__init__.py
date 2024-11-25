from __future__ import annotations

import importlib.metadata
import pathlib as pl
from typing import Dict, List, Optional, Tuple, TypeAlias, Union

from . import schema
from ._navground import (
    Action, Behavior, BehaviorModulation, BehaviorModulationRegister,
    BehaviorRegister, Buffer, BufferDescription, BufferMap, BuildInfo,
    CachedCollisionComputation, CollisionComputation, Controller,
    DependencyInfo, Disc, EnvironmentState, Frame, GeometricState,
    HasProperties, Kinematics, KinematicsRegister, LineSegment, Neighbor, Path,
    Pose2, Property, SensingState, SocialMargin,
    SocialMarginConstantModulation, SocialMarginLinearModulation,
    SocialMarginLogisticModulation, SocialMarginModulation,
    SocialMarginQuadraticModulation, SocialMarginZeroModulation, Target,
    Twist2, clamp_norm, get_build_dependencies, get_build_info)
from ._navground import get_loaded_plugins as get_loaded_cpp_plugins
from ._navground import get_plugins_dependencies
from ._navground import load_plugins as load_cpp_plugins
from ._navground import (normalize_angle, orientation_of, rotate, to_absolute,
                         to_absolute_point, to_relative, to_relative_point,
                         unit, uses_doubles)
from .property import PropertyField, Vector2, Vector2Like, register

BuildDependencies: TypeAlias = Dict[str, DependencyInfo]
PkgDependencies: TypeAlias = Dict[str, Dict[pl.Path, BuildDependencies]]


def load_behavior(value: str) -> Optional[Behavior]:
    return Behavior.load(value)


load_behavior.__doc__ = Behavior.load.__doc__


def load_behavior_modulation(value: str) -> Optional[BehaviorModulation]:
    return BehaviorModulation.load(value)


load_behavior_modulation.__doc__ = BehaviorModulation.load.__doc__


def load_kinematics(value: str) -> Optional[Kinematics]:
    return Kinematics.load(value)


load_kinematics.__doc__ = Kinematics.load.__doc__


def load_disc(value: str) -> Optional[Disc]:
    return Disc.load(value)


load_disc.__doc__ = Disc.load.__doc__


def load_line_segment(value: str) -> Optional[LineSegment]:
    return LineSegment.load(value)


load_line_segment.__doc__ = LineSegment.load.__doc__


def load_neighbor(value: str) -> Optional[Neighbor]:
    return Neighbor.load(value)


load_neighbor.__doc__ = Neighbor.load.__doc__

SUPPORT_YAML: TypeAlias = Union[Behavior, BehaviorModulation, Kinematics, Disc,
                                LineSegment, Neighbor]


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


from . import behavior_modulations, behaviors, kinematics

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
    "PropertyField", "Vector2", "Vector2Like", "register", "get_build_info",
    "BuildInfo", "get_build_dependencies", "get_plugins_dependencies",
    "DependencyInfo"
]
