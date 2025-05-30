from __future__ import annotations

import dataclasses as dc
import inspect
from collections.abc import Callable, MutableMapping
from typing import TYPE_CHECKING, Any, TypeAlias, cast

import numpy as np
import numpy.typing
from navground import core

if TYPE_CHECKING:
    from .. import (Entity, World)

from ..bounds import Bounds

Rect: TypeAlias = 'Bounds | np.typing.NDArray[np.floating[Any]]'
Attributes: TypeAlias = MutableMapping[str, str]
Point: TypeAlias = core.Vector2
EntityDecorator: TypeAlias = 'Callable[[Entity], Attributes]'
WorldDecorator: TypeAlias = 'Callable[[Entity, World], Attributes]'
Decorate: TypeAlias = 'EntityDecorator | WorldDecorator'


def wrap_as_world_decorator(decorate: Decorate) -> WorldDecorator:
    sig = inspect.signature(decorate)
    if len(sig.parameters) == 1:

        def f(e: Entity, w: World) -> Attributes:
            return cast(EntityDecorator, decorate)(e)

        return f
    return cast(WorldDecorator, decorate)


def rect_around(center: core.Vector2, width: float | None,
                height: float | None) -> Rect:
    if width is None:
        width = height
    if height is None:
        height = width
    if height is None or width is None:
        raise ValueError("Provide at least one of height or width")
    delta = np.array((width * 0.5, height * 0.5))
    return center - delta, center + delta


@dc.dataclass
class RenderConfig:
    """

    Configuration shared by rendering functions, like
    :py:func:`navground.sim.ui.svg_for_world`.

    """
    precision: int = 2
    """The number of decimal digits for poses"""
    decorate: Decorate | None = None
    """A function to decorate entities.
Should return a dictionary of valid SVG style attributes,
e.g., ``{"fill": "red"}`` for a given entity.
"""
    bounds: Rect | None = None
    """The rectangular area to be displayed"""
    width: int = 600
    """The width in pixels"""
    min_height: int = 100
    """The minimum height in pixels"""
    relative_margin: float = 0.05
    """The relative margin"""
    background_color: str = 'snow'
    """A valid SVG color for the background"""
    display_shape: bool = False
    """Whether to display the agent circular shape"""
    display_safety_margin: bool = False
    """Whether to display the agent safety margin"""
    grid: float = 0
    """The size of the square grid tile (set to zero or negative to skip drawing a grid)"""
    grid_color: str = 'grey'
    """The color of the grid"""
    grid_thickness: float = 0.01
    """The thickness of the grid"""
    rotation: tuple[core.Vector2, float] | float | None = None
    """A planar rotation applied before drawing [rad]"""
    extras: list[Callable[[World], str]] = dc.field(default_factory=list)
    """Extra rendering added at the end of to the svg"""
    background_extras: list[Callable[[World],
                                     str]] = dc.field(default_factory=list)
    """Extra rendering added at the beginning of to the svg"""


render_default_config = RenderConfig()
"""Default config, shared by all rendering functions"""
