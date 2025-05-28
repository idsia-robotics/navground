from __future__ import annotations

from typing import TYPE_CHECKING, TypeAlias

from navground import core

if TYPE_CHECKING:
    from . import BoundingBox, World

Bounds: TypeAlias = tuple[core.Vector2, core.Vector2]


def bounds_of_bounding_box(bb: BoundingBox) -> Bounds:
    return bb.p1, bb.p2


def bounds_for_world(world: World) -> Bounds:
    return bounds_of_bounding_box(world.bounding_box)
