from __future__ import annotations

import dataclasses as dc
import math
import os
from collections import ChainMap
from collections.abc import Callable, Collection
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing
from navground import core

if TYPE_CHECKING:
    from .. import (Agent, Entity, Obstacle, Wall, World)
    from .common import Attributes, Point, Decorate, Rect

from ..bounds import bounds_for_world
from .common import render_default_config, wrap_as_world_decorator


def svg_color(r: float, g: float, b: float) -> str:
    """
    Returns the SVG representation of an RGB color

    :param      r:    in [0, 1]
    :param      g:    in [0, 1]
    :param      b:    in [0, 1]

    :returns:   The SVG representation of the color
    """
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def flat(attributes: Attributes) -> str:
    attributes = dict(attributes)
    r = ''
    for k in ('id', 'class'):
        if k in attributes:
            value = attributes.pop(k)
            r += f'{k}="{value}" '
    if attributes:
        value = ';'.join(f"{k}:{v}" for k, v in attributes.items())
        r += f'style="{value}"'
    return r


def flat_dict(attributes: Attributes) -> dict[str, str]:
    attributes = dict(attributes)
    r: dict[str, str] = {}
    if 'class' in attributes:
        r['class'] = attributes.pop('class')
    r['style'] = ';'.join(f"{k}:{v}" for k, v in attributes.items())
    return r


def default_decoration(e: Entity) -> Attributes:
    return {}


def svg_polyline(points: list[Point],
                 precision: int = 2,
                 attributes: Attributes = {},
                 **kwargs: str) -> str:
    attributes = ChainMap(attributes, kwargs)
    ps = ' '.join(f"{cs[0]},{cs[1]}"
                  for cs in np.round(points, decimals=precision))
    return f"<polyline {flat(attributes)} points='{ps}'/>"


def svg_circle(center: Point,
               radius: float,
               precision: int = 2,
               attributes: Attributes = {},
               **kwargs: str) -> str:
    attributes = ChainMap(attributes, kwargs)
    cx, cy = np.round(center, decimals=precision)
    r = np.round(radius, decimals=precision)
    return f"<circle {flat(attributes)} cx='{cx}' cy='{cy}' r='{r}'/>"


def svg_g_circle(center: Point,
                 radius: float,
                 precision: int = 2,
                 attributes: Attributes = {},
                 **kwargs: str) -> str:
    attributes = ChainMap(attributes, kwargs)
    cx, cy = np.round(center, decimals=precision)
    r = np.round(radius, decimals=precision)
    c = f"<circle  transform='scale({r}, {r})' cx='0' cy='0' r='1'/>"
    return f"<g {flat(attributes)} transform='translate({cx}, {cy})'>{c}</g>"


def svg_g_use(proto: str,
              pose: core.Pose2,
              radius: float,
              precision: int = 2,
              attributes: Attributes = {},
              shape: bool = False,
              safety_margin: float | None = None,
              delta: core.Vector2 = np.zeros(2),
              **kwargs: str) -> str:
    attributes = ChainMap(attributes, kwargs)
    cx, cy = np.round(pose.position + delta, decimals=precision)
    angle = np.round(pose.orientation * 180 / math.pi, decimals=precision)
    r = np.round(radius, decimals=precision)
    g = (
        f'<g {flat(attributes)} transform="translate({cx}, {cy}) rotate({angle})">'
        f'<use xlink:href="#{proto}" transform="scale({r}, {r})"/>')
    if shape:
        g += f'<circle cx="0" cy="0" r="{r}" class="shape"/>'
    if safety_margin is not None:
        g += f'<circle cx="0" cy="0" r="{r + safety_margin}" class="safety_margin"/>'
    g += '</g>'
    return g


def entity_attributes(e: Entity,
                      name: str = '',
                      prefix: str = '',
                      attributes: Attributes = {},
                      **kwargs: Any) -> Attributes:
    return ChainMap(attributes, {
        'class': name,
        'id': f'{prefix}{e._uid}'
    }, kwargs)


def svg_for_wall(wall: Wall,
                 precision: int = 2,
                 prefix: str = '',
                 attributes: Attributes = {}) -> str:
    attributes = entity_attributes(wall, 'wall', prefix, attributes)
    return svg_polyline([wall.line.p1, wall.line.p2], precision, attributes)


def svg_for_obstacle(
    obstacle: Obstacle,
    precision: int = 2,
    prefix: str = '',
    attributes: Attributes = {},
    delta: core.Vector2 = np.zeros(2)) -> str:
    attributes = entity_attributes(obstacle, 'obstacle', prefix, attributes)
    return svg_circle(obstacle.disc.position + delta, obstacle.disc.radius,
                      precision, attributes)


def svg_for_agent(
    agent: Agent,
    precision: int = 2,
    prefix: str = '',
    attributes: Attributes = {},
    shape: bool = False,
    with_safety_margin: bool = False,
    delta: core.Vector2 = np.zeros(2)) -> str:
    proto = agent.type or 'agent'
    if agent.color:
        kwargs = {'fill': agent.color}
    else:
        kwargs = {}
    attributes = entity_attributes(agent, agent.type or 'agent', prefix,
                                   attributes, **kwargs)
    if with_safety_margin and agent.behavior:
        safety_margin = agent.behavior.safety_margin
    else:
        safety_margin = None
    return svg_g_use(proto,
                     agent.pose,
                     agent.radius,
                     precision,
                     attributes,
                     shape,
                     safety_margin,
                     delta=delta)


def svg_for_world(world: World | None = None, **kwargs: Any) -> str:
    """
    Draw the world as a SVG.

    :param      world:   The world to display
    :param      kwargs:  Optional configuration:
        same fields as :py:class:`navground.sim.ui.RenderConfig`.

    The actual configuration is computed by looking (in order) to

    1. the arguments of this function;
    2. the world-specific configuration :py:attr:`navground.sim.World.render_kwargs`;
    3. the default configuration :py:attr:`navground.sim.ui.render_default_config`.

    :returns:   An SVG string
    """
    return svg_for_world_and_dims(world, **kwargs)[0]


def svg_for_world_and_dims(world: World | None = None,
                           **kwargs: Any) -> tuple[str, dict[str, str]]:
    full_kwargs = dc.asdict(render_default_config)
    if world:
        full_kwargs.update(world.render_kwargs)
    full_kwargs.update(kwargs)
    return _svg_for_world_and_dims(world=world, **full_kwargs)


def _svg_for_world_and_dims(world: World | None = None,
                            *,
                            prefix: str = '',
                            precision: int = 2,
                            decorate: Decorate | None = None,
                            standalone: bool = True,
                            bounds: Rect | None = None,
                            width: int = 600,
                            min_height: int = 100,
                            relative_margin: float = 0.05,
                            include_default_style: bool = True,
                            external_style_path: str = '',
                            style: str = '',
                            background_color: str = 'snow',
                            display_shape: bool = False,
                            display_safety_margin: bool = False,
                            grid: float = 0,
                            grid_color: str = 'grey',
                            grid_thickness: float = 0.01,
                            rotation: tuple[core.Vector2, float] | float
                            | None = None,
                            extras: Collection[Callable[[World], str]] = [],
                            background_extras: Collection[Callable[[World],
                                                                   str]] = [],
                            **kwargs: Any) -> tuple[str, dict[str, str]]:
    import jinja2

    from .. import BoundingBox

    g = ""
    if world:
        if bounds is None:
            bounds = bounds_for_world(world)
        width, height, view_box, min_y = size(bounds, width, min_height,
                                              relative_margin)
        bb = BoundingBox(view_box[0], view_box[0] + view_box[2], min_y,
                         min_y + view_box[3])
        if decorate:
            w_decorate = wrap_as_world_decorator(decorate)
        else:
            w_decorate = None
        for wall in world.walls:
            g += svg_for_wall(wall, precision, prefix,
                              w_decorate(wall, world) if w_decorate else {})
        for delta, lbb in world.subdivide_bounding_box(bb, False):
            for obstacle in world.get_obstacles_in_region(lbb):
                g += svg_for_obstacle(
                    obstacle,
                    precision,
                    prefix,
                    w_decorate(obstacle, world) if w_decorate else {},
                    delta=delta)
            for agent in world.get_agents_in_region(lbb):

                g += svg_for_agent(
                    agent,
                    precision,
                    prefix,
                    w_decorate(agent, world) if w_decorate else {},
                    display_shape,
                    display_safety_margin,
                    delta=delta)
    else:
        height = width
        view_box = (0, 0, 1, 1)
        min_y = 0
    dims = {
        'width':
        f'{width}',
        'height':
        f'{height}',
        'viewBox':
        f"{view_box[0]:.4f} {view_box[1]:.4f} {view_box[2]:.4f} {view_box[3]:.4f}"
    }

    if grid > 0:
        min_x = (view_box[0] // grid - 1) * grid
        max_x = ((view_box[0] + view_box[2]) // grid + 2) * grid
        max_y = (-view_box[1] // grid + 2) * grid
        min_y = ((-view_box[1] - view_box[3]) // grid - 1) * grid
        grid_xs = np.arange(min_x, max_x, grid)
        grid_ys = np.arange(min_y, max_y, grid)
        grid_kwargs = {
            'grid_xs': grid_xs,
            'grid_ys': grid_ys,
            'grid_color': grid_color,
            'grid_thickness': grid_thickness
        }
    else:
        grid_kwargs = {}

    if rotation is not None:
        if isinstance(rotation, tuple):
            (x, y), theta = rotation
        else:
            # rotate around center
            x = view_box[0] + view_box[2] / 2
            y = -view_box[1] - view_box[3] / 2
            theta = rotation
        r = f'rotate({theta / math.pi * 180: .4f}, {x: .4f}, {y: .4f})'
    else:
        r = ''
    if world:
        epilog = '\n'.join([e(world) for e in extras])
        prolog = '\n'.join([e(world) for e in background_extras])
    else:
        epilog = ''
        prolog = ''
    folder = os.path.dirname(os.path.realpath(__file__))
    template_folder = os.path.join(folder, 'templates')
    jinjia_env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(template_folder))
    return jinjia_env.get_template('world.svg').render(
        svg_world=g,
        prefix=prefix,
        external_style_path=external_style_path,
        include_default_style=include_default_style,
        style=style,
        background_color=background_color,
        display_shape=display_shape,
        grid=grid,
        rotation=r,
        prolog=prolog,
        epilog=epilog,
        **grid_kwargs,
        **dims), dims


def size(
    bounds: Rect,
    width: int = 600,
    min_height: float = 100,
    relative_margin: float = 0.05
) -> tuple[int, int, tuple[float, float, float, float], float]:
    # We want python not numpy floats
    min_p, max_p = [np.asarray(xs).tolist() for xs in bounds]
    min_x, min_y = min_p
    max_x, max_y = max_p
    world_width = max_x - min_x
    world_height = max_y - min_y
    min_x -= world_width * relative_margin
    min_y -= world_height * relative_margin
    max_x += world_width * relative_margin
    max_y += world_height * relative_margin
    world_width = max_x - min_x
    world_height = max_y - min_y
    if world_width:
        scale = width / world_width
    else:
        scale = 1
    height = int(max(min_height, scale * world_height))
    if height % 2:
        height = height + 1
    return width, height, (min_x, -max_y, world_width, world_height), min_y


def save(world: World, path: str = '', **kwargs: Any) -> None:
    svg = svg_for_world(world, **kwargs)
    with open(path, 'w') as f:
        f.write(svg)
