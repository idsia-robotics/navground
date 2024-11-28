import math
import os
from collections import ChainMap
from collections.abc import Callable, Collection, MutableMapping
from typing import Any

import jinja2
import numpy as np
import numpy.typing
from navground import core

from .. import (Agent, BoundingBox, Bounds, Entity, Obstacle, Wall, World,
                bounds_for_world)

Rect = Bounds | np.typing.NDArray[np.floating[Any]]
Attributes = MutableMapping[str, str]
Point = core.Vector2
Decorate = Callable[[Entity], Attributes]

folder = os.path.dirname(os.path.realpath(__file__))
template_folder = os.path.join(folder, 'templates')
jinjia_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(template_folder))


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
    delta: core.Vector2 = np.zeros(2)) -> str:
    proto = agent.type or 'agent'
    if agent.color:
        kwargs = {'fill': agent.color}
    else:
        kwargs = {}
    attributes = entity_attributes(agent, agent.type or 'agent', prefix,
                                   attributes, **kwargs)
    return svg_g_use(proto,
                     agent.pose,
                     agent.radius,
                     precision,
                     attributes,
                     shape,
                     delta=delta)


def svg_for_world(world: World | None = None,
                  precision: int = 3,
                  decorate: Decorate | None = None,
                  bounds: Rect | None = None,
                  width: int = 600,
                  min_height: int = 100,
                  relative_margin: float = 0.05,
                  background_color: str = 'snow',
                  display_shape: bool = False,
                  grid: float = 0,
                  grid_color: str = 'grey',
                  grid_thickness: float = 0.01,
                  rotation: tuple[core.Vector2, float] | float | None = None,
                  extras: Collection[Callable[[World], str]] = []) -> str:
    """
    Draw the world as a SVG.

    :param      world:                  The world to display
    :param      precision:              The number of decimal digits for poses
    :param      decorate:               A function to decorate entities.
                                        Should return a dictionary of valid SVG style attributes,
                                        e.g., ``{"fill": "red"}`` for a given entity.
    :param      bounds:                 The rectangular area to be displayed
    :param      width:                  The width in pixels
    :param      min_height:             The minimum height in pixels
    :param      relative_margin:        The relative margin
    :param      background_color:       A valid SVG color for the background
    :param      display_shape:          Whether to display the agent circular shape
    :param      grid:                   The size of the square grid tile
                                        (set to zero or negative to skip drawing a grid)
    :param      grid_color:             The color of the grid
    :param      grid_thickness:         The thickness of the grid
    :param      rotation:               A planar rotation applied before drawing [rad]
    :param      extras:                 Provides extras rendering added at the end of to the svg

    :returns:   An SVG string
    """
    return _svg_for_world(world=world,
                          precision=precision,
                          decorate=decorate,
                          bounds=bounds,
                          width=width,
                          min_height=min_height,
                          relative_margin=relative_margin,
                          background_color=background_color,
                          display_shape=display_shape,
                          grid=grid,
                          grid_color=grid_color,
                          grid_thickness=grid_thickness,
                          rotation=rotation,
                          extras=extras)[0]


def _svg_for_world(
    world: World | None = None,
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
    grid: float = 0,
    grid_color: str = 'grey',
    grid_thickness: float = 0.01,
    rotation: tuple[core.Vector2, float] | float | None = None,
    extras: Collection[Callable[[World], str]] = [],
) -> tuple[str, dict[str, str]]:
    g = ""
    if world:
        if bounds is None:
            bounds = bounds_for_world(world)
        width, height, view_box, min_y = size(bounds, width, min_height,
                                              relative_margin)
        bb = BoundingBox(view_box[0], view_box[0] + view_box[2], min_y,
                         min_y + view_box[3])
        for wall in world.walls:
            g += svg_for_wall(wall, precision, prefix,
                              decorate(wall) if decorate else {})
        for delta, lbb in world.subdivide_bounding_box(bb, False):
            for obstacle in world.get_obstacles_in_region(lbb):
                g += svg_for_obstacle(obstacle,
                                      precision,
                                      prefix,
                                      decorate(obstacle) if decorate else {},
                                      delta=delta)
            for agent in world.get_agents_in_region(lbb):

                g += svg_for_agent(agent,
                                   precision,
                                   prefix,
                                   decorate(agent) if decorate else {},
                                   display_shape,
                                   delta=delta)

        # g = "<g id='_world'>"
        # for wall in world.walls:
        #     g += svg_for_wall(wall, precision, prefix,
        #                       decorate(wall) if decorate else {})
        # for obstacle in world.obstacles:
        #     g += svg_for_obstacle(obstacle, precision, prefix,
        #                           decorate(obstacle) if decorate else {})
        # for agent in world.agents:
        #     g += svg_for_agent(agent, precision, prefix,
        #                        decorate(agent) if decorate else {},
        #                        display_shape)
        # if bounds is None:
        #     bb = world.bounding_box
        #     bounds = bb.p1, bb.p2
        # width, height, view_box, min_y = size(bounds, width, min_height,
        #                                       relative_margin)
        # g += "</g>"
        # for delta in world.get_lattice_grid(include_zero=False, c8=True):
        #     g += f'<use href="#_world" x="{delta[0]}" y="{delta[1]}" />'
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
    else:
        epilog = ''
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
        epilog=epilog,
        **grid_kwargs,
        **dims), dims


# def world_bound(world: World) -> Rect:
#     ps = []
#     for wall in world.walls:
#         ps.append(wall.line.p1)
#         ps.append(wall.line.p2)
#     for obstacle in world.obstacles:
#         delta = np.ones(2) * obstacle.disc.radius
#         p = obstacle.disc.position
#         ps.append(p - delta)
#         ps.append(p + delta)
#     for agent in world.agents:
#         delta = np.ones(2) * agent.radius
#         ps.append(agent.position - delta)
#         ps.append(agent.position + delta)
#     if not ps:
#         return np.zeros(2), np.zeros(2)
#     return np.min(ps, axis=0), np.max(ps, axis=0)


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
