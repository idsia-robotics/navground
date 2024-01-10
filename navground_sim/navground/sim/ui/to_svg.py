import math
import os
from collections import ChainMap
from typing import Any, Callable, List, Mapping, Optional, Tuple, cast

import jinja2
import numpy as np
from navground import core

from .. import Agent, Entity, Obstacle, Wall, World

Point = np.ndarray
Rect = Tuple[Point, Point]
Attributes = Mapping[str, str]
Decorate = Callable[[Entity], Attributes]

folder = os.path.dirname(os.path.realpath(__file__))
template_folder = os.path.join(folder, 'templates')
jinjia_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(template_folder))


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


def default_decoration(e: Entity) -> Attributes:
    return {}


def svg_polyline(points: List[Point],
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
              **kwargs: str) -> str:
    attributes = ChainMap(attributes, kwargs)
    cx, cy = np.round(pose.position, decimals=precision)
    angle = np.round(pose.orientation * 180 / math.pi, decimals=precision)
    r = np.round(radius, decimals=precision)
    g = (
        f'<g {flat(attributes)} transform="translate({cx}, {cy}) rotate({angle})">'
        f'<use xlink:href="#{proto}" transform="scale({r}, {r})"/>')
    if shape:
        g += f'<circle cx="0" cy="0" r="{r}" class="shape"/></g>'


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


def svg_for_obstacle(obstacle: Obstacle,
                     precision: int = 2,
                     prefix: str = '',
                     attributes: Attributes = {}) -> str:
    attributes = entity_attributes(obstacle, 'obstacle', prefix, attributes)
    return svg_circle(obstacle.disc.position, obstacle.disc.radius, precision,
                      attributes)


def svg_for_agent(agent: Agent,
                  precision: int = 2,
                  prefix: str = '',
                  attributes: Attributes = {},
                  shape: bool = False) -> str:
    proto = agent.type or 'agent'
    attributes = entity_attributes(agent,
                                   agent.type or 'agent',
                                   prefix,
                                   attributes,
                                   fill=agent.color)
    return svg_g_use(proto, agent.pose, agent.radius, precision, attributes, shape)


def svg_for_world(*args: Any, **kwargs: Any) -> str:
    return _svg_for_world(*args, **kwargs)[0]


def _svg_for_world(world: Optional[World] = None,
                   prefix: str = '',
                   precision: int = 2,
                   decorate: Decorate = default_decoration,
                   interactive: bool = False,
                   standalone: bool = True,
                   bounds: Optional[Rect] = None,
                   width: float = 600,
                   min_height: float = 100,
                   relative_margin: float = 0.05,
                   include_default_style: bool = True,
                   external_style_path: str = '',
                   style: str = '',
                   background_color: str = 'snow',
                   display_shape: bool = False) -> str:
    g = ""
    if world:
        for wall in world.walls:
            g += svg_for_wall(wall, precision, prefix, decorate(wall))
        for obstacle in world.obstacles:
            g += svg_for_obstacle(obstacle, precision, prefix,
                                  decorate(obstacle))
        for agent in world.agents:
            g += svg_for_agent(agent, precision, prefix, decorate(agent), display_shape)
        width, height, view_box, min_y = size(world, bounds, width, min_height,
                                              relative_margin)
    else:
        height = width
        view_box = (0, 0, 1, 1)
        min_y = 0
    dims = {
        'width':
        f'{width:.0f}',
        'height':
        f'{height:.0f}',
        'viewBox':
        f"{view_box[0]:.2f} {view_box[1]:.2f} {view_box[2]:.2f} {view_box[3]:.2f}"
    }
    return jinjia_env.get_template('world.svg').render(
        svg_world=g,
        prefix=prefix,
        external_style_path=external_style_path,
        include_default_style=include_default_style,
        style=style,
        background_color=background_color,
        display_shape=display_shape,
        **dims), dims


def world_bound(world: World) -> Rect:
    ps = []
    for wall in world.walls:
        ps.append(wall.line.p1)
        ps.append(wall.line.p2)
    for obstacle in world.obstacles:
        delta = np.ones(2) * obstacle.disc.radius
        p = obstacle.disc.position
        ps.append(p - delta)
        ps.append(p + delta)
    for agent in world.agents:
        delta = np.ones(2) * agent.radius
        ps.append(agent.position - delta)
        ps.append(agent.position + delta)
    if not ps:
        return np.zeros(2), np.zeros(2)
    return np.min(ps, axis=0), np.max(ps, axis=0)


def size(
    world: Optional[World] = None,
    bounds: Optional[Rect] = None,
    width: float = 600,
    min_height: float = 100,
    relative_margin: float = 0.05
) -> Tuple[float, float, Tuple[float, float, float, float], float]:
    if world is None and bounds is None:
        raise ValueError("Provide at least one of world or bounds")
    if world:
        min_p, max_p = world_bound(world)
        if bounds:
            min_p = np.min((min_p, bounds[0]), axis=0)
            max_p = np.max((max_p, bounds[1]), axis=0)
    else:
        min_p, max_p = bounds
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
    scale = width / world_width
    height = max(min_height, scale * world_height)
    return width, height, (min_x, -max_y, world_width, world_height), min_y


def save(world: World, path: str = '', **kwargs: Any) -> None:
    svg = svg_for_world(world, standalone=True, **kwargs)
    svg = cast(str, svg)
    with open(path, 'w') as f:
        f.write(svg)
