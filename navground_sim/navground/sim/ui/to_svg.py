import itertools
import operator
import os
from typing import (Any, Callable, Dict, Iterable, List, Optional, Tuple,
                    Union, cast)

import jinja2
import numpy as np

from .. import Agent, Entity, Obstacle, Wall, World

Point = np.ndarray
Rect = Tuple[Point, Point]
Attribute = Tuple[str, str]
Decorate = Callable[[Entity], List[Attribute]]

folder = os.path.dirname(os.path.realpath(__file__))
template_folder = os.path.join(folder, 'templates')
jinjia_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(template_folder))


def join_values(name: str, attributes: Iterable[Attribute]) -> str:
    if name == 'class':
        return ' '.join(value for name, value in attributes)
    attribute_list = list(attributes)
    if attribute_list:
        return attribute_list[-1][1]
    return ''


def flat(attributes: Iterable[Attribute]) -> str:
    key = operator.itemgetter(0)
    return ' '.join(f'{name}="{join_values(name, values)}"'
                    for name, values in itertools.groupby(attributes, key=key))


def default_decoration(e: Entity) -> List[Attribute]:
    return []


def svg_polyline(points: List[Point],
                 precision: int = 2,
                 attributes: List[Attribute] = [],
                 **kwargs: str) -> str:
    attributes = attributes + list(kwargs.items())
    ps = ' '.join(f"{cs[0]},{cs[1]}"
                  for cs in np.round(points, decimals=precision))
    return f"<polyline {flat(attributes)} points='{ps}'/>"


def svg_circle(center: Point,
               radius: float,
               precision: int = 2,
               attributes: List[Attribute] = [],
               **kwargs: str) -> str:
    attributes = attributes + list(kwargs.items())
    cx, cy = np.round(center, decimals=precision)
    r = np.round(radius, decimals=precision)
    return f"<circle {flat(attributes)} cx='{cx}' cy='{cy}' r='{r}'/>"


def svg_g_circle(center: Point,
                 radius: float,
                 precision: int = 2,
                 attributes: List[Attribute] = [],
                 **kwargs: str) -> str:
    attributes = attributes + list(kwargs.items())
    cx, cy = np.round(center, decimals=precision)
    r = np.round(radius, decimals=precision)
    c = f"<circle  transform='scale({r}, {r})' cx='0' cy='0' r='1'/>"
    return f"<g {flat(attributes)} transform='translate({cx}, {cy})'>{c}</g>"


def entity_attributes(e: Entity,
                      class_name: str = '',
                      decorate: Decorate = default_decoration,
                      prefix: str = ''):
    return decorate(e) + [('id', f'{prefix}{e._uid}'), ('class', class_name)]


def svg_for_wall(wall: Wall,
                 precision: int = 2,
                 decorate: Decorate = default_decoration,
                 prefix: str = '') -> str:
    attributes = entity_attributes(wall, 'wall', decorate, prefix)
    return svg_polyline([wall.line.p1, wall.line.p2], precision, attributes)


def svg_for_obstacle(obstacle: Obstacle,
                     precision: int = 2,
                     decorate: Decorate = default_decoration,
                     prefix: str = '') -> str:
    attributes = entity_attributes(obstacle, 'obstacle', decorate, prefix)
    return svg_circle(obstacle.disc.position, obstacle.disc.radius, precision,
                      attributes)


def svg_for_agent(agent: Agent,
                  precision: int = 2,
                  decorate: Decorate = default_decoration,
                  prefix: str = '') -> str:
    attributes = entity_attributes(agent, 'agent', decorate, prefix)
    return svg_g_circle(agent.position, agent.radius, precision, attributes)


def svg_for_world(world: World,
                  precision: int = 2,
                  decorate: Decorate = default_decoration,
                  interactive: bool = False,
                  standalone: bool = True,
                  bounds: Optional[Rect] = None,
                  width: float = 600,
                  min_height: float = 100,
                  relative_margin: float = 0.05,
                  include_style_path: Optional[str] = None,
                  background: bool = True,
                  prefix: str = '',
                  style: str = '') -> Union[Tuple[str, Dict[str, str]], str]:
    g = ""
    for wall in world.walls:
        g += svg_for_wall(wall, precision, decorate, prefix)
    for obstacle in world.obstacles:
        g += svg_for_obstacle(obstacle, precision, decorate, prefix)
    for agent in world.agents:
        g += svg_for_agent(agent, precision, decorate, prefix)
    width, height, view_box, min_y = size(world, bounds, width, min_height,
                                          relative_margin)
    dims = {
        'width':
        f'{width:.0f}',
        'height':
        f'{height:.0f}',
        'viewBox':
        f"{view_box[0]:.2f} {view_box[1]:.2f} {view_box[2]:.2f} {view_box[3]:.2f}"
    }
    if background:
        g += ("<rect class='background' "
              f"height='{view_box[3]:.2f}' width='{view_box[2]:.2f}' "
              f"x='{view_box[0]:.2f}' y='{min_y:.2f}'/>")
    g += ""
    if not standalone:
        return g, dims
    else:
        return jinjia_env.get_template('world.svg').render(
            svg_world=g,
            include_style_path=include_style_path,
            background=background,
            style=style,
            **dims)


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
    return np.min(ps, axis=0), np.max(ps, axis=0)


def size(
    world: World,
    bounds: Optional[Rect] = None,
    width: float = 600,
    min_height: float = 100,
    relative_margin: float = 0.05
) -> Tuple[float, float, Tuple[float, float, float, float], float]:
    min_p, max_p = world_bound(world)
    if bounds:
        min_p = np.min((min_p, bounds[0]), axis=0)
        max_p = np.max((max_p, bounds[1]), axis=0)
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
