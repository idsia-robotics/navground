# import itertools
import math
from typing import Callable, Sequence, Optional

import numpy as np
from matplotlib import patches
from matplotlib import pyplot as plt
from navground import core, sim

Indices = list[int] | slice


def plot_agent(ax: plt.Axes,
               position: core.Vector2,
               radius: float,
               velocity: Optional[core.Vector2] = None,
               color: str | None = None,
               alpha: float = 0.5) -> None:
    circle = patches.Circle(tuple(position), radius, color=color, alpha=alpha)
    ax.add_patch(circle)
    if velocity is not None:
        vel = patches.Arrow(position[0],
                            position[1],
                            velocity[0],
                            velocity[1],
                            width=0.1,
                            color=color)
        ax.add_patch(vel)


def plot_world(ax: plt.Axes,
               world: sim.World,
               color: str = 'k',
               with_box: bool = False,
               with_agents: bool = False,
               agent_color: str = 'b') -> None:
    for obstacle in world.obstacles:
        disc = obstacle.disc
        c = plt.Circle(tuple(disc.position), disc.radius, color=color)
        ax.add_patch(c)
    if with_agents:
        for agent in world.agents:
            c = plt.Circle(tuple(agent.position), agent.radius, color=agent_color)
            ax.add_patch(c)
    for wall in world.walls:
        line = wall.line
        x, y = np.asarray((line.p1, line.p2)).T
        ax.add_line(plt.Line2D(x, y, color=color))
    bb = world.bounding_box
    ax.set_ylim(bb.min_y, bb.max_y)
    ax.set_xlim(bb.min_x, bb.max_x)
    ax.tick_params(left=False, bottom=False)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_aspect('equal')
    if not with_box:
        ax.axis('off')


def plot_trajectory(ax: plt.Axes,
                    points: np.ndarray,
                    radius: float,
                    color: str,
                    safety_margin: float = 0,
                    with_shape: bool = True,
                    step: int = 1,
                    label: str = '') -> None:
    ax.plot(*points[::].T, '-', color=color, label=label)
    if step > 0:
        if with_shape:
            for x, y in points[::step]:
                c = plt.Circle((x, y), radius, color=color, alpha=0.5)
                ax.add_patch(c)
                if safety_margin > 0:
                    c = plt.Circle((x, y),
                                   radius + safety_margin,
                                   color=color,
                                   alpha=0.5,
                                   fill=False,
                                   linestyle='--')
                    ax.add_patch(c)
        else:
            ax.plot(*points[::step].T, '.', color=color)


def plot_run(ax: plt.Axes,
             run: sim.ExperimentalRun,
             agent_indices: Indices = slice(None),
             step: int = 30,
             with_box: bool = False,
             agent_color: Callable[[sim.Agent], str] | None = None,
             with_shape: bool = True,
             with_world: bool = True,
             label: str = ''):
    if with_world:
        plot_world(ax, world=run.world, with_box=with_box)
    lia = list(enumerate(run.world.agents))
    if isinstance(agent_indices, slice):
        lia = lia[agent_indices]
    else:
        lia = [lia[i] for i in agent_indices]
    for i, agent in lia:
        points = run.poses[:, i, :2]
        if agent.behavior:
            safety_margin = agent.behavior.safety_margin
        else:
            safety_margin = 0
        if agent_color:
            color = agent_color(agent)
        else:
            color = agent.color
        plot_trajectory(ax,
                        points=points,
                        radius=agent.radius,
                        safety_margin=safety_margin,
                        color=color,
                        step=step,
                        with_shape=with_shape,
                        label=label)


def plot_runs(runs: Sequence[sim.ExperimentalRun],
              columns: int = 1,
              width: float = 10,
              fig: plt.Figure | None = None,
              **kwargs) -> plt.Figure:
    rows = math.ceil(len(runs) / columns)
    if not fig:
        fig, axs = plt.subplots(rows, columns)
        is_new_fig = True
        axs = axs.flatten()
    else:
        axs = fig.axes
        is_new_fig = False
    hs = []
    for ax, run in zip(axs, runs):
        plot_run(ax, run, **kwargs)
        bx = ax.get_xbound()
        by = ax.get_ybound()
        hs.append((by[1] - by[0]) / (bx[1] - bx[0]))
    if is_new_fig:
        for ax in axs[len(runs):]:
            ax.set_axis_off()
        height = np.median(hs) * width * rows / columns
        fig.set_size_inches(width, height)
    return fig
