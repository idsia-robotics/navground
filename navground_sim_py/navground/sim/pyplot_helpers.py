import math
from typing import Any
from collections.abc import Callable, Mapping, Sequence

import numpy as np
import numpy.typing
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Arrow, Circle
from navground import core, sim

Indices = list[int] | slice


def plot_agent(ax: Axes,
               agent: sim.Agent,
               pose: core.Pose2 | None = None,
               velocity: core.Vector2 | None = None,
               velocity_arrow_width: float = 0,
               velocity_arrow_color: str = 'k',
               color: str = "",
               alpha: float = 1.0,
               dot_color: str = "k",
               dot_radius: float = 0.25,
               with_safety_margin: bool = False) -> None:
    """
    Plots an agent as a disc together with:
    - a smaller dot to indicate the direction
    - an optional arrow for its velocity
    - an optional circle for the safety margin

    :param      ax:                   The pyplot axes
    :param      agent:                The agent
    :param      pose:                 A pose that override the agent own pose.
    :param      velocity:             A velocity that override the agent own velocity.
    :param      velocity_arrow_width: The width of the velocity arrow
    :param      color:                The color
    :param      alpha:                The opacity
    :param      dot_color:            The color of the dot
    :param      dot_radius:           The relative radius of the dot
                                      as fraction of the radius
    :param      with_safety_margin:   Whether to display the safety margin
    """
    if pose is None:
        pose = agent.pose
    position = pose.position
    if velocity is None:
        velocity = agent.velocity
    if not color:
        color = agent.color or 'b'
    circle = Circle(tuple(position), agent.radius, color=color, alpha=alpha)
    ax.add_patch(circle)
    dot_center = position + core.unit(
        pose.orientation) * agent.radius * (1 - dot_radius)
    dot = Circle(tuple(dot_center),
                 dot_radius * agent.radius,
                 color=dot_color,
                 alpha=alpha)
    ax.add_patch(dot)

    if velocity_arrow_width > 0 and np.any(velocity):
        vel = Arrow(position[0],
                    position[1],
                    velocity[0],
                    velocity[1],
                    width=velocity_arrow_width * agent.radius,
                    color=color,
                    edgecolor=velocity_arrow_color,
                    alpha=0.5)
        ax.add_patch(vel)
    if with_safety_margin and agent.behavior:
        safety_margin = agent.behavior.safety_margin
        if safety_margin > 0:
            c = Circle(tuple(position),
                       agent.radius + safety_margin,
                       color=color,
                       alpha=alpha,
                       fill=False,
                       linestyle='--')
            ax.add_patch(c)


def plot_world(ax: Axes,
               world: sim.World,
               obstacles_color: str = 'k',
               in_box: bool = False,
               no_ticks: bool = False,
               with_agents: bool = False,
               **kwargs: Any) -> None:
    """
    Plots a world.

    :param      ax:              The pyplot axes
    :param      world:           The world
    :param      obstacles_color: The color of obstacles and walls
    :param      in_box:          Whether to restrict the plot within the world bounding box
    :param      no_ticks:        Whether to remove the axis ticks
    :param      with_agents:     Whether to plot agents
    :param      kwargs:          Keywords passed to :py:func:`plot_agent`
    """
    for obstacle in world.obstacles:
        disc = obstacle.disc
        c = Circle(tuple(disc.position), disc.radius, color=obstacles_color)
        ax.add_patch(c)
    if with_agents:
        for agent in world.agents:
            plot_agent(ax, agent, **kwargs)
    for wall in world.walls:
        line = wall.line
        x, y = np.asarray((line.p1, line.p2)).T
        ax.add_line(Line2D(x, y, color=obstacles_color))
    bb = world.bounding_box
    ax.set_aspect('equal')
    if in_box:
        ax.set_ylim(bb.min_y, bb.max_y)
        ax.set_xlim(bb.min_x, bb.max_x)
    if no_ticks:
        ax.tick_params(left=False, bottom=False)
        ax.set_xticklabels([])
        ax.set_yticklabels([])


def plot_trajectory(ax: Axes,
                    poses: np.typing.NDArray[np.floating[Any]],
                    color: str,
                    agent: sim.Agent | None = None,
                    step: int = 0,
                    label: str = '',
                    **kwargs: Any) -> None:
    """
    Plots a trajectory composed by an array of poses ``(x, y, theta)``

    :param      ax:       The pyplot axes
    :param      poses:    The poses
    :param      color:    The color
    :param      agent:    The agent to display or none
    :param      step:     The regular step at which to display the agent
                          if provided or a dot
    :param      label:    The label
    :param      kwargs:   Keywords passed to :py:func:`plot_agent`
    """
    if step > 0:
        if agent:
            for x, y, theta in poses[::step]:
                plot_agent(ax,
                           agent,
                           color=color,
                           pose=core.Pose2((x, y), theta),
                           velocity=np.zeros(2),
                           **kwargs)
        else:
            ax.plot(*poses[::step, :2].T, '.', color=color)
    ax.plot(*poses[::, :2].T, '-', color=color, label=label)


def plot_run(
    ax: Axes,
    run: sim.ExperimentalRun | sim.RecordedExperimentalRun,
    agent_indices: Indices = slice(None),
    step: int = 0,
    label: str = '',
    with_agent: bool = False,
    color: Callable[[sim.Agent], str] | None = None,
    agent_kwargs: Mapping[str, Any] = {},
    with_world: bool = True,
    world_kwargs: Mapping[str, Any] = {},
) -> None:
    """
    Plots an experimental run

    :param      ax:             The pyplot axes
    :param      run:            The run
    :param      agent_indices:  The indices of the agents whose trajectory
                                should be plotted.
    :param      step:           The regular step at which to display the agent
                                if provided or a dot
    :param      label:          The label
    :param      with_agent:     Whether to display agents
    :param      color:          A function to assign a color to an agent.
                                If not provided, it will fallback to the agent's own color.
    :param      agent_kwargs:   Keywords arguments passed to :py:func:`plot_agent`
    :param      with_world:     Whether to display the world
    :param      world_kwargs:   Keywords arguments passed to :py:func:`plot_world`
    """
    if with_world:
        plot_world(ax, world=run.world, with_agents=False, **world_kwargs)
    lia = list(enumerate(run.world.agents))
    if isinstance(agent_indices, slice):
        lia = lia[agent_indices]
    else:
        lia = [lia[i] for i in agent_indices]
    for i, agent in lia:
        if run.poses is None:
            continue
        poses = np.asarray(run.poses)[:, i, :]
        if color:
            agent_color = color(agent)
        else:
            agent_color = agent.color
        plot_trajectory(ax,
                        poses=poses,
                        color=agent_color,
                        step=step,
                        agent=agent if with_agent else None,
                        label=label,
                        **agent_kwargs)


def plot_runs(runs: Sequence[sim.ExperimentalRun],
              columns: int = 1,
              width: float = 10,
              fig: Figure | None = None,
              hide_axes: bool = True,
              **kwargs: Any) -> Figure:
    """
    Plots several runs using :py:func:`plot_run` as subplots.

    :param      runs:     The runs
    :param      columns:  The number of subplots columns
    :param      width:    The width of a subplot
    :param      fig:      An optional figuer:
    :param      hide_axes: Whether to hide the axes
    :param      kwargs:   Keywords arguments passed to :py:func:`plot_run`

    :returns:   The figure
    """
    rows = math.ceil(len(runs) / columns)
    if not fig:
        fig, _ = plt.subplots(rows, columns)
        is_new_fig = True
    else:
        is_new_fig = False
    hs = []
    for ax, run in zip(fig.axes, runs, strict=False):
        plot_run(ax, run, **kwargs)
        bx = ax.get_xbound()
        by = ax.get_ybound()
        hs.append((by[1] - by[0]) / (bx[1] - bx[0]))
        if hide_axes:
            ax.set_axis_off()
    if is_new_fig:
        for ax in fig.axes[len(runs):]:
            ax.set_axis_off()
        height = float(np.median(hs) * width * rows / columns)
        fig.set_size_inches(width, height)
    return fig
