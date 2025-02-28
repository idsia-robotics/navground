from __future__ import annotations

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
from matplotlib.transforms import Affine2D, Bbox
from navground import core, sim

Indices = list[int] | slice


def plot_agent(ax: Axes,
               agent: sim.Agent,
               pose: core.Pose2 | None = None,
               velocity: core.Vector2 | None = None,
               velocity_arrow_width: float = 0,
               velocity_arrow_edge_color: str = 'k',
               velocity_arrow_alpha: float = 0.5,
               color: str = "",
               alpha: float = 1.0,
               dot_color: str = "k",
               dot_radius: float = 0.25,
               with_safety_margin: bool = False,
               transform: Affine2D | None = None,
               zorder: int = 1) -> None:
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
    :param      velocity_arrow_edge_color: The stroke color of the velocity arrow
    :param      velocity_arrow_alpha: The opacity of the velocity arrow
    :param      color:                The color
    :param      alpha:                The opacity
    :param      dot_color:            The color of the dot
    :param      dot_radius:           The relative radius of the dot
                                      as fraction of the radius
    :param      with_safety_margin:   Whether to display the safety margin
    :param      transform:            An optional affine transformation to apply to the plot
    :param      zorder:               The Z-order
    """
    if transform:
        kwargs = {'transform': transform + ax.transData}
    else:
        kwargs = {}
    if pose is None:
        pose = agent.pose
    position = pose.position
    if velocity is None:
        velocity = agent.velocity
    if not color:
        color = agent.color or 'b'
    circle = Circle(tuple(position),
                    agent.radius,
                    color=color,
                    alpha=alpha,
                    zorder=zorder,
                    **kwargs)
    ax.add_patch(circle)
    dot_center = position + core.unit(
        pose.orientation) * agent.radius * (1 - dot_radius)
    dot = Circle(tuple(dot_center),
                 dot_radius * agent.radius,
                 color=dot_color,
                 alpha=alpha,
                 zorder=zorder,
                 **kwargs)
    ax.add_patch(dot)

    if velocity_arrow_width > 0 and np.any(velocity):
        vel = Arrow(position[0],
                    position[1],
                    velocity[0],
                    velocity[1],
                    width=velocity_arrow_width * agent.radius,
                    facecolor=color,
                    edgecolor=velocity_arrow_edge_color,
                    alpha=velocity_arrow_alpha,
                    zorder=zorder,
                    **kwargs)
        ax.add_patch(vel)
    if with_safety_margin and agent.behavior:
        safety_margin = agent.behavior.safety_margin
        if safety_margin > 0:
            c = Circle(tuple(position),
                       agent.radius + safety_margin,
                       color=color,
                       alpha=alpha,
                       fill=False,
                       linestyle='--',
                       zorder=zorder,
                       **kwargs)
            ax.add_patch(c)


def plot_world(ax: Axes,
               world: sim.World,
               obstacles_color: str = 'k',
               obstacles_alpha: float = 1.0,
               in_box: bool = False,
               no_ticks: bool = False,
               with_agents: bool = False,
               transform: Affine2D | None = None,
               zorder: int = 1,
               **kwargs: Any) -> None:
    """
    Plots a world.

    :param      ax:              The pyplot axes
    :param      world:           The world
    :param      obstacles_color: The color of obstacles and walls
    :param      obstacles_alpha: The opacity of obstacles and walls
    :param      in_box:          Whether to restrict the plot within the world bounding box
    :param      no_ticks:        Whether to remove the axis ticks
    :param      with_agents:     Whether to plot agents
    :param      transform:       An optional affine transformation to apply to the plot
    :param      zorder:          The Z-order
    :param      kwargs:          Keywords passed to :py:func:`plot_agent`
    """
    if transform:
        patch_kwargs = {'transform': transform + ax.transData}
    else:
        patch_kwargs = {}
    for obstacle in world.obstacles:
        disc = obstacle.disc
        c = Circle(tuple(disc.position),
                   disc.radius,
                   color=obstacles_color,
                   alpha=obstacles_alpha,
                   **patch_kwargs)
        ax.add_patch(c)
    if with_agents:
        for agent in world.agents:
            plot_agent(ax, agent, transform=transform, **kwargs)
    for wall in world.walls:
        line = wall.line
        x, y = np.asarray((line.p1, line.p2)).T
        ax.add_line(
            Line2D(x, y, color=obstacles_color, zorder=zorder,
                   **patch_kwargs))  # type: ignore[arg-type]
    bb = world.bounding_box
    ax.set_aspect('equal')
    if in_box:
        if transform:
            mbb = Bbox([(bb.min_x, bb.min_y), (bb.max_x, bb.max_y)])
            mbb = transform.transform_bbox(mbb)
            bb = sim.BoundingBox(min_x=mbb.xmin,
                                 max_x=mbb.xmax,
                                 min_y=mbb.ymin,
                                 max_y=mbb.ymax)
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
                    plot_last_pose: bool = False,
                    zorder: int = 1,
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
    :param      plot_last_pose: Whether to force plotting the last agent pose
    :param      zorder:   The Z-order
    :param      kwargs:   Keywords passed to :py:func:`plot_agent`
    """
    ax.plot(*poses[::, :2].T, '-', color=color, label=label, zorder=zorder)
    if step > 0:
        if agent:
            empty = True
            for pose in poses[::step]:
                x, y, theta = pose
                empty = False
                plot_agent(ax,
                           agent,
                           color=color,
                           pose=core.Pose2((x, y), theta),
                           velocity=np.zeros(2),
                           zorder=zorder,
                           **kwargs)
            if plot_last_pose and (empty
                                   or not np.isclose(pose, poses[-1]).all()):
                x, y, theta = poses[-1]
                plot_agent(ax,
                           agent,
                           color=color,
                           pose=core.Pose2((x, y), theta),
                           velocity=np.zeros(2),
                           zorder=zorder,
                           **kwargs)
        else:
            ax.plot(*poses[::step, :2].T, '.', color=color, zorder=zorder)


def plot_run(
    ax: Axes,
    run: sim.ExperimentalRun | sim.RecordedExperimentalRun,
    agent_indices: Indices = slice(None),
    step: int = 0,
    label: str = '',
    with_agent: bool = False,
    color: Callable[[sim.Agent], str] | None = None,
    zorder: int = 1,
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
        plot_world(ax,
                   world=run.world,
                   with_agents=False,
                   zorder=zorder,
                   **world_kwargs)
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
                        zorder=zorder,
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


def transform_from_pose(pose: core.Pose2) -> Affine2D:
    """
    Compute an affine transform from a pose

    :param      pose:  The pose

    :returns:   The corresponding affine transformation
    """
    return Affine2D().rotate(pose.orientation).translate(*pose.position)


def plot_grid_map(ax: Axes,
                  gridmap: core.GridMap,
                  cmap='gray',
                  vmin: int = 0,
                  vmax: int = 255,
                  transform: Affine2D | None = None) -> None:
    """
    Plot a grid map

    :param      ax:         The axes
    :param      gridmap:    The gridmap
    :param      cmap:       The color map
    :param      vmin:       The minimal value of the grid map
    :param      vmax:       The maximal value of the grid map
    :param      transform:  An optional transform to apply to the gridmap
    """
    kwargs = {}
    if transform:
        kwargs['transform'] = transform + ax.transData
    image = gridmap.map
    height, width = image.shape
    resolution = gridmap.resolution
    x0, y0 = gridmap.origin
    x1 = x0 + width * resolution
    y1 = y0 + height * resolution
    ax.imshow(image,
              cmap=cmap,
              vmin=vmin,
              vmax=vmax,
              interpolation='nearest',
              extent=(x0, x1, y0, y1),
              origin="lower",
              **kwargs)  # type: ignore[arg-type]


def plot_scan(ax: Axes,
              scan: sim.state_estimations.LidarScan,
              pose: core.Pose2 = core.Pose2(),
              color: str = 'r',
              alpha: float = 0.5) -> None:
    """
    Plot a Lidar scan

    :param      ax:     The axes
    :param      scan:   The scan
    :param      pose:   The pose of reference frame of the scan
    :param      color:  The color
    :param      alpha:  The opacity
    """
    a = np.asarray(scan.angles) + pose.orientation
    ps = pose.position
    x = np.cos(a) * scan.ranges + ps[0]
    y = np.sin(a) * scan.ranges + ps[1]
    ax.plot(x, y, '.', alpha=alpha, color=color)
