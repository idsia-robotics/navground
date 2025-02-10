from __future__ import annotations

import pathlib
from typing import Any

import moviepy as mpy  # type: ignore[import-untyped]

# No more `editor` in moviepy>=2
# https://zulko.github.io/moviepy/getting_started/updating_to_v2.html

if mpy.__version__ > '2':
    MOVIEPY_VERSION = 2
else:
    import moviepy.editor as mpy  # type: ignore[import-untyped]
    MOVIEPY_VERSION = 1

import numpy as np
from navground import core

from .. import (Agent, ExperimentalRun, RecordedExperimentalRun, World,
                bounds_of_bounding_box)
from .render import Image, image_for_world
from .to_svg import Rect


def make_video(world: World,
               time_step: float,
               duration: float,
               factor: float = 1.0,
               background_color: str = "snow",
               follow: Agent | None = None,
               bounds: Rect | None = None,
               terminate_when_all_idle_or_stuck: bool = True,
               rotation: tuple[core.Vector2, float] | float | None = None,
               **kwargs: Any) -> mpy.VideoClip:
    t0 = world.time
    theta = 0.0

    if follow:
        if bounds is None:
            relative_bounds = np.asarray(((-1, -1), (1, 1)))
        else:
            relative_bounds = np.asarray(bounds)
        if isinstance(rotation, float):
            theta = rotation

    def make_frame(t: float) -> Image:
        nonlocal bounds
        nonlocal rotation
        t = t * factor
        while world.time - t0 + time_step < t:
            world.update(time_step)
        dt = t - world.time - t0
        if dt > 0:
            world.update(dt)
        if follow:
            bounds = relative_bounds + follow.position
            if theta:
                rotation = (follow.position, theta)

        # TODO(Jerome): they don't have always the same size ...
        return image_for_world(world,
                               background_color=background_color,
                               bounds=bounds,
                               rotation=rotation,
                               **kwargs)

    return mpy.VideoClip(make_frame, duration=duration / factor)


def make_video_from_run(run: RecordedExperimentalRun | ExperimentalRun,
                        factor: float = 1.0,
                        background_color: str = "snow",
                        follow: Agent | None = None,
                        bounds: Rect | None = None,
                        from_time: float = 0,
                        to_time: float | None = None,
                        rotation: tuple[core.Vector2, float] | float
                        | None = None,
                        **kwargs: Any) -> mpy.VideoClip:

    frame = None
    step = -1
    theta = 0.0

    if to_time is None:
        to_time = run.final_sim_time

    to_time = max(0, min(run.final_sim_time, to_time))
    from_time = min(from_time, max(0, from_time))
    run.reset()

    if follow:
        if bounds is None:
            relative_bounds = np.asarray(((-1, -1), (1, 1)))
        else:
            relative_bounds = np.asarray(bounds)
        if isinstance(rotation, float):
            theta = rotation
    elif not bounds:
        bounds = bounds_of_bounding_box(run.bounding_box)

    def make_frame(t: float) -> Image:
        nonlocal frame
        nonlocal step
        nonlocal bounds
        nonlocal rotation
        t = from_time + t * factor
        new_step = int(t // run.time_step)
        if new_step != step or frame is None:
            if run.go_to_step(new_step):
                step = new_step
                if follow:
                    bounds = relative_bounds + follow.position
                    if theta:
                        rotation = (follow.position, theta)
                # TODO(Jerome): they don't have always the same size ...
                frame = image_for_world(run.world,
                                        bounds=bounds,
                                        background_color=background_color,
                                        rotation=rotation,
                                        **kwargs)
            else:
                raise RuntimeError("Could not make frame")
        return frame

    # sim_duration = run.time_step * run.recorded_steps
    # sim_duration = run.final_sim_time
    sim_duration = to_time - from_time

    return mpy.VideoClip(make_frame, duration=sim_duration / factor)


def record_video(path: str | pathlib.Path,
                 world: World,
                 time_step: float,
                 duration: float,
                 factor: float = 1.0,
                 fps: int = 30,
                 follow: Agent | None = None,
                 bounds: Rect | None = None,
                 **kwargs: Any) -> None:
    """
    Record a video while performing a simulation.

    :param      path:              The path where to save the video.
                                   Should have a valid video format suffix
                                   supported by ffmpeg (e.g., ``.mp4``) or ``.gif``.
    :param      world:             The world to simulate
    :param      time_step:         The time step
    :param      duration:          The simulation duration
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      follow:            Optional agent to center the view on
    :para       bounds:            Optional view area (relative id ``follow`` is specified)
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video(world,
                      time_step,
                      duration,
                      factor,
                      follow=follow,
                      bounds=bounds,
                      **kwargs)
    suffix = pathlib.Path(path).suffix
    if suffix.lower() == ".gif":
        clip.write_gif(str(path), fps=fps)
    else:
        kwargs = {'audio': False, 'logger': None}
        if MOVIEPY_VERSION == 1:
            kwargs['verbose'] = False
        clip.write_videofile(str(path), fps=fps, **kwargs)


def record_video_from_run(path: str | pathlib.Path,
                          run: RecordedExperimentalRun | ExperimentalRun,
                          factor: float = 1.0,
                          fps: int = 30,
                          follow: Agent | None = None,
                          bounds: Rect | None = None,
                          from_time: float = 0,
                          to_time: float | None = None,
                          **kwargs: Any) -> None:
    """
    Create a video from a recorded simulation.

    :param      path:              The path where to save the video.
                                   Should have a valid video format suffix
                                   supported by ffmpeg (e.g., ``.mp4``) or ``.gif``.
    :param      run:               The recorded run
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      follow:            Optional agent to center the view on
    :para       bounds:            Optional view area (relative id ``follow`` is specified)
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video_from_run(run,
                               factor,
                               follow=follow,
                               bounds=bounds,
                               from_time=from_time,
                               to_time=to_time,
                               **kwargs)
    suffix = pathlib.Path(path).suffix
    if suffix.lower() == ".gif":
        clip.write_gif(str(path), fps=fps)
    else:
        kwargs = {'audio': False, 'logger': None}
        if MOVIEPY_VERSION == 1:
            kwargs['verbose'] = False
        clip.write_videofile(str(path), fps=fps, **kwargs)


def display_video(world: World,
                  time_step: float,
                  duration: float,
                  factor: float = 1.0,
                  fps: int = 30,
                  display_width: int = 640,
                  follow: Agent | None = None,
                  bounds: Rect | None = None,
                  **kwargs: Any) -> Any:
    """
    Perform a simulation and displays the recorded video in a notebook

    :param      world:             The world to simulate
    :param      time_step:         The time step
    :param      duration:          The simulation duration
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      display_width:     The size of the video view
    :param      follow:            Optional agent to center the view on
    :para       bounds:            Optional view area (relative id ``follow`` is specified)
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video(world,
                      time_step,
                      duration,
                      factor,
                      follow=follow,
                      bounds=bounds,
                      **kwargs)
    # Renamed `ipython_display` to `display_in_notebook` in moviepy>2
    if MOVIEPY_VERSION == 1:
        return clip.ipython_display(fps=fps,
                                    rd_kwargs=dict(verbose=False, logger=None),
                                    width=display_width)
    return clip.display_in_notebook(fps=fps,
                                    rd_kwargs=dict(logger=None),
                                    width=display_width)


def display_video_from_run(run: RecordedExperimentalRun | ExperimentalRun,
                           factor: float = 1.0,
                           fps: int = 30,
                           display_width: int = 640,
                           follow: Agent | None = None,
                           bounds: Rect | None = None,
                           from_time: float = 0,
                           to_time: float | None = None,
                           **kwargs: Any) -> Any:
    """
    Displays a video created from a recorded simulation in a notebook

    :param      run:               The recorded run
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      display_width:     The size of the video view
    :param      follow:            Optional agent to center the view on
    :para       bounds:            Optional view area (relative id ``follow`` is specified)
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video_from_run(run,
                               factor,
                               follow=follow,
                               bounds=bounds,
                               from_time=from_time,
                               to_time=to_time,
                               **kwargs)
    # Renamed `ipython_display` to `display_in_notebook` in moviepy>2
    if MOVIEPY_VERSION == 1:
        return clip.ipython_display(fps=fps,
                                    rd_kwargs=dict(verbose=False, logger=None),
                                    width=display_width)
    return clip.display_in_notebook(fps=fps,
                                    rd_kwargs=dict(logger=None),
                                    width=display_width)
