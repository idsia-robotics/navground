import pathlib
from typing import Any, Union

import cairosvg
import moviepy.editor as mpy
import numpy as np

from .. import RecordedExperimentalRun, World
from .to_svg import svg_for_world


def _surface_to_npim(surface):
    """ Transforms a Cairo surface into a numpy array. """
    im = +np.frombuffer(surface.get_data(), np.uint8)
    H, W = surface.get_height(), surface.get_width()
    im.shape = (H, W, 4)  # for RGBA
    return im[:, :, 2::-1]


def _svg_to_npim(svg_bytestring, dpi=96, background_color="snow"):
    """ Renders a svg bytestring as a RGB image in a numpy array """
    tree = cairosvg.parser.Tree(bytestring=svg_bytestring)
    surf = cairosvg.surface.PNGSurface(tree,
                                       None,
                                       dpi,
                                       background_color=background_color).cairo
    return _surface_to_npim(surf)


def make_video(world: World,
               time_step: float,
               duration: float,
               factor: float = 1.0,
               background_color: str = "snow",
               **kwargs: Any) -> mpy.VideoClip:
    t0 = world.time

    def make_frame(t: float) -> np.ndarray:
        t = t * factor
        while world.time - t0 + time_step < t:
            world.update(time_step)
        dt = t - world.time - t0
        if dt > 0:
            world.update(dt)
        svg_data = svg_for_world(world, **kwargs)
        # TODO(Jerome): they don't have always the same size ...
        r = _svg_to_npim(svg_data, background_color=background_color)
        return r

    return mpy.VideoClip(make_frame, duration=duration / factor)


def make_video_from_run(run: RecordedExperimentalRun,
                        factor: float = 1.0,
                        background_color: str = "snow",
                        **kwargs: Any) -> mpy.VideoClip:

    frame = None
    bounds = run.bounds

    def make_frame(t: float) -> np.ndarray:
        nonlocal frame
        t = t * factor
        new_step = int(t // run.time_step)
        if new_step != run._step or frame is None:
            if run.go_to_step(new_step):
                svg_data = svg_for_world(run.world, bounds=bounds, **kwargs)
                # TODO(Jerome): they don't have always the same size ...
                frame = _svg_to_npim(svg_data,
                                     background_color=background_color)
        return frame

    return mpy.VideoClip(make_frame, duration=run._final_sim_time / factor)


def record_video(path: Union[str, pathlib.Path],
                 world: World,
                 time_step: float,
                 duration: float,
                 factor: float = 1.0,
                 fps: int = 30,
                 **kwargs: Any):
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
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video(world, time_step, duration, factor, **kwargs)
    suffix = pathlib.Path(path).suffix
    if suffix.lower() == ".gif":
        clip.write_gif(str(path), fps=fps)
    else:
        clip.write_videofile(str(path), fps=fps, audio=False)


def record_video_from_run(path: Union[str, pathlib.Path],
                          run: RecordedExperimentalRun,
                          factor: float = 1.0,
                          fps: int = 30,
                          **kwargs: Any):
    """
    Create a video from a recorded simulation.

    :param      path:              The path where to save the video.
                                   Should have a valid video format suffix
                                   supported by ffmpeg (e.g., ``.mp4``) or ``.gif``.
    :param      run:               The recorded run
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video_from_run(run, factor, **kwargs)
    suffix = pathlib.Path(path).suffix
    if suffix.lower() == ".gif":
        clip.write_gif(str(path), fps=fps)
    else:
        clip.write_videofile(str(path), fps=fps, audio=False)


def display_video(world: World,
                  time_step: float,
                  duration: float,
                  factor: float = 1.0,
                  fps: int = 30,
                  display_width: int = 640,
                  **kwargs: Any) -> Any:
    """
    Perform a simulation and displays the recorded video in a notebook

    :param      world:             The world to simulate
    :param      time_step:         The time step
    :param      duration:          The simulation duration
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      display_width:     The size of the video view
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video(world, time_step, duration, factor, **kwargs)
    return clip.ipython_display(fps=fps, width=display_width)


def display_video_from_run(run: RecordedExperimentalRun,
                           factor: float = 1.0,
                           fps: int = 30,
                           display_width: int = 640,
                           **kwargs: Any) -> Any:
    """
    Displays a video created from a recorded simulation in a notebook

    :param      run:               The recorded run
    :param      factor:            The real-time factor
    :param      fps:               The video fps
    :param      display_width:     The size of the video view
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    clip = make_video_from_run(run, factor, **kwargs)
    return clip.ipython_display(fps=fps, width=display_width)
