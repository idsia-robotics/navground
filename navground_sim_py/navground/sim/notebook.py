from typing import Any

from IPython.display import HTML, SVG, display_html

from . import World
from .real_time import RealTimeSimulation
from .ui import Rect, WebUI, html_for_world, svg_for_world


def notebook_view(width: int = 600, **kwargs: Any) -> HTML:
    """
    Display an empty world view in a jupyter notebook ready for a
    a :py:class:`navground.sim.ui.WebUI` to keep it up-to-date.

    :param      width:   The width in pixels
    :param      kwargs:  Additional keywords passed to :py:func:`navground.sim.ui.html_for_world`

    :returns:   A HTML view
    """
    return HTML(  # type: ignore[no-untyped-call]
        html_for_world(world=None,
                       with_websocket=True,
                       width=width,
                       notebook=True,
                       **kwargs))


def display_in_notebook(world: World, **kwargs: Any) -> SVG:
    """
    Displays a world in the notebook, either as HTML or SVG

    :param      world:             The world to display
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.svg_for_world`
    """
    return SVG(data=svg_for_world(world,  # type: ignore[no-untyped-call]
                                  **kwargs))


async def run_in_notebook(world: World,
                          time_step: float,
                          duration: float,
                          factor: float = 1.0,
                          width: int = 600,
                          port: int = 8002,
                          bounds: Rect | None = None,
                          **kwargs: Any) -> Any:
    """
    Perform a simulation and displays the recorded video in a notebook

    :param      world:             The world to simulate
    :param      time_step:         The time step
    :param      duration:          The simulation duration
    :param      factor:            The real-time factor
    :param      width:             The size of the view
    :param      port:              The websocket port
    :param      kwargs:            Arguments forwarded to :py:func:`navground.sim.ui.html_for_world`
    """

    view = notebook_view(width=width, port=port, **kwargs)
    display_html(view)  # type: ignore[no-untyped-call]
    ui = WebUI(port=port)
    await ui.prepare()
    rt = RealTimeSimulation(world=world,
                            time_step=time_step,
                            factor=factor,
                            web_ui=ui,
                            bounds=bounds)
    await rt.run()
    await ui.stop()
