from typing import Any

from . import World

from .ui.web_ui import Rect, WebUI
from .ui.to_html import notebook_view
from IPython.display import display_html
from .real_time import RealTimeSimulation


async def display_in_notebook(world: World,
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
    display_html(view)
    ui = WebUI(port=port)
    await ui.prepare()
    rt = RealTimeSimulation(world=world,
                            time_step=time_step,
                            factor=factor,
                            web_ui=ui,
                            bounds=bounds)
    await rt.run()
    await ui.stop()
