import argparse
import asyncio
import logging

import h5py

from .real_time import RealTimeReplay
from .recording import Recording
from .ui.web_ui import WebUI


async def run(file: h5py.File,
              with_ui: bool = True,
              factor: float = 1.0,
              ui_fps: float = 25.0,
              background: str = 'lightgray') -> None:
    if with_ui:
        web_ui = WebUI(host='127.0.0.1', max_rate=ui_fps)
        await web_ui.prepare()
        logging.info("Waiting for a client")
        while web_ui.number_of_client == 0:
            await asyncio.sleep(1.0)
        await web_ui.set_background_color(background)
    else:
        web_ui = None
    recording = Recording(file)
    for run in recording:
        rt_sim = RealTimeReplay(run=run, factor=factor, web_ui=web_ui)
        logging.info(f"Start run {run.index}")
        await rt_sim.run()
        sleep_percent = 100 * rt_sim.slept / rt_sim.total
        logging.info(f"Done. Total time: {rt_sim.total:.3f} s: "
                     f"sleep {sleep_percent:.1f}% | "
                     f"simulation {100 - sleep_percent:.1f}%")
    if web_ui:
        await web_ui.stop()


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='h5df file', type=str)
    parser.add_argument('--factor',
                        help='Real time factor',
                        type=float,
                        default=1.0)
    parser.add_argument('--ui-fps', help='Max fps', type=float, default=25.0)
    parser.add_argument('--no-ui', help='UI', action='store_true')
    parser.add_argument('--background', help='background color', type=str, default="lightgray")
    arg = parser.parse_args()
    file = h5py.File(arg.input, 'r')
    loop = asyncio.get_event_loop()
    loop.run_until_complete(
        run(with_ui=not arg.no_ui,
            file=file,
            factor=arg.factor,
            ui_fps=arg.ui_fps))
