import argparse
import asyncio
import logging
import webbrowser

import h5py

from .real_time import RealTimeReplay
from .recording import Recording
from .ui import view_path
from .ui.web_ui import WebUI


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Replay an experiment in real-time')
    parser.add_argument('path',
                        help='The path an HDF5 file that store an experiment',
                        type=str)
    parser.add_argument('--factor',
                        help='Real-time factor (set to 1.0 to run in real-time, set to higher to run faster or to lower to run slower then real-time)',
                        type=float,
                        default=1.0)
    parser.add_argument('--no-ui', help='Do not use a view', action='store_true')
    parser.add_argument('--no-browser', help='Do not open a browser view', action='store_true')
    parser.add_argument('--ui-fps', help='Maximal update rate of the view', type=float, default=25.0)
    parser.add_argument('--background-color', help='View background color', type=str, default="lightgray")
    return parser


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
    arg = parser().parse_args()
    should_open_view = not (arg.no_ui or arg.no_browser)
    if should_open_view:
        webbrowser.open(f"file://{view_path}")
    try:
        file = h5py.File(arg.path, 'r')
    except:
        logging.error(f"Could not open HDF5 file {arg.path}")
        sys.exit(1)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(
        run(with_ui=not arg.no_ui,
            file=file,
            factor=arg.factor,
            ui_fps=arg.ui_fps))
