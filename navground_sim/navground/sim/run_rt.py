import argparse
import asyncio
import logging
import os
import sys
import webbrowser
from pathlib import Path
from typing import Optional

from . import Scenario, World, load_experiment, load_py_plugins
from .real_time import RealTimeSimulation
from .ui.web_ui import Rect, WebUI
from .ui import view_path


def until_done(world: World, max_duration: float = -1):

    def f():
        done = [a.task.done() for a in world.agents if a.task]
        return ((done and all(done))
                 or (max_duration > 0 and world.time > max_duration))

    return f


async def run(scenario: Scenario,
              with_ui: bool = True,
              factor: float = 1.0,
              time_step: float = 0.04,
              ui_fps: float = 25.0,
              max_duration: float = -1,
              background: str = 'lightgray',
              bounds: Optional[Rect] = None) -> None:
    world = World()
    scenario.init_world(world)
    world.run(1, 0.0)
    if with_ui:
        web_ui = WebUI(host='127.0.0.1', max_rate=ui_fps)
        await web_ui.prepare()
        logging.info("Waiting for a client")
        while web_ui.number_of_client == 0:
            await asyncio.sleep(1.0)
        await web_ui.set_background_color(background)
    else:
        web_ui = None
    rt_sim = RealTimeSimulation(world,
                                time_step=time_step,
                                factor=factor,
                                web_ui=web_ui,
                                bounds=bounds)
    logging.info(f"Run simulation for {max_duration:.1f} s")
    await rt_sim.run(until=until_done(world, max_duration=max_duration))
    sleep_percent = 100 * rt_sim.slept / rt_sim.total
    logging.info(f"Simulation done. Total real time: {rt_sim.total:.3f} s: "
                 f"sleep {sleep_percent:.1f}% | "
                 f"simulation {100 - sleep_percent:.1f}%")
    if web_ui:
        await web_ui.stop()


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Runs an experiment using the Python interpreter in real-time')
    parser.add_argument('YAML',
                        help='YAML string, or path to a YAML file, describing an experiment',
                        type=str)
    parser.add_argument('--factor',
                        help='Real-time factor (set to 1.0 to run in real-time, set to higher to run faster or to lower to run slower then real-time)',
                        type=float,
                        default=1.0)
    parser.add_argument('--no-ui', help='Do not use a view', action='store_true')
    parser.add_argument('--no-browser', help='Do not open a browser view', action='store_true')
    parser.add_argument('--ui-fps', help='Maximal update rate of the view', type=float, default=25.0)
    parser.add_argument('--background-color', help='View background color', type=str, default="lightgray")
    parser.add_argument('--area', help='Minimal area rendered in the view', type=float,
                        default=(0.0, 0.0, 0.0, 0.0), nargs=4,
                        metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"))
    return parser


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    load_py_plugins()
    arg = parser().parse_args()
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.YAML
    should_open_view = not (arg.no_ui or arg.no_browser)
    if should_open_view:
        webbrowser.open(f"file://{view_path}")
    try:
        experiment = load_experiment(yaml)
    except RuntimeError as e:
        logging.error(f"Could not load the experiment: {e}")
        sys.exit(1)
    loop = asyncio.get_event_loop()
    bounds = arg.area[:2], arg.area[2:]
    loop.run_until_complete(
        run(with_ui=not arg.no_ui,
            scenario=experiment.scenario,
            factor=arg.factor,
            time_step=experiment.time_step,
            max_duration=experiment.steps * experiment.time_step,
            ui_fps=arg.ui_fps,
            background=arg.background_color,
            bounds=bounds))
