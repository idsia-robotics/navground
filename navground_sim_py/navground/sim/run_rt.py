import argparse
import asyncio
import logging
import os
import pathlib
import random
import sys
from typing import TYPE_CHECKING, Optional

from navground.core import command

from . import Experiment, World, _Scenario, load_experiment, load_plugins
from .real_time import RealTimeSimulation

if TYPE_CHECKING:
    from .ui import Decorate
    from .ui.web_ui import Rect


def until_done(world: World, max_duration: float = -1):

    def f():
        done = [a.task.done() for a in world.agents if a.task]
        return ((done and all(done))
                or (max_duration > 0 and world.time > max_duration))

    return f


async def run(scenario: _Scenario,
              with_ui: bool = True,
              factor: float = 1.0,
              time_step: float = 0.04,
              ui_fps: float = 25.0,
              max_duration: float = -1,
              port: int = 8000,
              background_color: str = 'lightgray',
              bounds: Optional['Rect'] = None,
              display_deadlocks: bool = False,
              display_collisions: bool = False,
              seed: int = -1,
              decorate: Optional['Decorate'] = None) -> None:
    world = World()
    if seed < 0:
        seed = random.randint(0, 2**31)
    scenario.init_world(world, seed=seed)
    world.run(1, 0.0)
    if with_ui:

        from .ui.web_ui import WebUI

        web_ui = WebUI(host='127.0.0.1',
                       port=port,
                       max_rate=ui_fps,
                       display_deadlocks=display_deadlocks,
                       display_collisions=display_collisions,
                       decorate=decorate)
        await web_ui.prepare()
        logging.info("Waiting for a client")
        while web_ui.number_of_client == 0:
            await asyncio.sleep(1.0)
        await web_ui.set_background_color(background_color)
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


def description() -> str:
    return 'Runs an experiment using the Python interpreter in real-time.'


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser)
    parser.description = description()
    parser.add_argument(
        'YAML',
        help='YAML string, or path to a YAML file, describing an experiment',
        type=str)
    parser.add_argument(
        '--factor',
        help=
        'Real-time factor (set to 1.0 to run in real-time, set to higher to run faster or to lower to run slower then real-time)',
        type=float,
        default=1.0)
    parser.add_argument('--no-ui',
                        help='Do not use a view',
                        action='store_true')
    parser.add_argument(
        '--seed',
        help=
        'The random seed of the simulation. If negative, it will be picked randomly',
        default=-1,
        type=int)
    parser.add_argument('--port', help='The websocket port', default=8000)
    parser.add_argument('--width',
                        help='Size of the view in pixels',
                        default=500,
                        type=int)
    parser.add_argument('--no-browser',
                        help='Do not open a browser view',
                        action='store_true')
    parser.add_argument('--ui-fps',
                        help='Maximal update rate of the view',
                        type=float,
                        default=25.0)
    parser.add_argument('--background-color',
                        help='View background color',
                        type=str,
                        default="lightgray")
    parser.add_argument(
        '--area',
        help='Minimal area rendered in the view',
        type=float,
        # default=(0.0, 0.0, 0.0, 0.0),
        nargs=4,
        metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"))
    parser.add_argument('--display-shape',
                        help='Display effective agent shape',
                        action='store_true')
    parser.add_argument('--display-deadlocks',
                        help='Color deadlocked agent in blue',
                        action='store_true')
    parser.add_argument('--display-collisions',
                        help='Color deadlocked agent in red',
                        action='store_true')
    parser.add_argument(
        '--html',
        help=
        'Where to save the HTML file. Leave empty to use a temporary directory',
        default='')


def _main(arg: argparse.Namespace,
          decorate: Optional['Decorate'] = None) -> None:
    command._main(arg)
    logging.basicConfig(level=logging.INFO)
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.YAML
    should_open_view = not (arg.no_ui or arg.no_browser)

    if should_open_view:
        if arg.html:
            path = pathlib.Path(arg.html)
        else:
            path = None

        from .ui import open_html

        open_html(width=arg.width,
                  port=arg.port,
                  display_shape=arg.display_shape,
                  path=path)
    experiment: Experiment | None = None
    try:
        experiment = load_experiment(yaml)
    except RuntimeError as e:
        logging.error(f"Error while parsing: {e}")
    if not experiment:
        logging.error(f"Could not load the experiment")
        sys.exit(1)
    loop = asyncio.get_event_loop()
    if arg.area is not None:
        bounds = arg.area[:2], arg.area[2:]
    else:
        bounds = None
    loop.run_until_complete(
        run(with_ui=not arg.no_ui,
            scenario=experiment.scenario,
            factor=arg.factor,
            time_step=experiment.time_step,
            max_duration=experiment.steps * experiment.time_step,
            ui_fps=arg.ui_fps,
            port=arg.port,
            background_color=arg.background_color,
            bounds=bounds,
            display_deadlocks=arg.display_deadlocks,
            display_collisions=arg.display_collisions,
            seed=arg.seed,
            decorate=decorate))


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
