import argparse
import logging
import os
import random
import sys
from typing import Any, Optional

from . import _Scenario, World, load_experiment, load_plugins
from .ui.video import record_video
from .ui import Decorate


def run(path: str,
        scenario: _Scenario,
        factor: float = 1.0,
        time_step: float = 0.04,
        fps: int = 24,
        duration: float = 60.0,
        seed: int = -1,
        decorate: Optional[Decorate] = None,
        **kwargs: Any) -> None:
    world = World()
    if seed < 0:
        seed = random.randint(0, 2**31)
    scenario.init_world(world, seed=seed)
    record_video(path,
                 world,
                 time_step,
                 duration,
                 factor=factor,
                 decorate=decorate, **kwargs)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Make video from an experiment using the Python interpreter'
    )
    parser.add_argument(
        'YAML',
        help='YAML string, or path to a YAML file, describing an experiment',
        type=str)
    parser.add_argument('path', help='Path where to save the video', type=str)
    parser.add_argument(
        '--seed',
        help=
        'The random seed of the simulation. If negative, it will be picked randomly',
        default=-1,
        type=int)
    parser.add_argument(
        '--factor',
        help=
        'Real-time factor (set to 1.0 to run in real-time, set to higher to run faster or to lower to run slower then real-time)',
        type=float,
        default=1.0)
    parser.add_argument('--width',
                        help='Size of the video in pixels',
                        default=640,
                        type=int)
    parser.add_argument('--fps', help='Video fps', type=int, default=24)
    parser.add_argument('--background-color',
                        help='View background color',
                        type=str,
                        default="lightgray")
    parser.add_argument('--area',
                        help='Minimal area rendered in the view',
                        type=float,
                        # default=(0.0, 0.0, 0.0, 0.0),
                        nargs=4,
                        metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"))
    parser.add_argument('--display-shape',
                        help='Display effective agent shape',
                        action='store_true')
    # parser.add_argument('--display-deadlocks',
    #                     help='Color deadlocked agent in blue',
    #                     action='store_true')
    # parser.add_argument('--display-collisions',
    #                     help='Color deadlocked agent in red',
    #                     action='store_true')
    return parser


def main(decorate: Optional[Decorate] = None) -> None:
    logging.basicConfig(level=logging.INFO)
    load_plugins()
    arg = parser().parse_args()
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.YAML

    try:
        experiment = load_experiment(yaml)
    except RuntimeError as e:
        logging.error(f"Could not load the experiment: {e}")
        sys.exit(1)
    if arg.area is not None:
        bounds = arg.area[:2], arg.area[2:]
    else:
        bounds = None
    run(path=arg.path,
        scenario=experiment.scenario,
        factor=arg.factor,
        time_step=experiment.time_step,
        duration=experiment.steps * experiment.time_step,
        fps=arg.fps,
        width=arg.width,
        background_color=arg.background_color,
        bounds=bounds,
        seed=arg.seed,
        decorate=decorate)
