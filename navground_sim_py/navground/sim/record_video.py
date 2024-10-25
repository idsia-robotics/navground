import argparse
import logging
import os
import random
import sys
from typing import TYPE_CHECKING, Any, Optional

from navground.core import command

from . import (Agent, Experiment, ExperimentalRun, RecordedExperiment,
               RecordedExperimentalRun, load_experiment)

if TYPE_CHECKING:
    from .ui import Decorate


def run(path: str,
        experiment: Experiment | RecordedExperiment,
        factor: float = 1.0,
        fps: int = 24,
        seed: int = -1,
        decorate: Optional['Decorate'] = None,
        follow_index: int = -1,
        **kwargs: Any) -> None:

    from .ui.video import record_video_from_run

    if isinstance(experiment, Experiment):
        if seed < 0:
            seed = random.randint(0, 2**31)
        experiment.record_config.pose = True
        run: ExperimentalRun | RecordedExperimentalRun = experiment.run_once(
            seed)
    else:
        if seed < 0:
            run = experiment.runs[0]
        else:
            run = experiment.runs[seed]
    if follow_index >= 0 and follow_index < len(run.world.agents):
        follow: Agent | None = run.world.agents[follow_index]
    else:
        follow = None
    record_video_from_run(path,
                          run,
                          factor=factor,
                          decorate=decorate,
                          follow=follow,
                          **kwargs)


def description() -> str:
    return 'Make video from an experiment using the Python interpreter.'


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser)
    parser.description = description()
    parser.add_argument(
        'input',
        help=('YAML string, or path to a YAML file describing an experiment,'
              ' or a path to HDF5 file of a recorded experiment'),
        type=str)
    parser.add_argument('output',
                        help='Path where to save the video',
                        type=str)
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
    parser.add_argument(
        '--area',
        help='Minimal area rendered in the view',
        type=float,
        # default=(0.0, 0.0, 0.0, 0.0),
        nargs=4,
        metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"))
    parser.add_argument('--relative_margin',
                        help='A relative margin to add around the area',
                        default='0.05',
                        type=float)
    parser.add_argument('--display-shape',
                        help='Display effective agent shape',
                        action='store_true')
    parser.add_argument('--follow',
                        help='The index of the agent to follow',
                        default=-1,
                        type=int)
    parser.add_argument('--grid',
                        help='The size of the grid',
                        default='0',
                        type=float)

    # parser.add_argument('--display-deadlocks',
    #                     help='Color deadlocked agent in blue',
    #                     action='store_true')
    # parser.add_argument('--display-collisions',
    #                     help='Color deadlocked agent in red',
    #                     action='store_true')


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def _load_recorded_experiment(path: str) -> Optional[RecordedExperiment]:
    try:
        import h5py

        file = h5py.File(path)
        return RecordedExperiment(file=file)
    except Exception:
        return None


def _load_experiment(value: str) -> Optional[Experiment]:
    if os.path.exists(value) and os.path.isfile(value):
        with open(value, 'r') as f:
            yaml = f.read()
    else:
        yaml = value
    try:
        return load_experiment(yaml)
    except RuntimeError:
        return None


def main(decorate: Optional['Decorate'] = None) -> None:
    arg = parser().parse_args()
    _main(arg, decorate=decorate)


def _main(arg: argparse.Namespace,
          decorate: Optional['Decorate'] = None) -> None:
    command._main(arg)
    logging.basicConfig(level=logging.INFO)
    experiment = _load_recorded_experiment(arg.input) or _load_experiment(
        arg.input)
    if not experiment:
        logging.error(f"Could not load the experiment from {arg.input}")
        sys.exit(1)

    if arg.area is not None:
        bounds = arg.area[:2], arg.area[2:]
    else:
        bounds = None
    run(path=arg.output,
        experiment=experiment,
        factor=arg.factor,
        fps=arg.fps,
        width=arg.width,
        background_color=arg.background_color,
        bounds=bounds,
        seed=arg.seed,
        decorate=decorate,
        follow_index=arg.follow,
        grid=arg.grid,
        relative_margin=arg.relative_margin)
