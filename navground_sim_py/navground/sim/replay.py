import argparse
import asyncio
import logging
import pathlib
import sys
from typing import TYPE_CHECKING, Optional, Union

from .real_time import RealTimeSimulation
from .recorded_experiment import RecordedExperiment, RecordedExperimentalRun

if TYPE_CHECKING:
    from .ui.web_ui import Rect, WebUI
    import h5py  # type: ignore


class RealTimeReplay(RealTimeSimulation):
    """
    Helper class to replay an :py:class:`navground.sim.RecordedExperimentalRun`
    in real-time.

    >>> # load a run from a recorded experiment
    >>> run = ...
    >>> sim = RealTimeSimulation(run=run, web_ui=WebUI())
    >>> await sim.run()

    """

    def __init__(self,
                 run: RecordedExperimentalRun,
                 factor: float = 1.0,
                 web_ui: Optional['WebUI'] = None,
                 bounds: Optional['Rect'] = None):
        """
        Constructs a new instance.

        :param      run:     The recorded run
        :param      factor:  The real time factor
        :param      web_ui:  An optional web user interface to sync with
        :param      bounds:     The region to display in the web_ui.
                                If not set, it will compute it from
                                the recorded poses.

        The run should contain at least recorded poses, else it would
        be useless to replay it.
        """
        self._run = run
        assert run.bounds is not None, "Will not replay a run without recorded poses"
        super().__init__(world=run.world,
                         time_step=run.time_step,
                         web_ui=web_ui,
                         factor=factor,
                         bounds=bounds or run.bounds)

    def _step(self) -> bool:
        return self._run.do_step()


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def description() -> str:
    return 'Replay an experiment in real-time.'

def init_parser(parser: argparse.ArgumentParser) -> None:
    parser.description = description()
    parser.add_argument('path',
                        help='The path an HDF5 file that store an experiment',
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
        '--record_video',
        help=
        'The path to a folder where to save videos instead of replaying in an UI. If left empty, it will not create videos.',
        type=str,
        default='')
    parser.add_argument('--no-browser',
                        help='Do not open a browser view',
                        action='store_true')
    parser.add_argument('--ui-fps',
                        help='Maximal update rate of the view',
                        type=float,
                        default=25.0)
    parser.add_argument(
        '--seed',
        help=
        'The seed of the run to replay. If negative or not specified, will replay all runs',
        type=int,
        default=-1)
    parser.add_argument('--background-color',
                        help='View background color',
                        type=str,
                        default="lightgray")
    parser.add_argument(
        '--html',
        help=
        'Where to save the HTML file. Leave empty to use a temporary directory',
        default='')


async def run(file: 'h5py.File',
              with_ui: bool = True,
              factor: float = 1.0,
              ui_fps: float = 25.0,
              background_color: str = 'snow',
              seed: int = -1,
              video: Union[str, pathlib.Path] = '') -> None:
    if with_ui:

        from .ui.web_ui import WebUI

        web_ui = WebUI(host='127.0.0.1', max_rate=ui_fps)
        await web_ui.prepare()
        logging.info("Waiting for a client")
        while web_ui.number_of_client == 0:
            await asyncio.sleep(1.0)
        await web_ui.set_background_color(background_color)
    else:
        web_ui = None
    recording = RecordedExperiment(file=file)
    for run_seed, run in recording.runs.items():
        if seed >= 0 and run_seed != seed:
            # logging.info(f"Skip run {run.seed}")
            continue
        if run.poses is None or len(run.poses) == 0:
            logging.warning(f"Run {run.seed} has no poses")
            continue
        if video:

            from .ui.video import record_video_from_run

            path = pathlib.Path(video) / f"run_{run.seed}.mp4"
            record_video_from_run(path,
                                  run,
                                  factor=factor,
                                  background_color=background_color)
        else:
            rt_sim = RealTimeReplay(run=run, factor=factor, web_ui=web_ui)
            logging.info(f"Start run {run.seed}")
            await rt_sim.run()
            sleep_percent = 100 * rt_sim.slept / rt_sim.total
            logging.info(f"Done. Total time: {rt_sim.total:.3f} s: "
                         f"sleep {sleep_percent:.1f}% | "
                         f"simulation {100 - sleep_percent:.1f}%")
    if web_ui:
        await web_ui.stop()


def _main(arg: argparse.Namespace) -> None:
    logging.basicConfig(level=logging.INFO)
    should_open_view = not (arg.no_ui or arg.no_browser or arg.record_video)
    if should_open_view:
        if arg.html:
            path = pathlib.Path(arg.html)
        else:
            path = None

        from .ui import open_html

        open_html(path=path)
    try:
        import h5py

        file = h5py.File(arg.path, 'r')
    except:
        logging.error(f"Could not open HDF5 file {arg.path}")
        sys.exit(1)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(
        run(with_ui=not (arg.no_ui or arg.record_video),
            file=file,
            factor=arg.factor,
            ui_fps=arg.ui_fps,
            seed=arg.seed,
            video=arg.record_video,
            background_color=arg.background_color))


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
