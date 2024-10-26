import argparse
import logging
import os
import sys

from navground import sim
from navground.core import command

def description() -> str:
    return 'Runs an experiment using the Python interpreter'


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser)
    parser.description = description()
    parser.add_argument(
        'YAML',
        help='YAML string, or path to a YAML file, describing an experiment',
        type=str)
    parser.add_argument('--tqdm', help='Display tqdm bar', action='store_true')
    parser.add_argument(
        "--run_index",
        help="Will overwrite the experiment own run_index if positive.",
        default=-1,
        type=int)
    parser.add_argument(
        "--runs",
        help="Will overwrite the experiment own runs if positive.",
        default=-1,
        type=int)
    parser.add_argument("--threads",
                        help="Number of threads",
                        default=1,
                        type=int)
    parser.add_argument("--processes",
                        help="Number of processes",
                        default=1,
                        type=int)
    parser.add_argument(
        "--save_single_hdf5",
        help=
        "Whether to store a single HDF5 file when using multiple processes",
        action='store_true')
    parser.add_argument(
        "--use_multiprocess",
        help=
        "Whether to use the multiprocess package instead of multiprocessings",
        action='store_true')


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    logging.basicConfig(level=logging.INFO)
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.YAML
    try:
        experiment = sim.load_experiment(yaml)
    except RuntimeError as e:
        logging.error(f"Could not load the experiment: {e}")
        sys.exit(1)
    if arg.threads != 1 and arg.processes != 1:
        logging.warning(
            "Specify only either --threads or --processes. Will ignore --threads"
        )
        arg.threads = 1

    if experiment:
        if arg.run_index >= 0:
            experiment.run_index = arg.run_index
        if arg.runs >= 0:
            experiment.number_of_runs = arg.runs
        if arg.threads > 1 and sim.uses_python(experiment):
            print("This experiment requires Python "
                  "to run and cannot be parallelized over threads")
            arg.threads = 1
        print("Performing experiment using Python ...")
        if arg.tqdm:
            from tqdm import tqdm

            with tqdm(total=experiment.number_of_runs) as bar:
                def cb(_: sim.ExperimentalRun) -> None:
                    bar.update(1)

                if arg.processes > 1:
                    experiment.run_mp(number_of_processes=arg.processes,  # type: ignore
                                      bar=bar,
                                      keep=arg.save_single_hdf5,
                                      use_multiprocess=arg.use_multiprocess)
                else:
                    experiment.add_run_callback(cb)
                    experiment.run(keep=False, number_of_threads=arg.threads)
        else:
            if arg.processes > 1:
                experiment.run_mp(number_of_processes=arg.processes,  # type: ignore
                                  keep=arg.save_single_hdf5,
                                  use_multiprocess=arg.use_multiprocess)
            else:
                experiment.run(keep=False, number_of_threads=arg.threads)

        print("Experiment done")
        print(f"Duration: {experiment.duration.total_seconds(): .3f} s")
        if experiment.path:
            print(f"Saved to: {experiment.path}")


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
