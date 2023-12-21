import argparse
import logging
import os
import sys

from navground import sim


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Runs an experiment using the Python interpreter')
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
    return parser


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    # nav.load_plugins()
    sim.load_py_plugins()
    arg = parser().parse_args()
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
                if arg.processes > 1:
                    experiment.run_mp(number_of_processes=arg.processes,
                                      tqdm=bar)
                else:
                    experiment.add_run_callback(lambda _: bar.update(1))
                    experiment.run(keep=False, number_of_threads=arg.threads)
        else:
            if arg.processes > 1:
                experiment.run_mp(number_of_processes=arg.processes)
            else:
                experiment.run(keep=False, number_of_threads=arg.threads)

        print("Experiment done")
        print(f"Duration: {experiment.duration.total_seconds(): .3f} s")
        if experiment.path:
            print(f"Saved to: {experiment.path}")
