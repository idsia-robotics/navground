import argparse
import logging
import os
import sys

from navground import sim


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Runs an experiment using the Python interpreter')
    parser.add_argument('YAML',
                        help='YAML string, or path to a YAML file, describing an experiment',
                        type=str)
    parser.add_argument('--tqdm', help='Display tqdm bar', action='store_true')
    parser.add_argument("--run_index", help="Will overwrite the experiment own run_index if positive.", default=-1, type=int)
    parser.add_argument("--runs", help="Will overwrite the experiment own runs if positive.", default=-1, type=int)
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
    if experiment:
        if arg.run_index >= 0:
            experiment.run_index = arg.run_index
        if arg.runs >= 0:
            experiment.number_of_runs = arg.runs
        print("Performing experiment using Python ...")
        if arg.tqdm:
            from tqdm import tqdm

            with tqdm(total=experiment.number_of_runs) as bar:
                experiment.add_run_callback(lambda: bar.update(1))
                experiment.run(False)
        else:
            experiment.run(False)

        print("Experiment done")
        print(f"Duration: {experiment.duration.total_seconds(): .3f} s")
        if experiment.path:
            print(f"Saved to: {experiment.path}")
