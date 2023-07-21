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
        experiment.run()
        print("Experiment done using Python")
        print(f"Duration: {experiment.duration.total_seconds(): .3f} s")
        if experiment.path:
           print(f"Saved to: {experiment.path}")
