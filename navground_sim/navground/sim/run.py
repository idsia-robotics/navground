import argparse
import logging
import time

from navground import sim


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    # nav.load_plugins()
    sim.load_py_plugins()
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml',
                        help='yaml string',
                        type=str,
                        default="",
                        nargs='?')
    parser.add_argument('--input', help='yaml file', type=str, default="")
    arg = parser.parse_args()
    if arg.input:
        with open(arg.input, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.yaml
    experiment = sim.load_experiment(yaml)
    if experiment:
        # print(sim.dump(experiment))
        logging.info("Start simulation")
        start = time.time_ns()
        experiment.run()
        logging.info(f"Done in {1e-6 * (time.time_ns() - start): .1f} ms")
    else:
        logging.error("Could not load experiment")
