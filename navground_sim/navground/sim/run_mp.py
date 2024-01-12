import itertools
import multiprocessing as mp
import pathlib
import warnings
from queue import Empty
from typing import TYPE_CHECKING, Callable, List, Optional

import h5py
import numpy as np
from navground import sim

if TYPE_CHECKING:
    import tqdm


def _load_and_run_experiment(yaml: str,
                             start_index: int,
                             number_of_runs: int,
                             data_path: pathlib.Path,
                             queue: Optional[mp.Queue] = None) -> None:
    experiment = sim.load_experiment(yaml)
    if queue:
        experiment.add_run_callback(lambda run: queue.put(run.seed))
    experiment.run(False,
                   number_of_runs=number_of_runs,
                   start_index=start_index,
                   data_path=data_path)


def _divide(number: int, chunks: int) -> List[int]:
    n = number // chunks
    ns = [n] * chunks
    r = number % chunks
    for i in range(r):
        ns[i] += 1
    return ns


def run_mp(experiment: sim.Experiment,
           number_of_processes: int,
           number_of_runs: Optional[int] = None,
           start_index: Optional[int] = None,
           callback: Optional[Callable[[int], None]] = None,
           bar: Optional['tqdm.tqdm'] = None) -> None:
    """

    Run an experiment distributing its runs in parallel over multiple processes.

    Use this to parallelize experiments that contains Python classes, see :py:fun:``uses_python``,
    :py:attr:`sim.Experiment.run` parallelizes over multiple threads instead and cannot
    be used to run such experiments because of the GIL.

    The experiment will save the data to :py:attr:`sim.Experiment.path`
    without loading it in memory. To access the data, you will need
    to load the HFD5 file.

    :param      experiment:           The experiment
    :param      number_of_processes:  The number of processes
    :param      number_of_runs:       The number of runs
    :param      start_index:          The index of the first run
    :param      callback:             An optional callback to run after each run is completed
    :param      bar:                  An optional tqdm bar to display the progresses.

    """

    if number_of_processes < 1:
        warnings.warn(
            f'Negative number of processes {number_of_processes} ... will not run the experiment'
        )
        return

    if number_of_processes > mp.cpu_count():
        warnings.warn(
            f'More processes {number_of_processes} than number of cores {mp.cpu_count()}'
        )
    experiment.start()

    if callback or bar:
        m = mp.Manager()
        queue = m.Queue()
    else:
        queue = None

    if number_of_runs is not None:
        experiment.number_of_runs = number_of_runs
    if start_index is not None:
        experiment.run_index = start_index

    if bar:
        bar.total = experiment.number_of_runs

    chunks = _divide(experiment.number_of_runs, number_of_processes)
    ss = np.cumsum(chunks)
    start_indices = np.insert(ss, 0, 0) + experiment.run_index
    paths = [
        experiment.path.parent / f"data_{i}.h5"
        for i in range(number_of_processes)
    ]
    yaml = itertools.repeat(sim.dump(experiment), number_of_processes)
    queues = itertools.repeat(queue, number_of_processes)
    partial_experiments = list(zip(yaml, start_indices, chunks, paths, queues))

    with mp.Pool(number_of_processes) as p:
        r = p.starmap_async(_load_and_run_experiment, partial_experiments)
        if queue is not None:
            while not r.ready():
                try:
                    i = queue.get(timeout=1)
                    if callback:
                        callback(i)
                    if bar:
                        bar.update(1)
                except Empty:
                    pass
        else:
            r.wait()
    experiment.stop()
    path = experiment.path
    with h5py.File(path, 'a') as file:
        for (_, i, number_of_runs, data_path, _) in partial_experiments:
            for j in range(i, i + number_of_runs):
                file[f'run_{j}'] = h5py.ExternalLink(data_path, f"run_{j}")
