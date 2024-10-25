import itertools
import multiprocessing

try:
    import multiprocess  # type: ignore
except ImportError:
    multiprocess = None

import pathlib
import warnings
from queue import Empty
from typing import (TYPE_CHECKING, Callable, Dict, Iterable, List, Optional,
                    Tuple)

import numpy as np
from navground import sim

if TYPE_CHECKING:
    import tqdm

Probes = Tuple[List[Callable[[], sim.Probe]],
               Dict[str, Callable[[], sim.RecordProbe]],
               Dict[str, Callable[[], sim.GroupRecordProbe]]]

ScenarioInitCallback = Callable[['sim._navground_sim.Scenario', int], None]


def _load_and_run_experiment(
    keep: bool,
    yaml: str,
    start_index: int,
    number_of_runs: int,
    data_path: Optional[pathlib.Path],
    queue: Optional[multiprocessing.Queue] = None,
    probes: Probes = ([], {}, {}),
    scenario_init_callback: ScenarioInitCallback | None = None
) -> Dict[int, sim.ExperimentalRun]:

    experiment = sim.load_experiment(yaml)
    if not experiment:
        return {}
    experiment.save_directory = ''  # type: ignore
    experiment._probes, experiment._record_probes, experiment._group_record_probes = probes
    experiment.scenario_init_callback = scenario_init_callback
    if queue:
        experiment.add_run_callback(lambda run: queue.put(run.seed))
    experiment.run(keep,
                   number_of_runs=number_of_runs,
                   start_index=start_index,
                   data_path=data_path)
    return experiment.runs


def _divide(number: int, chunks: int) -> List[int]:
    n = number // chunks
    ns = [n] * chunks
    r = number % chunks
    for i in range(r):
        ns[i] += 1
    return ns


def run_mp(experiment: sim.Experiment,
           number_of_processes: int,
           keep: bool = False,
           number_of_runs: Optional[int] = None,
           start_index: Optional[int] = None,
           callback: Optional[Callable[[int], None]] = None,
           bar: Optional['tqdm.tqdm'] = None,
           scenario_init_callback: ScenarioInitCallback | None = None,
           use_multiprocess: bool = False) -> None:
    """

    Run an experiment distributing its runs in parallel over multiple processes.

    Use this to parallelize experiments that contains Python classes, see :py:fun:``uses_python``,
    :py:attr:`sim.Experiment.run` parallelizes over multiple threads instead and cannot
    be used to run such experiments because of the GIL.

    If ``keep=True``,  the experiment will query the runs from the different processes and hold them in memory.
    If it is configured to save the data, it will save a single HDF5 file.

    If ``keep=False``, the experiment won't keep the runs in memory. If it is configured to save the data,
    it will save one HDF5 file per process (``"data_<i>.h5"``) and one "data.h5" linking all the runs together.
    To access the data, you will need to load the HFD5 file.

    :param      experiment:             The experiment
    :param      number_of_processes:    The number of processes
    :param      keep:                   Whether to keep runs in memory
    :param      number_of_runs:         The number of runs
    :param      start_index:            The index of the first run
    :param      callback:               An optional callback to run after each run is completed
    :param      bar:                    An optional tqdm bar to display the progresses.
    :param      scenario_init_callback: An optional callback to set as :py:attr:`Experiment.scenario_init_callback`.
    :param      use_multiprocess:       Whether to use the `multiprocess` package instead of `multiprocessing`
    """
    if use_multiprocess and multiprocess:
        mp = multiprocess
    else:
        mp = multiprocessing

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

    if callback or bar is not None:
        m = mp.Manager()
        queue = m.Queue()
    else:
        queue = None

    if number_of_runs is not None:
        experiment.number_of_runs = number_of_runs
    if start_index is not None:
        experiment.run_index = start_index

    if bar is not None:
        bar.total = experiment.number_of_runs

    chunks = _divide(experiment.number_of_runs, number_of_processes)
    ss = np.cumsum(chunks)
    start_indices = np.insert(ss, 0, 0) + experiment.run_index
    if keep:
        paths: Iterable[pathlib.Path | None] = itertools.repeat(
            None, number_of_processes)
    else:
        paths = [
            experiment.path.parent /
            f"data_{i}.h5" if experiment.path else None
            for i in range(number_of_processes)
        ]
    scenario_init_callbacks = itertools.repeat(scenario_init_callback)
    yaml = itertools.repeat(sim.dump(experiment), number_of_processes)
    queues = itertools.repeat(queue, number_of_processes)
    keeps = itertools.repeat(keep, number_of_processes)
    probes = itertools.repeat((experiment._probes, experiment._record_probes,
                               experiment._group_record_probes),
                              number_of_processes)
    partial_experiments = list(
        zip(keeps, yaml, start_indices, chunks, paths, queues, probes,
            scenario_init_callbacks))

    with mp.Pool(number_of_processes) as p:
        r = p.starmap_async(_load_and_run_experiment, partial_experiments)
        if queue is not None:
            while not r.ready():
                try:
                    i = queue.get(timeout=1)
                    if callback:
                        callback(i)
                    if bar is not None:
                        bar.update(1)
                except Empty:
                    pass
        else:
            r.wait()
    for runs in r.get():
        for i, run in runs.items():
            experiment.add_run(i, run)
    path = experiment.path
    save_runs = keep and path is not None
    experiment.stop(save_runs=save_runs)
    add_links = not keep and path is not None
    if add_links:
        import h5py

        with h5py.File(path, 'a') as file:
            for (_, _, i, number_of_runs, data_path,
                 *_) in partial_experiments:
                if data_path:
                    for j in range(i, i + number_of_runs):
                        file[f'run_{j}'] = h5py.ExternalLink(
                            data_path, f"run_{j}")
