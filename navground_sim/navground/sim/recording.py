from typing import Iterator

import h5py
import numpy as np
from navground.core import Pose2

from . import World, load_experiment, load_world


class Run:

    def __init__(self,
                 world: World,
                 recorded_poses: np.ndarray,
                 time_step: float,
                 index: int = -1):
        self.world = world
        self.recorded_poses = recorded_poses
        self.time_step = time_step
        self._max_step = len(self.recorded_poses) - 1
        self._step = 0
        self.index = index
        p1 = np.min(self.recorded_poses, axis=(0, 1))[:2]
        p2 = np.max(self.recorded_poses, axis=(0, 1))[:2]
        self.bounds = (p1, p2)

    def do_step(self):
        if self._step < self._max_step:
            # self.world.time += self.time_step
            for ps, agent in zip(self.recorded_poses[self._step],
                                 self.world.agents):
                agent.pose = Pose2(ps[:2], ps[2])
            self._step += 1
            return True
        return False

    @property
    def done(self) -> bool:
        return self._step == self._max_step


class Recording:

    def __init__(self, file: h5py.File):
        self.experiment = load_experiment(file.attrs['experiment'])
        self.file = file
        self._length = 0
        while self._length in self:
            self._length += 1

    def __contains__(self, index: int) -> bool:
        return f'run_{index}' in self.file

    def get_run(self, index: int) -> Run:
        run = self.file[f'run_{index}']
        world = load_world(run.attrs['world'])
        return Run(world=world,
                   recorded_poses=run['poses'],
                   time_step=self.experiment.time_step,
                   index=index)

    def __len__(self) -> int:
        return self._length

    def __iter__(self) -> Iterator[Run]:
        index = 0
        while index in self:
            yield self.get_run(index)
            index += 1
