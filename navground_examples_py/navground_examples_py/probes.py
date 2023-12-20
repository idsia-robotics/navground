from typing import Tuple

import numpy as np
from navground import sim


class IsMovingProbe(sim.Probe):

    def __init__(self):
        super().__init__()
        self.type = np.int8

    def prepare(self, world: sim.World, maximal_steps: int) -> None:
        pass

    def update(self, world: sim.World) -> None:
        self.record_step()
        for agent in world.agents:
            self.push(agent.twist.is_almost_zero())

    def shape(self) -> Tuple[int, ...]:
        return (self.steps, self.size // self.steps if self.steps else 0)

    def finalize(self, world: sim.World, steps: int) -> None:
        pass


class IsMovingSparseProbe(sim.MapProbe):

    def __init__(self):
        super().__init__()
        self.type = np.float64

    def prepare(self, world: sim.World, maximal_steps: int) -> None:
        pass

    def update(self, world: sim.World) -> None:
        for agent in world.agents:
            if agent.twist.is_almost_zero():
                self.push(agent._uid, world.time)

    def shape(self, key: int) -> Tuple[int, ...]:
        return (self.size(key), )

    def finalize(self, world: sim.World, steps: int) -> None:
        pass
