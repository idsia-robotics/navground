import sys
from typing import Dict, List

import numpy as np
from navground import sim


class CheckIfMoving(sim.Probe):

    def update(self, run: sim.ExperimentalRun) -> None:
        for agent in run.world.agents:
            if agent.twist.is_almost_zero():
                print(
                    f"Agent {agent._uid} is not moving at time {run.world.time:.2f} s",
                    file=sys.stderr)


class IsMovingProbe(sim.RecordProbe):
    dtype = np.uint8

    def update(self, run: sim.ExperimentalRun) -> None:
        for agent in run.world.agents:
            self.data.push(not agent.twist.is_almost_zero())

    def get_shape(self, world: sim.World) -> List[int]:
        return [len(world.agents)]


class IsMovingSparseProbe(sim.GroupRecordProbe):

    dtype = np.float64

    def update(self, run: sim.ExperimentalRun) -> None:
        for agent in run.world.agents:
            if agent.twist.is_almost_zero():
                ds = self.get_data(str(agent._uid))
                ds.push(run.world.time)

    def get_shapes(self, world: sim.World,
                   use_uid: bool) -> Dict[str, List[int]]:
        return {
            str(a._uid if use_uid else i): []
            for i, a in enumerate(world.agents)
        }
