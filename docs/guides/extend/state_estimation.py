from navground import core, sim


class MyStateEstimation(sim.PyMyStateEstimation):

    # CAN override
    # executed at the start of the simulation
    # def prepare(self, agent: sim.Agent, world: sim.World) -> None: ...

    # CAN override
    # executed at the end of the simulation
    # def close(self) -> None: ...

    # CAN override
    # executed during the the simulation, should update the state
    # update the environment state according to the agent and world
    def update(self, agent: sim.Agent, world: sim.World,
               state: core.EnvironmentState) -> None:
        if isinstance(state, SupportedEnvironmentState):
            # update the state
            pass
