import numpy
from navground import core, sim


class MySensor(sim.PySensor):

    # MUST override
    # return the description of the data written by the sensor
    def get_description(self) -> dict[str, core.BufferDescription]:
        return {'my_field': core.BufferDescription(...)}

    # CAN override
    # executed at the start of the simulation
    # def prepare(self, agent: sim.Agent, world: sim.World) -> None: ...

    # CAN override
    # executed during the the simulation, should update the state
    # update the environment state according to the agent and world
    def update(self, agent: sim.Agent, world: sim.World,
               state: core.EnvironmentState) -> None:
        if isinstance(state, core.SensingState):
            # compute the data
            my_data = ...
            # update the sensing state
            state.set_buffer(key="my_field",
                             buffer=core.Buffer(data=numpy.asarray(my_data)))
