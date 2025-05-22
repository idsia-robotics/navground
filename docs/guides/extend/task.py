from navground import sim


class MyTask(sim.Task):

    # Set the log size to a constant value
    LOG_SIZE = 0

    # CAN override
    # executed at the start of the simulation
    # def prepare(self, agent: sim.Agent, world: sim.World) -> None: ...

    # CAN override
    # executed at the end of the simulation
    # def close(self) -> None: ...

    # CAN override
    # executed during the the simulation, should update the state
    def update(self, agent: sim.Agent, world: sim.World, time: float) -> None:
        ...

    # CAN override
    # return if we are done or not
    def done(self) -> bool:
        ...

    # CAN override
    # return the size of the data with log for each event
    # do not override or return 0 if you are not logging anything
    # should return a constant
    def get_log_size(self) -> int:
        return self.LOG_SIZE
