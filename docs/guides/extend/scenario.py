from typing import SupportsInt

from navground import sim


class MyScenario(sim.Scenario):

    # SHOULD override
    # executed during the the simulation, should update the state
    # update the environment state according to the agent and world
    def init_world(self, world: sim.World, seed: SupportsInt | None = None) -> None:
        # call the super class: when the scenario is configured thought YAML,
        # it will add agents and obstacles as specified in the configuration.
        super().init_world(world, seed)
        # use the world random generator to sample random variable.
        rng = world.random_generator
        # manipulate the world: create/add/modify/delete agents and obstacles
