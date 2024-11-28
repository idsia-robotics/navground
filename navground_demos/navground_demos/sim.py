import argparse
import time
import pathlib as pl

from navground import core, sim


class ThymioDemo(sim.Scenario, name="PyThymioDemo"):  # type: ignore[call-arg]

    def __init__(self, behavior_type: str = "HL"):
        super().__init__()
        self._behavior_type = behavior_type

    def init_world(self, world: sim.World, seed: int | None = None) -> None:
        targets: list[core.Vector2Like] = [(1, 0), (-1, 0)]
        for i in range(2):
            task = sim.tasks.WaypointsTask(targets, True, 0.2)
            se = sim.state_estimations.BoundedStateEstimation(1.0)
            kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(
                0.166, 0.094)
            behavior = core.Behavior.make_type(self.behavior_type)
            agent = sim.Agent(0.08, behavior, kinematics, task, se, 0.02)
            if agent.behavior:
                agent.behavior.optimal_speed = 0.12
                agent.behavior.horizon = 1.0
                agent.behavior.safety_margin = 0.02
            agent.controller.speed_tolerance = 0.01
            agent.pose = core.Pose2((-0.5 if i else 0.5, 0.5), 0.0)
            agent.type = "thymio"
            world.add_agent(agent)
        world.add_obstacle(core.Disc((0.0, 0.0), 0.1))

    @property
    @sim.register("HL", "Behavior name")
    def behavior_type(self) -> str:
        return self._behavior_type

    @behavior_type.setter
    def behavior_type(self, value: str) -> None:
        self._behavior_type = value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--behavior", help="", type=str, default="HL")
    arg = parser.parse_args()
    demo = sim.Experiment(0.02, 50 * 60)
    demo.scenario = ThymioDemo(arg.behavior)
    demo.save_directory = pl.Path(".")
    demo.record_config.pose = True
    demo.name = "PyThymioDemo"
    print("Start simulating 1 minute at 50 ticks per second")
    begin = time.time()
    demo.run()
    end = time.time()
    ms = (end - begin) * 1e3
    print(f"Done simulating in {ms:.1f} ms")
