import argparse
import time
import sys

import numpy as np
from navground import core


def go_to(controller: core.Controller, target: np.ndarray) -> None:
    action = controller.go_to_position(target, 0.2)

    def done_cb(state):
        if state == core.Action.State.success:
            go_to(controller, -target)

    action.done_cb = done_cb


def run(behavior_name: str = "HL") -> None:
    dt = 0.02
    controllers = []
    agents = []
    target = np.array((1.0, 0.0))
    obstacles = [core.Disc((0.0, 0.0), 0.1)]
    for p in ((0.5, 0.0), (-0.5, 0.5)):
        behavior = core.Behavior.make_type(behavior_name)
        if not behavior:
            print(f"Could not create behavior of type {behavior_name}",
                  file=sys.stderr)
            exit(1)
        behavior.kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(
            0.166, 0.094)
        behavior.radius = 0.08
        behavior.horizon = 1.0
        behavior.safety_margin = 0.02
        behavior.optimal_speed = 0.12
        if isinstance(behavior.environment_state, core.GeometricState):
            behavior.environment_state.static_obstacles = obstacles
        controller = core.Controller(behavior)
        controller.speed_tolerance = 0.01
        behavior.position = np.asarray(p)
        controllers.append(controller)
        agents.append(behavior)
        go_to(controller, target)
    print('Start simulating 1 minute at 50 ticks per second')
    a = time.time()
    for _ in range(50 * 60):
        for controller in controllers:
            this = controller.behavior
            if isinstance(this.environment_state, core.GeometricState):
                this.environment_state.neighbors = [
                    core.Neighbor(agent.position, agent.radius, agent.velocity,
                                  0) for agent in agents if agent != this
                ]
        for controller in controllers:
            cmd = controller.update(dt)
            controller.behavior.actuate(cmd, dt)
            # if controller.idle:
            #     go_to(controller, -controller.behavior.target.position)

    print(f'Done simulating in {1000 * (time.time() - a):.1f} ms')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    arg = parser.parse_args()
    run(arg.behavior)
