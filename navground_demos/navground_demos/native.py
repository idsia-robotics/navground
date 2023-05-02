import argparse
import time

from navground import core
import numpy as np


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
        behavior.kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(0.166, 0.094)
        behavior.radius = 0.08
        behavior.horizon = 1.0
        behavior.safety_margin = 0.02
        behavior.optimal_speed = 0.12
        try:
            behavior.environment_state.static_obstacles = obstacles
        except AttributeError:
            pass
        controller = core.Controller(behavior)
        controller.speed_tolerance = 0.01
        behavior.position = p
        controllers.append(controller)
        agents.append(behavior)
        go_to(controller, target)
    print('Start simulating 1 minute at 50 ticks per second')
    a = time.time()
    for _ in range(50 * 60):
        for controller in controllers:
            this = controller.behavior
            try:
                controller.behavior.environment_state.neighbors = [
                    core.Neighbor(
                            agent.position, agent.radius, agent.velocity, 0)
                    for agent in agents if agent != this]
            except AttributeError:
                pass
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
