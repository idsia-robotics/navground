import argparse  # noqa: D100
import sys

import numpy
from navground import core


def main(behavior_name: str = 'HL') -> None:
    """Test obstacle avoidance behavior for a few iterations

    Args:
        behavior_name (str, optional): The name of the behavior
    """
    behavior = core.Behavior.make_type(behavior_name)
    if not behavior:
        print(f'No behavior with name {behavior_name}')
        sys.exit(1)
    print(f'Use behavior {behavior_name}')
    behavior.kinematics = core.kinematics.OmnidirectionalKinematics(1.0, 1.0)
    behavior.radius = 0.1
    dt = 0.1
    behavior.horizon = 5.0
    behavior.position = numpy.array((0, 0.05))
    behavior.target = core.Target(position=(10.0, 0.0))
    if isinstance(behavior.environment_state, core.GeometricState):
        behavior.environment_state.static_obstacles = [
            core.Disc(position=(1.5, 0.0), radius=0.5)
        ]
    for _ in range(30):
        cmd = behavior.compute_cmd(dt)
        behavior.actuate(cmd, dt)
    pose = behavior.pose
    twist = behavior.twist
    print(f'End loop @ ({pose.position[0]:.3f}, {pose.position[1]:.3f})'
          f'({twist.velocity[0]:.3f}, {twist.velocity[1]:.3f})')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default='HL')
    arg = parser.parse_args()
    main(arg.behavior)
