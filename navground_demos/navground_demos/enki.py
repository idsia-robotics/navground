import argparse
import itertools
import sys
import time
from collections.abc import Iterator

import numpy
import pyenki
from navground import core


def enki2world(cs: tuple[float, float]) -> core.Vector2:
    return numpy.array([0.01 * c for c in cs])


def world2enki(cs: core.Vector2Like) -> tuple[float, float]:
    return (100 * cs[0], 100 * cs[1])


class Thymio(pyenki.Thymio2):

    targets: Iterator[tuple[tuple[float, float, float], tuple[float, float]]]
    thymios: set['Thymio']

    def __init__(self,
                 behavior_name: str = "HL",
                 obstacles: list[pyenki.CircularObject] = []):
        super().__init__(use_aseba_units=False)
        behavior = core.Behavior.make_type(behavior_name)
        if not behavior:
            print(f"No behavior with name {behavior_name}", file=sys.stderr)
            sys.exit(1)
        self.behavior = behavior
        self.behavior.kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(
            0.01 * self.max_wheel_speed, 0.01 * self.wheel_axis)
        self.behavior.radius = 0.08
        self.behavior.safety_margin = 0.02
        self.behavior.optimal_speed = 0.12
        self.behavior.horizon = 1.0
        self.controller = core.Controller(self.behavior)
        self.controller.speed_tolerance = 0.01
        if isinstance(self.behavior.environment_state, core.GeometricState):
            self.behavior.environment_state.static_obstacles = [
                core.Disc(position=enki2world(obstacle.position),
                          radius=0.01 * obstacle.radius)
                for obstacle in obstacles
            ]

    def controlStep(self, dt: float) -> None:
        self.behavior.orientation = self.angle
        self.behavior.position = enki2world(self.position)
        self.behavior.velocity = enki2world(self.velocity)
        if isinstance(self.behavior.environment_state, core.GeometricState):
            self.behavior.environment_state.neighbors = [
                core.Neighbor(position=enki2world(thymio.position),
                              radius=0.08,
                              velocity=enki2world(thymio.velocity),
                              id=0) for thymio in self.thymios
            ]
        _ = self.controller.update(dt)
        self.motor_left_target, self.motor_right_target = world2enki(
            self.behavior.actuated_wheel_speeds)
        if self.controller.idle:
            color, target = next(self.targets)
            self.controller.go_to_position(target, 0.2)
            self.set_led_top(*color)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--behavior', help='', type=str, default="HL")
    parser.add_argument('--factor', help='', type=float, default=1.0)
    arg = parser.parse_args()
    behavior_name: str = arg.behavior
    world = pyenki.World(300, 300)
    r = (1.0, 0.0, 0.0)
    g = (0.0, 1.0, 0.0)
    cylinder = pyenki.CircularObject(10.0, 10.0, -1, pyenki.Color(0.8, 0.3, 0))
    cylinder.position = world2enki((1.5, 1.5))
    world.add_object(cylinder)
    obstacles = [cylinder]
    targets = ((r, (2.5, 1.5)), (g, (0.5, 1.5)))
    for color, point in targets:
        cylinder = pyenki.CircularObject(10.0, 1.0, -1, pyenki.Color(*color))
        cylinder.position = world2enki(point)
        world.add_object(cylinder)
    thymios = set()
    for point in ((2.0, 1.5), (1.0, 1.5)):
        thymio = Thymio(behavior_name=behavior_name, obstacles=obstacles)
        world.add_object(thymio)
        thymio.position = world2enki(point)
        thymio.angle = 0.0
        thymio.targets = itertools.cycle(targets)
        thymios.add(thymio)

    for thymio in thymios:
        thymio.thymios = thymios - {thymio}
    if arg.factor > 0:
        world.run_in_viewer(cam_position=(100, 100),  # type: ignore
                            cam_altitude=300.0,
                            cam_yaw=0.0,
                            cam_pitch=-1,
                            walls_height=10,
                            orthographic=False,
                            realtime_factor=arg.factor)
    else:
        print('Start simulating 1 minute at 50 ticks per second')
        a = time.time()
        for _ in range(50 * 60):
            world.step(0.02)
        print(f'Done simulating in {1000 * (time.time() - a):.1f} ms')
