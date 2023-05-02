import argparse
import ctypes
import time
from typing import Type

from navground import core
from navground import sim


def print_register(cls: Type, title: str) -> None:
    print(title)
    print('=' * len(title))
    for name, properties in cls.type_properties.items():
        print(f'- {name}')
        for k, p in properties.items():
            print(f'     - {k} = {p.default_value} [{p.type_name}]')


def main() -> None:
    import navground_demos.sim
    path = ("/Users/jerome.guzzi/Dev/ROS/ros2_ws/install/lib/"
            "navground_demos/libnavground_demos.dylib")
    # Error ... is not doing much (apparently ... as I don't get the other c++ sampler)
    # After solving this second error, I could try to bind the command to load the library from c++
    lib = ctypes.cdll.LoadLibrary(path)
    print(vars(lib))

    # print_register(nav.Behavior, 'Behaviors')
    # print_register(nav.Kinematics, 'Kinematics')
    # print_register(sim.Task, 'Tasks')
    # print_register(sim.StateEstimation, 'State estimations')
    print_register(sim.WorldSampler, 'World samplers')
    return

    ws = sim.WorldSampler.make_type('PyThymioDemo')
    world = ws.sample()
    world.run(100, 0.1)
    title = 'After 100 steps (10 s)'
    title += '\n' + len(title) * '='
    print(title)
    print(sim.dump(world))
