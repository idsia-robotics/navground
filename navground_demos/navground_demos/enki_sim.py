import argparse
import os

import pyenki
from navground import sim

from .enki import enki2world, world2enki


class ThymioWithAgent(pyenki.Thymio2):

    def __init__(self, agent: sim.Agent):
        super().__init__(use_aseba_units=False)
        self.agent = agent
        agent.enki_object = self

    def controlStep(self, dt: float) -> None:
        self.motor_left_target, self.motor_right_target = world2enki(
            self.agent.behavior.actuated_wheel_speeds)


class EnkiExperiment:

    height = 0.1

    def __init__(self, experiment: sim.Experiment, factor: float):
        self.experiment = experiment
        self.factor = factor

    def prepare_run(self, seed: int) -> None:
        self.enki_world = pyenki.World()
        self.experiment.start_run(seed, True)
        for agent in self.experiment.world.agents:
            if agent.type == "thymio":
                thymio = ThymioWithAgent(agent)
                thymio.position = world2enki(agent.position)
                thymio.angle = agent.orientation
                self.enki_world.add_object(thymio)
        for obstacle in self.experiment.world.obstacles:
            cylinder = pyenki.CircularObject(100 * obstacle.disc.radius,
                                             100 * self.height, -1,
                                             pyenki.Color(0.5, 0.5, 0.5))

            cylinder.position = world2enki(obstacle.disc.position)
            self.enki_world.add_object(cylinder)
        for wall in self.experiment.world.walls:
            pass

    def update(self, dt: float) -> None:
        for agent in self.experiment.world.agents:
            if agent.type == "thymio":
                thymio = agent.enki_object
                agent.position = enki2world(thymio.position)
                agent.orientation = thymio.angle
                agent.velocity = enki2world(thymio.velocity)
        self.experiment.world.update_dry(dt)
        self.experiment.update()

    def run_once(self, seed: int) -> None:
        self.prepare_run(seed)
        if self.factor > 0:
            self.enki_world.run_in_viewer(
                cam_position=(100, 100),
                cam_altitude=300.0,
                cam_yaw=0.0,
                cam_pitch=-1,
                walls_height=self.height / 100,
                orthographic=False,
                realtime_factor=self.factor,
                callback=self.update)
        else:
            self.enki_world.run(self.experiment.steps,
                                self.experiment.time_step,
                                self.update)
        self.experiment.stop_run()

    def run(self):
        self.experiment.start()
        for i in range(self.experiment.runs + self.experiment.run_index):
            self.run_once(i)
        self.experiment.stop()


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Runs an experiment using the pyenki')
    parser.add_argument(
        'YAML',
        help='YAML string, or path to a YAML file, describing an experiment',
        type=str)
    parser.add_argument(
        '--factor',
        help=(
            'Real-time factor (set to 1.0 to run in real-time, set to higher '
            'to run faster or to lower to run slower then real-time). '
            'Set to zero or negative value to run as fast as possible without'
            ' visualization.'),
        type=float,
        default=0.0)
    return parser


def main() -> None:
    arg = parser().parse_args()
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.YAML
    experiment = EnkiExperiment(experiment=sim.load_experiment(yaml),
                                factor=arg.factor)
    experiment.run()
