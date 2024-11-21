import argparse
import os
import sys

import pyenki
from navground import sim

from .enki import enki2world, world2enki


class ThymioWithAgent(pyenki.Thymio2):

    def __init__(self, agent: sim.Agent):
        super().__init__(use_aseba_units=False)
        self.agent = agent
        agent.enki_object = self  # type: ignore

    def controlStep(self, dt: float) -> None:
        if not self.agent.behavior:
            return
        self.motor_left_target, self.motor_right_target = world2enki(
            self.agent.behavior.actuated_wheel_speeds)


class EnkiExperiment:

    height = 0.1

    def __init__(self, experiment: sim.Experiment, factor: float):
        self.experiment = experiment
        self.factor = factor

    def prepare_run(self, seed: int) -> None:
        self.enki_world = pyenki.World()
        self.exp_run = self.experiment.init_run(seed)
        self.world = self.exp_run.world
        for agent in self.world.agents:
            if agent.type == "thymio":
                thymio = ThymioWithAgent(agent)
                thymio.position = world2enki(agent.position)
                thymio.angle = agent.orientation
                self.enki_world.add_object(thymio)
        for obstacle in self.world.obstacles:
            cylinder = pyenki.CircularObject(100 * obstacle.disc.radius,
                                             100 * self.height, -1,
                                             pyenki.Color(0.5, 0.5, 0.5))

            cylinder.position = world2enki(obstacle.disc.position)
            self.enki_world.add_object(cylinder)
        for wall in self.world.walls:
            pass

    def update(self, dt: float) -> bool:
        for agent in self.world.agents:
            if agent.type == "thymio":
                thymio = agent.enki_object  # type: ignore
                agent.position = enki2world(thymio.position)
                agent.orientation = thymio.angle
                agent.velocity = enki2world(thymio.velocity)
        self.world.update_dry(dt)
        self.experiment.update_run(self.exp_run)
        if self.experiment.terminate_when_all_idle_or_stuck and self.world.agents_are_idle_or_stuck(
        ):
            return True
        if self.world.step >= self.experiment.steps:
            return True
        return False

    def run_once(self, seed: int) -> None:
        self.prepare_run(seed)
        self.experiment.start_run(self.exp_run)
        if self.factor > 0:
            self.enki_world.run_in_viewer(cam_position=(100, 100),  # type: ignore
                                          cam_altitude=300.0,
                                          cam_yaw=0.0,
                                          cam_pitch=-1,
                                          walls_height=self.height / 100,
                                          orthographic=False,
                                          realtime_factor=self.factor,
                                          callback=self.update)
        else:
            self.enki_world.run(self.experiment.steps,  # type: ignore
                                self.experiment.time_step, self.update)
        self.experiment.stop_run(self.exp_run)
        print(self.exp_run.poses.shape)

    def run(self):
        self.experiment.start()
        for i in range(self.experiment.number_of_runs +
                       self.experiment.run_index):
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
    nexp = sim.load_experiment(yaml)
    if not nexp:
        print("Could not load experiment", file=sys.stderr)
        exit(1)
    experiment = EnkiExperiment(experiment=nexp, factor=arg.factor)
    experiment.run()
