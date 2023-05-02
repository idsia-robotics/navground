import argparse
import asyncio
import logging

from typing import Optional

from . import Scenario, World, load_experiment
from .real_time import RealTimeSimulation
from .ui.web_ui import WebUI, Rect


def until_idle(world: World, max_duration: float = -1):

    def f():
        return (all(a.controller.idle for a in world.agents)
                or (max_duration > 0 and world.time > max_duration))

    return f


async def run(scenario: Scenario,
              with_ui: bool = True,
              factor: float = 1.0,
              time_step: float = 0.04,
              ui_fps: float = 25.0,
              max_duration: float = -1,
              background: str = 'lightgray',
              bounds: Optional[Rect] = None) -> None:
    world = World()
    scenario.init_world(world)
    world.run(1, 0.0)
    if with_ui:
        web_ui = WebUI(host='127.0.0.1', max_rate=ui_fps)
        await web_ui.prepare()
        logging.info("Waiting for a client")
        while web_ui.number_of_client == 0:
            await asyncio.sleep(1.0)
        await web_ui.set_background_color(background)
    else:
        web_ui = None
    rt_sim = RealTimeSimulation(world,
                                time_step=time_step,
                                factor=factor,
                                web_ui=web_ui,
                                bounds=bounds)
    logging.info("Start simulation")
    await rt_sim.run(until=until_idle(world, max_duration=max_duration))
    sleep_percent = 100 * rt_sim.slept / rt_sim.total
    logging.info(f"Simulation done. Total time: {rt_sim.total:.3f} s: "
                 f"sleep {sleep_percent:.1f}% | "
                 f"simulation {100 - sleep_percent:.1f}%")
    if web_ui:
        await web_ui.stop()


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml',
                        help='yaml string',
                        type=str,
                        default="",
                        nargs='?')
    parser.add_argument('--input', help='yaml file', type=str, default="")
    parser.add_argument('--factor',
                        help='Real time factor',
                        type=float,
                        default=1.0)
    parser.add_argument('--ui-fps', help='Max fps', type=float, default=25.0)
    parser.add_argument('--no-ui', help='UI', action='store_true')
    parser.add_argument('--background', help='background color', type=str, default="lightgray")
    parser.add_argument('--min-x', help='min x', type=float, default="0.0")
    parser.add_argument('--min-y', help='min x', type=float, default="0.0")
    parser.add_argument('--max-x', help='min x', type=float, default="0.0")
    parser.add_argument('--max-y', help='min x', type=float, default="0.0")
    arg = parser.parse_args()
    if arg.input:
        with open(arg.input, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.yaml
    experiment = load_experiment(yaml)
    loop = asyncio.get_event_loop()
    bounds = ((arg.min_x, arg.min_y), (arg.max_x, arg.max_y))
    loop.run_until_complete(
        run(with_ui=not arg.no_ui,
            scenario=experiment.scenario,
            factor=arg.factor,
            time_step=experiment.time_step,
            max_duration=experiment.steps * experiment.time_step,
            ui_fps=arg.ui_fps,
            background=arg.background,
            bounds=bounds))
