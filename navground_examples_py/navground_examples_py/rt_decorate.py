from typing import Dict

from navground import sim
from navground.sim.ui import svg_color
from navground.sim.run_rt import main as main_rt


def decorate(entity: sim.Entity) -> Dict[str, str]:
    if isinstance(entity, sim.Agent) and entity.behavior:
        e = entity.behavior.efficacy
        return {'fill': svg_color(0.0, e, 1.0 - e)}
    return {}


def main():
    main_rt(decorate)
