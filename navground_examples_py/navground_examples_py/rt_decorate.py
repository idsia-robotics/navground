from __future__ import annotations

from navground import sim
from navground.sim.run_rt import main as main_rt
from navground.sim.ui import svg_color


def decorate(entity: sim.Entity) -> dict[str, str]:
    if isinstance(entity, sim.Agent) and entity.behavior:
        e = entity.behavior.efficacy
        return {'fill': svg_color(0.0, e, 1.0 - e)}
    return {}


def main() -> None:
    main_rt(decorate)
