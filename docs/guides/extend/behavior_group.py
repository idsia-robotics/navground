from typing import SupportsFloat

from navground import core


class MyBehaviorGroup(core.BehaviorGroup):

    # MUST override
    def compute_cmds(self, time_step: SupportsFloat) -> list[core.Twist2]: ...
