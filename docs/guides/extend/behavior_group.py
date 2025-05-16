from navground import core


class MyBehaviorGroup(core.BehaviorGroup):

    # MUST override
    def compute_cmds(self, time_step: float) -> list[core.Twist2]: ...
