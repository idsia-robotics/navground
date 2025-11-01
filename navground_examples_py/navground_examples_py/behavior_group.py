from __future__ import annotations

from typing import SupportsFloat

import numpy
from navground import core


class RandomSyncBehaviorGroup(core.BehaviorGroup):

    def __init__(self) -> None:
        core.BehaviorGroup.__init__(self)
        self.rg = numpy.random.default_rng(seed=0)
        self.step = 0
        self.cmds: list[core.Twist2] = []

    def compute_cmds(self, time_step: SupportsFloat) -> list[core.Twist2]:
        if self.step % self.size == 0 or self.size != len(self.cmds):
            self.cmds = [
                core.Twist2(
                    (self.rg.uniform(0, 1) * behavior.get_target_speed(), 0),
                    0) for behavior in self.members
            ]
        else:
            self.cmds = self.cmds[1:] + self.cmds[:1]
        self.step += 1
        return self.cmds


class RandomSyncBehavior(core.BehaviorGroupMember):

    _groups: dict[int, core.BehaviorGroup] = {}

    def make_group(self) -> RandomSyncBehaviorGroup:
        return RandomSyncBehaviorGroup()

    def get_group_hash(self) -> int:
        return 0

    def get_groups(self) -> dict[int, core.BehaviorGroup]:
        return self._groups


def main() -> None:
    """
        This example shows how to define and use a group behavior.
    """
    kinematics = core.kinematics.OmnidirectionalKinematics(1.0, 1.0)
    behaviors = [RandomSyncBehavior(kinematics, 0) for _ in range(10)]
    y = 0.0
    print("Start loop:")
    for behavior in behaviors:
        behavior.position = numpy.array((0, y))
        behavior.prepare()
        y += 1.0
        print(f'- {behavior.position[0]} ({behavior.velocity[0]})')

    for _ in range(10):
        for behavior in behaviors:
            cmd = behavior.compute_cmd(0.1)
            behavior.actuate(cmd, 0.1)
    print('End loop:')
    for behavior in behaviors:
        behavior.close()
        print(f'- {behavior.position[0]} ({behavior.velocity[0]})')


if __name__ == '__main__':
    main()
