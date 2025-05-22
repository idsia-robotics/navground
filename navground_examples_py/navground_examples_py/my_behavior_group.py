from navground import core


class PyIdleBehaviorGroup(core.BehaviorGroup):

    def compute_cmds(self, time_step: float):
        return [core.Twist2() for _ in self.members]


class PyIdleBehaviorGroupMember(core.BehaviorGroupMember, name="PyIdleGroup"):

    _groups: dict[int, PyIdleBehaviorGroup] = {}

    def make_group(self) -> PyIdleBehaviorGroup:
        return PyIdleBehaviorGroup()

    def get_group_hash(self) -> int:
        return 0

    def get_groups(self) -> dict[int, PyIdleBehaviorGroup]:
        return self._groups
