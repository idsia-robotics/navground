from navground import core


class MyBehaviorGroupMember(core.BehaviorGroupMember):

    # Groups could be stored in a class variable

    _groups: dict[int, core.BehaviorGroup] = {}

    # MUST override
    def get_groups(self) -> dict[int, core.BehaviorGroup]:
        return self._groups

    # MUST override
    def make_group(self) -> core.BehaviorGroup: ...

    # CAN override
    # def get_group_hash(self) -> int: ...
