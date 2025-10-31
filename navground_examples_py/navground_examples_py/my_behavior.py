from __future__ import annotations

from typing import SupportsFloat

import numpy as np
from navground import core


class PyIdleBehavior(core.Behavior, name="PyIdle"):

    def __init__(self,
                 kinematics: core.Kinematics | None = None,
                 radius: float = 0) -> None:
        super().__init__(kinematics, radius)
        self._ignore_obstacles = False

    def desired_velocity_towards_point(
            self, point: core.Vector2Like, speed: SupportsFloat,
            time_step: SupportsFloat) -> core.Vector2:
        if self.ignore_obstacles:
            return core.clamp_norm(np.asarray(point) - self.position, speed)
        return core.zeros2()

    def desired_velocity_towards_velocity(
            self, velocity: core.Vector2Like,
            time_step: SupportsFloat) -> core.Vector2:
        if self.ignore_obstacles:
            return np.asarray(velocity)
        return core.zeros2()

    @property
    @core.register(False,
                   "whether to move towards the target, ignoring obstacles")
    def ignore_obstacles(self) -> bool:
        return self._ignore_obstacles

    @ignore_obstacles.setter
    def ignore_obstacles(self, value: bool) -> None:
        self._ignore_obstacles = value
