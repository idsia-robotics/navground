import numpy as np
from navground import core


class PyIdleBehavior(core.Behavior, name="PyIdle"):

    def __init__(self,
                 kinematics: core.Kinematics | None = None,
                 radius: float = 0) -> None:
        super().__init__(kinematics, radius)
        self._ignore_obstacles = False

    def desired_velocity_towards_point(self, point: core.Vector2, speed: float,
                                       time_step: float) -> core.Vector2:
        if self.ignore_obstacles:
            return core.clamp_norm(point - self.position, speed)
        return np.zeros(2)

    def desired_velocity_towards_velocity(self, velocity: core.Vector2,
                                          time_step: float) -> core.Vector2:
        if self.ignore_obstacles:
            return velocity
        return np.zeros(2)

    @property
    @core.register(False,
                   "whether to move towards the target, ignoring obstacles")
    def ignore_obstacles(self) -> bool:
        return self._ignore_obstacles

    @ignore_obstacles.setter
    def ignore_obstacles(self, value: bool) -> None:
        self._ignore_obstacles = value
