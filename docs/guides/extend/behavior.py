from navground import core

# May use a custom environment state
class MyEnvironmentState(core.EnvironmentState):
    ...


class MyBehavior(core.Behavior):

    def __init__(self, kinematics: core.Kinematics | None = None, radius: float = 0):
        super().__init__(kinematics, radius)
        self._env_state = MyEnvironmentState()

    def get_environment_state(self) -> core.EnvironmentState:
        return self._env_state

    # CAN override
    # executed before the first evaluation
    # def prepare(self) -> None: ...

    # CAN override
    # executed after the last evaluation
    # def close(self) -> None: ...

    # CAN override
    # def compute_cmd_internal(self, time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_along_path(self, path: core.Path, speed: float , time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_pose(self, pose: core.Pose2, speed: float , angular_speed: float, time_step: float , frame: core.Frame ) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_point(self, point: core.Vector2, speed: float , time_step: float) -> core.Twist2: ...

    # CAN override
    # def  cmd_twist_towards_velocity(self, velocity: core.Vector2, time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_orientation(self, orientation: float, angular_speed: float, time_step: float ) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_angular_speed(self, angular_speed: float, time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_stopping(self, time_step: float) -> core.Twist2: ...

    # CAN override
    # def desired_velocity_towards_point(self, point: core.Vector2, speed: float , time_step: float) -> core.Vector2: ...

    # CAN override
    # def  desired_velocity_towards_velocity(self, velocity: core.Vector2, time_step: float) -> core.Vector2: ...

    # CAN override
    # def twist_towards_velocity(self, velocity: core.Vector2) -> core.Twist2: ...
