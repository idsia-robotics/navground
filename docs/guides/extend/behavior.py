from navground import core

class MyBehavior(core.PyBehavior):

    def __init__(self, kinematics: core.Kinematics | None = None, radius: float):
        super().__init__(kinematics, radius)
        self._env_state = MyEnvironmentState()

    def get_environment_state(self) -> core.EnvironmentState:
        return self._env_state

    # CAN override
    # def compute_cmd_internal(time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_along_path(path: core.Path, speed: float , time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_pose(pose: core.Pose2, speed: float , angular_speed: float, time_step: float , frame: core.Frame ) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_point(point: core.Vector2, speed: float , time_step: float) -> core.Twist2: ...

    # CAN override
    # def  cmd_twist_towards_velocity(velocity: core.Vector2, time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_orientation(orientation: float, angular_speed: float, time_step: float ) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_angular_speed(angular_speed: float, time_step: float) -> core.Twist2: ...

    # CAN override
    # def cmd_twist_towards_stopping(time_step: float) -> core.Twist2: ...

    # CAN override
    # def desired_velocity_towards_point(point: core.Vector2, speed: float , time_step: float) -> core.Vector2: ...

    # CAN override
    # def  desired_velocity_towards_velocity(velocity: core.Vector2, time_step: float) -> core.Vector2: ...

    # CAN override
    # def twist_towards_velocity(velocity: core.Vector2) -> core.Twist2: ...
