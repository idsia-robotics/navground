from __future__ import annotations
import numpy
import os
import typing
import math
import pathlib

__all__ = [
    'Action', 'AheadKinematics', 'Behavior', 'BehaviorRegister', 'Buffer',
    'BufferDescription', 'BufferMap', 'CachedCollisionComputation',
    'CollisionComputation', 'Controller', 'Disc', 'DummyBehavior',
    'EnvironmentState', 'FourWheelsOmniDriveKinematics', 'Frame',
    'GeometricState', 'HLBehavior', 'HRVOBehavior', 'HasProperties',
    'Kinematics', 'KinematicsRegister', 'LineSegment', 'Neighbor',
    'ORCABehavior', 'OmnidirectionalKinematics', 'Pose2', 'Property',
    'SensingState', 'SocialMargin', 'SocialMarginConstantModulation',
    'SocialMarginLinearModulation', 'SocialMarginLogisticModulation',
    'SocialMarginModulation', 'SocialMarginQuadraticModulation',
    'SocialMarginZeroModulation', 'Target', 'Twist2',
    'TwoWheelsDifferentialDriveKinematics',
    'DynamicTwoWheelsDifferentialDriveKinematics', 'WheeledKinematics',
    'behavior_has_geometric_state', 'dump', 'load_behavior', 'load_kinematics',
    'load_plugins', 'to_absolute', 'to_relative', 'BehaviorModulation'
]

Vector2Like = numpy.ndarray | tuple[float, float] | list[float]


def uses_doubles() -> bool:
    ...


class Action:
    """
    Holds the state of a long running task and notifies observers when it
    terminates.
    """

    class State:
        """
        The state of an action.
        """
        __members__: typing.ClassVar[dict[
            str, Action.
            State]]  # value = {'idle': <State.idle: 0>, 'running': <State.running: 1>, 'failure': <State.failure: 2>, 'success': <State.success: 3>}
        failure: typing.ClassVar[Action.State]  # value = <State.failure: 2>
        idle: typing.ClassVar[Action.State]  # value = <State.idle: 0>
        running: typing.ClassVar[Action.State]  # value = <State.running: 1>
        success: typing.ClassVar[Action.State]  # value = <State.success: 3>

        def __eq__(self, other: typing.Any) -> bool:
            ...

        def __getstate__(self) -> int:
            ...

        def __hash__(self) -> int:
            ...

        def __index__(self) -> int:
            ...

        def __init__(self, value: int) -> None:
            ...

        def __int__(self) -> int:
            ...

        def __ne__(self, other: typing.Any) -> bool:
            ...

        def __repr__(self) -> str:
            ...

        def __setstate__(self, state: int) -> None:
            ...

        def __str__(self) -> str:
            ...

        @property
        def name(self) -> str:
            ...

        @property
        def value(self) -> int:
            ...

    def abort(self) -> None:
        """
        Abort the action, calling the :py:attr:`done_cb` if set.
        """

    @property
    def done(self) -> bool:
        """
        Return whenever the action is done or not.

        :return:
            True if the action failed or succeeded
        """

    @property
    def done_cb(self) -> typing.Callable[[Action.State], None] | None:
        """
        A callback called when the action terminates

        The callback argument is the final state of the action.
        """

    @done_cb.setter
    def done_cb(self,
                arg0: typing.Callable[[Action.State], None] | None) -> None:
        ...

    @property
    def running(self) -> bool:
        """
        Return whenever the action is running.

        :return:
            True if the action is running
        """

    @property
    def running_cb(self) -> typing.Callable[[float], None] | None:
        """
        A callback called when the action is running

        The callback argument is the minimal expected time to terminate the
        action.
        """

    @running_cb.setter
    def running_cb(self, arg0: typing.Callable[[float], None] | None) -> None:
        ...

    @property
    def state(self) -> Action.State:
        """
        The current state
        """


class AheadKinematics(Kinematics):
    """
    Kinematics for non-wheeled agents that head towards where they move
    (e.g., people)
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self, max_speed: float, max_angular_speed: float) -> None:
        """
        Constructs a new instance.

        :param max_speed:
            The maximal speed

        :param max_angular_speed:
            The maximal angular speed
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...


class Behavior(BehaviorRegister, HasProperties):
    """
    This class describes a generic behavior to reach a target position
    avoiding collision. Users should not instantiate this class (it's
    behavior just keeps the agent in place) but one of it's concrete sub-
    classes.

    The following lists the typical usage of a behavior.

    *At initialization*

    1. select the concrete behavior, the agent's size (agents are shaped
       like discs), and :py:class:`Kinematics`;

2. configure the generic parameters : : py : attr :`optimal_speed`,
    : py : attr :`horizon`,
    : py : attr :`safety_margin` : py : attr :`rotation_tau`,
    and : py : attr :`heading_behavior`;

    3. configure the specific parameters of the concrete behavior.

    *At regular time intervals*

    1. update the agent's state with :py:attr:`pose` and :py:attr:`twist` (or
       other convenience methods)

    2. update the target with :py:attr:`target`

    3. update the environment state :py:meth:`get_environment_state`

    4. ask for a control commands by calling :py:meth:`compute_cmd`

    5. actuate the control commands through user code
    """

    class Heading:
        """
        Different behavior variants for the angular motion when it is no
        constrained by the kinematics or already specified by the current
        target.

        They ability to apply them depends on the kinematics and on the
        current target.
        """
        __members__: typing.ClassVar[dict[
            str, Behavior.
            Heading]]  # value = {'idle': <Heading.idle: 0>, 'target_point': <Heading.target_point: 1>, 'target_angle': <Heading.target_angle: 2>, 'target_angular_speed': <Heading.target_angular_speed: 3>, 'velocity': <Heading.velocity: 4>}
        idle: typing.ClassVar[Behavior.Heading]  # value = <Heading.idle: 0>
        target_angle: typing.ClassVar[
            Behavior.Heading]  # value = <Heading.target_angle: 2>
        target_angular_speed: typing.ClassVar[
            Behavior.Heading]  # value = <Heading.target_angular_speed: 3>
        target_point: typing.ClassVar[
            Behavior.Heading]  # value = <Heading.target_point: 1>
        velocity: typing.ClassVar[
            Behavior.Heading]  # value = <Heading.velocity: 4>

        def __eq__(self, other: typing.Any) -> bool:
            ...

        def __getstate__(self) -> int:
            ...

        def __hash__(self) -> int:
            ...

        def __index__(self) -> int:
            ...

        def __init__(self, value: int) -> None:
            ...

        def __int__(self) -> int:
            ...

        def __ne__(self, other: typing.Any) -> bool:
            ...

        def __repr__(self) -> str:
            ...

        def __setstate__(self, state: int) -> None:
            ...

        def __str__(self) -> str:
            ...

        @property
        def name(self) -> str:
            ...

        @property
        def value(self) -> int:
            ...

    def __getstate__(self) -> tuple:
        ...

    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0) -> None:
        """
        Constructs a new instance.

        :param kinematics:
            The kinematics of the agent.

        :param radius:
            The radius of the agent.
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...

    @typing.overload
    def actuate(self, twist: Twist2, time_step: float) -> None:
        """
        Actuate a twist command, integrating using :py:meth:`Pose2.integrate`

        :param twist_cmd:
            The twist

        :param time_step:
            The time step
        """

    @typing.overload
    def actuate(self, time_step: float) -> None:
        """
        Convenience method to actuate the stored actuated twist command,

        :param time_step:
            The time step
        """

    def check_if_target_satisfied(self) -> bool:
        """
        Check if the current target has been satisfied

        :return:
            True if the current target has been satisfied.
        """

    def compute_cmd_internal(self, time_step: float, frame: Frame) -> Twist2:
        ...

    def compute_cmd(self,
                    time_step: float,
                    frame: Frame | None = None) -> Twist2:
        """
        Query the behavior to get a control command

        Before calling this method, update the state using methods such as
        :py:attr:`pose`, and :py:attr:`twist` and set the target :py:attr:`target`.

        Behaviors may use caching to speed up the next queries if the state
        does not change.

        :param time_step:
            The control time step. Not all behavior use it but some may use
            it, for example, to limit accelerations.

        :param frame:
            The desired frame of reference for the twist. Leave undefined to
            use the default frame depending on the kinematics (see :py:attr:`default_cmd_frame`)

        :return:
            The control command as a twist in the specified frame.
        """

    def estimate_time_until_target_satisfied(self) -> float:
        """
        Estimate how much time before the target is satisfied

        :return:
            A positive value if the target is not yet satisfied, else 0.
        """

    def feasible_angular_speed(self, arg0: float) -> float:
        """
        Clamp an angular speed in the range of feasible values given by the
        kinematics.

        :param value:
            The desired value

        :return:
            the nearest feasible value
        """

    def feasible_speed(self, arg0: float) -> float:
        """
        Clamp a speed in the range of feasible values given by the kinematics.

        :param value:
            The desired value

        :return:
            the nearest feasible value
        """

    @typing.overload
    def feasible_twist(self, arg0: Twist2, arg1: Frame | None) -> Twist2:
        """
        Clamp an twist in the range of feasible values given by the
        kinematics.

        :param value:
            The desired value

        :param frame:
            The desired frame

        :return:
            the nearest feasible value
        """

    @typing.overload
    def feasible_twist(self, arg0: Twist2, time_step: float, arg1: Frame | None) -> Twist2:
        ...

    def get_actuated_twist(self, frame: Frame = ...) -> Twist2:
        """
        Gets the last actuated twist.

        :param frame:
            The desired frame of reference.

        :return:
            The actuated twist.
        """

    def get_environment_state(self) -> EnvironmentState:
        """
        Gets the environment state.

        :return:
            The environment state.
        """

    def get_twist(self, frame: Frame = ...) -> Twist2:
        """
        Gets the current twist.

        :param frame:
            The desired frame of reference.

        :return:
            The current twist.
        """

    def get_velocity(self, frame: Frame = ...) -> numpy.ndarray:
        """
        Convenience method to get the current velocity. See :py:meth:`get_twist`

        :param frame:
            The desired frame of reference.

        :return:
            The velocity.
        """

    def set_state_from(self, other: Behavior) -> None:
        """
        Clone the state of this behavior from another behavior

        :param other:
            The other behavior
        """

    def set_velocity(self, velocity: Vector2Like, frame: Frame = ...) -> None:
        """
        Convenience method to set the current velocity. See :py:attr:`twist`

        :param value:
            The velocity

        :param frame:
            The desired frame of reference.
        """

    def to_frame(self, arg0: Twist2, arg1: Frame) -> Twist2:
        """
        Convert a twist to a reference frame.

        :param value:
            The original twist.

        :param frame:
            The desired frame of reference

        :return:
            The twist in the desired frame of reference.
        """

    def twist_from_wheel_speeds(self, arg0: list[float]) -> Twist2:
        """
        Convenience method to transform from wheel speeds to twist.

        If the agent is not wheeled (:py:attr:`Kinematics.is_wheeled`), an zero
        twist is returned.

        :param value:
            The wheel speeds

        :return:
            The corresponding twist.
        """

    def wheel_speeds_from_twist(self, arg0: Twist2) -> list[float]:
        """
        Convenience method to transform from twist to wheel speeds.

        If the agent is not wheeled (:py:attr:`Kinematics.is_wheeled`), an empty
        vector is returned.

        :param value:
            The twist

        :return:
            The corresponding wheel speeds.
        """

    @property
    def actuated_twist(self) -> Twist2:
        """
        The last actuated twist.

        :param frame:
            the desired frame of reference.
        """

    @actuated_twist.setter
    def actuated_twist(self, arg1: Twist2) -> None:
        ...

    @property
    def actuated_wheel_speeds(self) -> list[float]:
        """
        Convenience method to get the last actuated wheel speeds from :py:attr:`actuated_twist`.

        If the agent is not wheeled (:py:attr:`Kinematics.is_wheeled`), an empty
        vector is returned.
        """

    @property
    def angular_speed(self) -> float:
        """
        Convenience method to get the current the angular speed.
        """

    @angular_speed.setter
    def angular_speed(self, arg1: float) -> None:
        ...

    @property
    def assume_cmd_is_actuated(self) -> bool:
        """
        Whether to assume that the compute command will be actuated as it
        is.

        if set, :py:class:`behavior` will assume that the control command computed by
        :py:meth:`compute_cmd` be actuated, therefore setting :py:attr:`actuated_twist` to that value. if not set, the user should set :py:attr:`behavior.actuated_twist` before querying for a new control
        commands: some behavior use the old actuated control command to
        compute smoother controls.
        """

    @assume_cmd_is_actuated.setter
    def assume_cmd_is_actuated(self, arg1: bool) -> None:
        ...

    @property
    def default_cmd_frame(self) -> Frame:
        """
        The most natural frame for the current kinematics: :py:meth:`Frame.relative` in case the agent is wheeled, else :
    py:meth:`Frame.absolute`.

        :return:
            The frame
        """

    @property
    def desired_velocity(self) -> numpy.ndarray:
        """
        The last computed desired velocity.
        """

    @property
    def efficacy(self) -> float:
        """
        The efficacy: the projection of the current velocity on the ideal
        velocity (ignoring obstacles) towards the target.

        a value of 1.0 denotes ideal efficacy, value of 0.0 that the agent is
        stuck.
        """

    @property
    def environment_state(self) -> EnvironmentState:
        """
        The environment state.
        """

    @property
    def heading_behavior(self) -> Behavior.Heading:
        """
        The heading behavior:
        """

    @heading_behavior.setter
    def heading_behavior(self, arg1: Behavior.Heading) -> None:
        ...

    @property
    def horizon(self) -> float:
        """
        The horizon: the size of the portion of environment around the
        agent considered when computing possible collisions. larger values
        generally lead to a more computationally expensive :py:meth:`compute_cmd`
        but fewer deadlocks and less jamming.
        """

    @horizon.setter
    def horizon(self, arg1: float) -> None:
        ...

    @property
    def is_stuck(self) -> bool:
        """
        Determines if the agent is stuck: if should move but it is still.
        """

    @property
    def kinematics(self) -> Kinematics:
        """
        The kinematics.
        """

    @kinematics.setter
    def kinematics(self, arg1: Kinematics) -> None:
        ...

    @property
    def max_angular_speed(self) -> float:
        """
        The maximal angular speed speed.
        """

    @max_angular_speed.setter
    def max_angular_speed(self, arg1: float) -> None:
        ...

    @property
    def max_speed(self) -> float:
        """
        The maximal speed.
        """

    @max_speed.setter
    def max_speed(self, arg1: float) -> None:
        ...

    @property
    def optimal_angular_speed(self) -> float:
        """
        The desired optimal angular speed.

        unless configured with :py:attr:`optimal_angular_speed`, it is set to
        :py:attr:`max_angular_speed`.
        """

    @optimal_angular_speed.setter
    def optimal_angular_speed(self, arg1: float) -> None:
        ...

    @property
    def optimal_speed(self) -> float:
        """
        The desired optimal speed.

        unless configured with :py:attr:`optimal_speed`, it is set to :py:attr:`max_speed`.
        """

    @optimal_speed.setter
    def optimal_speed(self, arg1: float) -> None:
        ...

    @property
    def orientation(self) -> float:
        """
        Convenience method to get the current orientation. See :py:attr:`pose`.
        """

    @orientation.setter
    def orientation(self, arg1: float) -> None:
        ...

    @property
    def pose(self) -> Pose2:
        """
        The current pose in the world-fixed frame.
        """

    @pose.setter
    def pose(self, arg1: Pose2) -> None:
        ...

    @property
    def position(self) -> numpy.ndarray:
        """
        Convenience method to get the current position in the world-fixed
        frame. See :py:attr:`pose`.
        """

    @position.setter
    def position(self, arg1: Vector2Like) -> None:
        ...

    @property
    def radius(self) -> float:
        """
        The radius of the agent.
        """

    @radius.setter
    def radius(self, arg1: float) -> None:
        ...

    @property
    def rotation_tau(self) -> float:
        """
        The relaxation time to rotate towards a desired orientation.

        the default behaviors applies a p control to rotations, e.g.,
        :math:`\\omega = \\frac{\\delta \\theta}{
      \\tau_\\textrm { rot }
    }
    `
        """

    @rotation_tau.setter
    def rotation_tau(self, arg1: float) -> None:
        ...

    @property
    def safety_margin(self) -> float:
        """
        The minimal safety margin to keep away from obstacles
        """

    @safety_margin.setter
    def safety_margin(self, arg1: float) -> None:
        ...

    @property
    def social_margin(self) -> SocialMargin:
        """
        The behavior social margin modulation
        """

    @property
    def target(self) -> Target:
        """
        The target.
        """

    @target.setter
    def target(self, arg1: Target) -> None:
        ...

    @property
    def twist(self) -> Twist2:
        """
        The current twist.

        :param frame:
            the desired frame of reference.
        """

    @twist.setter
    def twist(self, arg1: Twist2) -> None:
        ...

    @property
    def type(self) -> str:
        """
        The name associated to the type of an object.
        """

    @property
    def velocity(self) -> numpy.ndarray:
        """
        Convenience method to get the current velocity. See :py:meth:`get_twist`

        :param frame:
            The desired frame of reference.
        """

    @velocity.setter
    def velocity(self, arg1: Vector2Like) -> None:
        ...

    @property
    def wheel_speeds(self) -> list[float]:
        """
        Convenience method to get the current wheel speeds. See :py:meth:`get_twist`
        """

    @wheel_speeds.setter
    def wheel_speeds(self, arg1: list[float]) -> None:
        ...

    def get_target_position(self, arg1: Frame) -> numpy.ndarray | None:
        ...

    def get_target_orientation(self, arg1: Frame) -> float:
        ...

    def get_target_direction(self, arg1: Frame) -> numpy.ndarray | None:
        ...

    def get_target_velocity(self, arg1: Frame) -> numpy.ndarray:
        ...

    def get_target_angular_speed(self) -> float:
        ...

    def get_target_distance(self, arg1: bool = False) -> float | None:
        ...

    def get_target_speed(self) -> float:
        ...

    @property
    def modulations(self) -> list['BehaviorModulation']:
        ...

    def add_modulation(self, arg0: 'BehaviorModulation') -> None:
        ...

    def remove_modulation(self, arg0: 'BehaviorModulation') -> None:
        ...

    def clear_modulations(self) -> None:
        ...


class BehaviorRegister:
    type_properties: typing.ClassVar[dict]
    types: typing.ClassVar[list] = [
        'Dummy', 'HL', 'HRVO', 'ORCA', 'PyDummy', 'SocialForce'
    ]

    @staticmethod
    def _add_property(arg0: str, arg1: str, arg2: typing.Any, arg3: bool | int
                      | float | str | numpy.ndarray | list[bool] | list[int]
                      | list[float] | list[str] | list[numpy.ndarray],
                      arg4: str, arg5: list[str]) -> None:
        ...

    @staticmethod
    def _register_type(arg0: str, arg1: typing.Any) -> None:
        ...

    @staticmethod
    def make_type(name: str) -> typing.Any:
        """
        Create an object of a sub-class selected by name.

        :param type:
            The associated type name.

        :return:
            An object of a registered sub-class or ``None`` in case the desired name is not found.
        """


class BehaviorModulation(BehaviorModulationRegister, HasProperties):

    def __getstate__(self) -> tuple:
        ...

    def __init__(self) -> None:
        ...

    def __setstate__(self, arg0: tuple) -> None:
        ...

    def pre(self, arg0: Behavior, arg1: float) -> None:
        ...

    def post(self, arg0: Behavior, arg1: float, arg2: Twist2) -> Twist2:
        ...

    @property
    def enabled(self) -> bool:
        ...

    @enabled.setter
    def enabled(self, arg0: bool) -> None:
        ...

    @property
    def type(self) -> str:
        """
        The name associated to the type of an object.
        """


class BehaviorModulationRegister:
    type_properties: typing.ClassVar[dict]
    types: typing.ClassVar[list] = ['LimitAcceleration', 'Relaxation', 'MotorPID']

    @staticmethod
    def _add_property(arg0: str, arg1: str, arg2: typing.Any, arg3: bool | int
                      | float | str | numpy.ndarray | list[bool] | list[int]
                      | list[float] | list[str] | list[numpy.ndarray],
                      arg4: str, arg5: list[str]) -> None:
        ...

    @staticmethod
    def _register_type(arg0: str, arg1: typing.Any) -> None:
        ...

    @staticmethod
    def make_type(name: str) -> typing.Any:
        ...


class RelaxationModulation(BehaviorModulation):

    def __init__(self, tau: float = 0.125) -> None:
        ...

    @property
    def tau(self) -> float:
        ...

    @tau.setter
    def tau(self, arg0: float) -> None:
        ...


class LimitAccelerationModulation(BehaviorModulation):

    def __init__(self, max_acceleration: float = math.inf,
                 max_angular_acceleration: float = math.inf) -> None:
        ...

    @property
    def max_acceleration(self) -> float:
        ...

    @max_acceleration.setter
    def max_acceleration(self, arg0: float) -> None:
        ...

    @property
    def max_angular_acceleration(self) -> float:
        ...

    @max_angular_acceleration.setter
    def max_angular_acceleration(self, arg0: float) -> None:
        ...

class MotorPIDModulation(BehaviorModulation):

    def __init__(self, k_p: float = 1,
                 k_i: float = 0, k_d: float = 0) -> None:
        ...

    @property
    def k_p(self) -> float:
        ...

    @k_p.setter
    def k_p(self, arg0: float) -> None:
        ...

    @property
    def k_i(self) -> float:
        ...

    @k_i.setter
    def k_i(self, arg0: float) -> None:
        ...

    @property
    def k_d(self) -> float:
        ...

    @k_d.setter
    def k_d(self, arg0: float) -> None:
        ...


class Buffer:
    """
    A typed, bounded, multi-dimensional array, similar to ``numpy``
    arrays.
    """

    @typing.overload
    def __init__(
        self, description: BufferDescription,
        value: float | float | int | int | int | int | int | int | int | int
    ) -> None:
        """
        Constructs a new instance with data set to a uniform value.

        :param desc:
            The description

        :param value:
            The value to assign to all of the buffer.
        """

    @typing.overload
    def __init__(
        self, description: BufferDescription,
        data: list[float] | list[float] | list[int] | list[int] | list[int]
        | list[int] | list[int] | list[int] | list[int] | list[int]
    ) -> None:
        """
        Constructs a new instance, with data set to zero.

        :param desc:
            The description
        """

    @typing.overload
    def __init__(self, description: BufferDescription) -> None:
        """
        Constructs a new instance with data

        :param desc:
            The description

        :param value:
            The data
        """

    @typing.overload
    def __init__(
        self, data: list[float] | list[float] | list[int] | list[int]
        | list[int] | list[int] | list[int] | list[int] | list[int] | list[int]
    ) -> None:
        """
        Constructs an unbounded, flat buffer of data

        :param value:
            The data
        """

    def __repr__(self) -> str:
        ...

    def set_description(self, arg0: BufferDescription, arg1: bool) -> bool:
        """
        Sets the description.

        If force is set, it will set the description and possibly reset the
        buffer if the type or size have changed. If force is not set, it won't
        allow changing shape to a different size or type to a different type.

        :param value:
            The value

        :param force:
            Whenever data may be reset.

        :return:
            Whether the description was set or not.
        """

    @property
    def categorical(self) -> bool:
        """
        Whether buffer is categorical.
        """

    @categorical.setter
    def categorical(self, arg1: bool) -> None:
        ...

    @property
    def data(self) -> numpy.ndarray:
        """
        The data stored in the buffer

        template parameter ``t``:
            the desired type
        """

    @data.setter
    def data(self, arg1: Buffer) -> None:
        ...

    @property
    def description(self) -> BufferDescription:
        """
        The description.
        """

    @description.setter
    def description(self, arg1: BufferDescription) -> None:
        ...

    @property
    def high(self) -> float:
        """
        The upper bound.
        """

    @high.setter
    def high(self, arg1: float) -> None:
        ...

    @property
    def low(self) -> float:
        """
        The lower bound.
        """

    @low.setter
    def low(self, arg1: float) -> None:
        ...

    @property
    def shape(self) -> tuple:
        """
        The shape.
        """

    @shape.setter
    def shape(self, arg1: list[int]) -> None:
        ...

    @property
    def size(self) -> int:
        """
        The buffer size
        """

    @property
    def type(self) -> numpy.dtype[typing.Any]:
        """
        The buffer type
        """

    @type.setter
    def type(self, arg1: typing.Any) -> None:
        ...


class BufferDescription:
    """
    Describes a typed, bounded, multi-dimensional buffer.

    Mimics Gymnasium Box spaces
    (https://gymnasium.farama.org/api/spaces/).
    """

    def __init__(self,
                 shape: list[int],
                 dtype: typing.Any = 'float',
                 low: float = 2.2250738585072014e-308,
                 high: float = 1.7976931348623157e+308,
                 categorical: bool = False) -> None:
        """
        Constructs a new instance.

        :param shape:
            The shape

        :param type:
            The type

        :param low:
            The low

        :param high:
            The high

        :param categorical:
            The categorical
        """

    def __repr__(self) -> str:
        ...

    @property
    def categorical(self) -> bool:
        """
        Whether the [integer] data is categorical or not.
        """

    @property
    def high(self) -> float:
        """
        The upper limits
        """

    @property
    def low(self) -> float:
        """
        The lower limits
        """

    @property
    def shape(self) -> tuple:
        """
        The shape of the buffer
        """

    @property
    def type(self) -> numpy.dtype[typing.Any]:
        """
        A code that identify the type of data.
        """


class BufferMap:
    """
    A dictionary of type Dict[str, Buffer]
    """

    def __bool__(self) -> bool:
        """
        Check whether the map is nonempty
        """

    @typing.overload
    def __contains__(self, arg0: str) -> bool:
        ...

    @typing.overload
    def __contains__(self, arg0: typing.Any) -> bool:
        ...

    def __delitem__(self, arg0: str) -> None:
        ...

    def __getitem__(self, arg0: str) -> Buffer:
        ...

    def __init__(self) -> None:
        ...

    def __iter__(self) -> typing.Iterator[str]:
        ...

    def __len__(self) -> int:
        ...

    def __setitem__(self, arg0: str, arg1: Buffer) -> None:
        ...

    def items(self) -> typing.ItemsView:
        ...

    def keys(self) -> typing.KeysView:
        ...

    def values(self) -> typing.ValuesView:
        ...


class CachedCollisionComputation(CollisionComputation):
    """
    This class extend :py:class:`CollisionComputation` to cache the results.

    It assumes that the agents is only interested in possible collisions
    when moving in directions comprised in an interval :math:`[\\alpha,
    \\alpha + \\Delta]`, represented by in :math:`N` points at regular
    steps, only up to a maximal distance :math:`D`, and that the agent
    moves at at given speed.
    """

    def __init__(self) -> None:
        """
        Construct an instance
        """

    def get_free_distance(self, dynamic: bool) -> list[float]:
        """
        Returns the free distance to collision for the cached interval of
        headings.

        :param dynamic:
            If the agent is moving

        :return:
            The distance before possibly colliding for each direction in the
            interval :math:`[\\alpha, \\alpha + \\Delta]`.
        """

    @property
    def length(self) -> float:
        """
        The cache interval length :math:`\\delta`.
        """

    @length.setter
    def length(self, arg1: float) -> None:
        ...

    @property
    def max_distance(self) -> float:
        """
        The maximal distance :math:`d` to consider
        """

    @max_distance.setter
    def max_distance(self, arg1: float) -> None:
        ...

    @property
    def min_angle(self) -> float:
        """
        The cache interval lower bound :math:`\\alpha`.
        """

    @min_angle.setter
    def min_angle(self, arg1: float) -> None:
        ...

    @property
    def resolution(self) -> int:
        """
        The resolution: the number of discrete angles :math:`n`.
        """

    @resolution.setter
    def resolution(self, arg1: int) -> None:
        ...

    @property
    def speed(self) -> float:
        """
        The agent speed.
        """

    @speed.setter
    def speed(self, arg1: float) -> None:
        ...


class CollisionComputation:
    """
    This class compute collisions of moving points with lists of :py:class:`DiscCache` and :py:class:`LineSegment`.
    """

    def __init__(self) -> None:
        """
        Construct an instance.
        """

    def dynamic_free_distance(self, angle: float, max_distance: float,
                              speed: float) -> float:
        """
        Returns the free distance if the agent will be move

        :param angle:
            The angle (absolute)

        :param max_distance:
            The maximal distance to consider

        :param speed:
            The speed of the agent

        :return:
            The distance in direction `angle` before possibly colliding
        """

    def get_angles_for_sector(self, from_angle: float, length: float,
                              resolution: int) -> list[float]:
        """
        Return regularly sampled angles.

        :param from:
            The interval lower bound

        :param length:
            The length of the interval

        :param resolution:
            The number of values in the interval

        :return:
            Angles regularly sampled in the interval [from, from + length].
        """

    def get_contour_for_sector(self,
                               from_angle: float,
                               length: float,
                               resolution: int,
                               max_distance: float,
                               dynamic: bool,
                               speed: float = 0) -> list[float]:
        """
        Return the polar contour for an interval of headings.

        :param from:
            The interval lower bound

        :param length:
            The length of the interval

        :param resolution:
            The number of values in the interval

        :param max_distance:
            The maximum distance to consider (collision behind this distance
            are effectively ignored)

        :param dynamic:
            If the agent is moving

        :param speed:
            The speed at which the agent is moving

        :return:
            An arrays of angles sampled regularly in the interval [from, from
            + length] and one array with the free distance in their direction.
        """

    def get_free_distance_for_sector(self,
                                     from_angle: float,
                                     length: float,
                                     resolution: int,
                                     max_distance: float,
                                     dynamic: bool,
                                     speed: float = 0) -> list[float]:
        """
        Return the free distance to collision for an interval of headings.

        :param from:
            The interval lower bound

        :param length:
            The length of the interval

        :param resolution:
            The number of values in the interval

        :param max_distance:
            The maximum distance to consider (collision behind this distance
            are effectively ignored)

        :param dynamic:
            If the agent is moving

        :param speed:
            The speed at which the agent is moving

        :return:
            The free distance for each angle in the interval [from, from +
            length].
        """

    def setup(self,
              pose: Pose2 = ...,
              margin: float = 0,
              line_segments: list[LineSegment] = [],
              static_discs: list[Disc] = [],
              dynamic_discs: list[Neighbor] = []) -> None:
        """
        Set the state from collections of :py:class:`LineSegment`, :py:class:`Disc`, and
        :py:class:`Neighbor`.

        :param pose_:
            The pose

        :param margin_:
            The margin

        :param line_segments:
            The line segments

        :param static_discs:
            The static discs

        :param dynamic_discs:
            The dynamic discs
        """

    def static_free_distance(self,
                             angle: float,
                             max_distance: float,
                             include_neighbors: bool = True) -> float:
        """
        Returns the free distance if the agent will be static

        :param angle:
            The angle (absolute)

        :param max_distance:
            The maximal distance to consider

        :param include_neighbors:
            Indicates if the neighbors should be included in the computation

        :return:
            The distance in direction `angle` before possibly colliding
        """


class Controller:
    """
    This class exposes a higher level, stateful interface to a behavior,
    to which it delegates 2D collision avoidance.

    The controller provides actions (event-based interfaces) to

    - go to a point/pose, stopping once the target has been reached,

    - follow a point/pose, which does not terminates once the target has
      been reached

    - follow a velocity/twist

    *Typical usage of a controller*

    1. Pick and configure a :py:class:`Behavior`

    2. Initialize and configure the controller

    3. At regular intervals, update the state, using the :py:class:`Behavior`
       API, and call :py:meth:`update`.

    4. When needed, trigger one of the actions.

    5. Either manually check the action's :py:attr:`state` or set callbacks
       to be notified when the action terminates or updates.

    6. Actuate the target command by using the return value of :py:meth:`update`
       or by setting a callback :py:meth:`set_cmd_cb`
    """

    def __init__(self, behavior: Behavior | None = None) -> None:
        """
        Constructs a new instance.

        :param behavior:
            The navigation behavior
        """

    def follow_direction(self, direction: Vector2Like) -> Action:
        """
        Starts an action to follow a target direction.

        If an action is already running, the controller aborts it, unless it
        was following a velocity/twist, in which case it just updates the
        target.

        :param direction:
            The target direction. Must have positive norm.

        :return:
            The new action.
        """

    def follow_point(self, point: Vector2Like) -> Action:
        """
        Starts an action to follow a point.

        The action keeps running even after the agent arrive at the target.

        If an action is already running, the controller aborts it, unless it
        was following a point/pose, in which case it just updates the target.

        :param point:
            The target point

        :return:
            The new action.
        """

    def follow_pose(self, pose: Pose2) -> Action:
        """
        Starts an action to follow a pose

        The action keeps running even after the agent arrive at the target.

        If an action is already running, the controller aborts it, unless it
        was following a point/pose, in which case it just updates the target.

        :param pose:
            The target pose

        :return:
            The new action.
        """

    def follow_twist(self, twist: Twist2) -> Action:
        """
        Starts an action to follow a target twist.

        If an action is already running, the controller aborts it, unless it
        was following a velocity/twist, in which case it just updates the
        target.

        :param twist:
            The target twist

        :return:
            The new action.
        """

    def follow_velocity(self, velocity: Vector2Like) -> Action:
        """
        Starts an action to follow a target velocity.

        If an action is already running, the controller aborts it, unless it
        was following a velocity/twist, in which case it just updates the
        target.

        :param velocity:
            The target velocity

        :return:
            The new action.
        """

    def go_to_pose(self, pose: Pose2, position_tolerance: float,
                   orientation_tolerance: float) -> Action:
        """
        Starts an action to go to a pose.

        The action succeed once the agent arrives within a tolerance from the
        target pose and comes to a stop.

        If an action is already running, the controller aborts it.

        :param pose:
            The target pose

        :param position_tolerance:
            The spatial tolerance

        :param orientation_tolerance:
            The spatial tolerance

        :return:
            The new action.
        """

    def go_to_position(self, position: Vector2Like,
                       tolerance: float) -> Action:
        """
        Starts an action to go to a point.

        The action succeed once the agent arrives within a tolerance from the
        target point and comes to a stop.

        If an action is already running, the controller aborts it.

        :param point:
            The target point

        :param tolerance:
            The spatial tolerance

        :return:
            The new action.
        """

    def set_cmd_cb(self, callback: typing.Callable[[Twist2], None]) -> None:
        """
        Sets the callback called each time a command is computed for an active
        action.

        :param value:
            A callback that takes the command twist as argument.
        """

    def stop(self) -> None:
        """
        Abort the running action.
        """

    def update(self, time_step: float) -> Twist2:
        """
        Updates the control for time step.

        Internally calls :py:meth:`Behavior.compute_cmd` for collision avoidance.

        :param time_step:
            The time step

        :return:
            The command twist to execute the current action
        """

    @property
    def behavior(self) -> Behavior:
        """
        The navigation behavior used by the controller
        """

    @behavior.setter
    def behavior(self, arg1: Behavior) -> None:
        ...

    @property
    def cmd_frame(self) -> Frame | None:
        """
        The frame of reference for the command.
        """

    @cmd_frame.setter
    def cmd_frame(self, arg1: Frame | None) -> None:
        ...

    @property
    def idle(self) -> bool:
        """
        Returns whether the control action is idle.

        :return:
            True only if the control action is not running.
        """

    @property
    def speed_tolerance(self) -> float:
        """
        The minimal speed to consider the agent as stopped.

        the default is 1 cm/s.
        """

    @speed_tolerance.setter
    def speed_tolerance(self, arg1: float) -> None:
        ...

    @property
    def state(self) -> Action.State:
        """
        The state of the control action.
        """


class Disc:
    """
    A circular shape. Used to represent obstacles.
    """

    def __init__(self, position: Vector2Like, radius: float) -> None:
        """
        Constructs a new instance.

        :param position:
            The position

        :param radius:
            The radius
        """

    def __repr__(self) -> str:
        ...

    def distance(self, other: Disc) -> float:
        """
        Returns the signed distance to another disc. Negative if the discs
        overlaps.

        :param other:
            The other disc

        :return:
            The distance between the centers minus the radii.
        """

    @property
    def position(self) -> numpy.ndarray:
        """
        The center of the disc in world frame
        """

    @position.setter
    def position(self, arg0: Vector2Like) -> None:
        ...

    @property
    def radius(self) -> float:
        """
        Radius
        """

    @radius.setter
    def radius(self, arg0: float) -> None:
        ...


class DummyBehavior(Behavior):
    """
    Dummy behavior that ignores obstacles.

    Mainly useful to test the interaction with other components

    *Registered properties*: none

    *State*: empty
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0) -> None:
        """
        Constructs a new instance.

        :param kinematics:
            The kinematics of the agent.

        :param radius:
            The radius of the agent.
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...


class EnvironmentState:
    """
    """


class FourWheelsOmniDriveKinematics(WheeledKinematics):
    """
    Four Omni-differential wheels (e.g., a Robomaster)

    .. warning::
        We assume that the distance between front and back wheel centers
        is the same as the lateral distance.
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self, max_speed: float, axis: float) -> None:
        """
        Constructs a new instance.

        :param max_speed:
            The maximal wheel speed

        :param axis:
            The wheel axis (i.e., the distance between the wheels)
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...


class Frame:
    """
    """
    __members__: typing.ClassVar[dict[
        str,
        Frame]]  # value = {'relative': <Frame.relative: 0>, 'absolute': <Frame.absolute: 1>}
    absolute: typing.ClassVar[Frame]  # value = <Frame.absolute: 1>
    relative: typing.ClassVar[Frame]  # value = <Frame.relative: 0>

    def __eq__(self, other: typing.Any) -> bool:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __ne__(self, other: typing.Any) -> bool:
        ...

    def __repr__(self) -> str:
        ...

    def __setstate__(self, state: int) -> None:
        ...

    def __str__(self) -> str:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...


class GeometricState(EnvironmentState):
    """
    """

    def __init__(self) -> None:
        ...

    @property
    def line_obstacles(self) -> list[LineSegment]:
        """
        The current list of line obstacles. positions are in the world
        fixed frame.
        """

    @line_obstacles.setter
    def line_obstacles(self, arg1: list[LineSegment]) -> None:
        ...

    @property
    def neighbors(self) -> list[Neighbor]:
        """
        The current list of neighbors. positions are in the world fixed
        frame.
        """

    @neighbors.setter
    def neighbors(self, arg1: list[Neighbor]) -> None:
        ...

    @property
    def static_obstacles(self) -> list[Disc]:
        """
        The current list of static obstacles. positions are in the world
        fixed frame.
        """

    @static_obstacles.setter
    def static_obstacles(self, arg1: list[Disc]) -> None:
        ...


class HLBehavior(Behavior):
    """
    Human-like obstacle avoidance behavior.

    The behavior inspired by how pedestrian move, has been originally
    presented in

    Guzzi, J.; Giusti, A.; Gambardella, L.M.; Theraulaz, G.; Di Caro,
    G.A., "Human-friendly robot navigation in dynamic environments,"
    Robotics and Automation (ICRA), 2013 IEEE International Conference on,
    vol., no., pp.423,430, 6-10 May 2013

    *Registered properties*:

    - `tau` (float, :py:attr:`tau`),

    - `eta` (float, :py:attr:`eta`),

    - `aperture` (float, :py:attr:`aperture`),

    - `resolution` (float, :py:attr:`resolution`),

    - `epsilon` (float, :py:attr:`epsilon`),

    - `barrier_angle` (float, :py:attr:`barrier_angle`)

    *State*: :py:class:`GeometricState`
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0) -> None:
        """
        Contruct a new instance

        :param kinematics:
            The kinematics

        :param radius:
            The radius
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...

    def get_collision_distance(self, arg0: bool,
                               arg1: float | None) -> list[float]:
        """
        Gets the free distance to collision in :math:`[-\\alpha, \\alpha]` at
        regular intervals.

        :param assuming_static:
            If True, all obstacles are assumed static.

        :param speed:
            The desired speed. Will be set to the last used target speed if
            not specified.

        :return:
            A vector of distances of size :py:attr:`resolution`. Angles are in
            the agent frame.
        """

    @property
    def angular_resolution(self) -> float:
        """
        Convenience method that return the size of an angular segment in
        :math:`[-\\alpha, \\alpha]` to search for optimal directions.
        """

    @property
    def aperture(self) -> float:
        """
        The aperture :math:`\\alpha`: desired velocity is searched on a
        circular sector :math:`[-\\alpha, \\alpha]`.
        """

    @aperture.setter
    def aperture(self, arg1: float) -> None:
        ...

    @property
    def barrier_angle(self) -> float:
        """
        The barrier angle, i.e., the minimal angle with respect to a
        currently virtually colliding obstacle to ignore it.
        """

    @property
    def epsilon(self) -> float:
        """
        The lowest margin to an obstacle or neighbor.

        any obstacles nearer than this this value will be virtually pushing
        away from the agent.
        """

    @property
    def eta(self) -> float:
        """
        The time :math:`\\eta` that the behavior keeps away from
        collisions. higher values lead to slower speeds.
        """

    @eta.setter
    def eta(self, arg1: float) -> None:
        ...

    @property
    def resolution(self) -> int:
        """
        The number of subdivision of :math:`[-\\alpha, \\alpha]` to search
        for optimal directions.
        """

    @resolution.setter
    def resolution(self, arg1: int) -> None:
        ...

    @property
    def tau(self) -> float:
        """
        The relaxation time :math:`\\tau`. higher values lead to lower
        accelerations.
        """

    @tau.setter
    def tau(self, arg1: float) -> None:
        ...


class HRVOBehavior(Behavior):
    """
    Hybrid Velocity Obstacle Behavior

    A wrapper of the open-source implementation from
    http://gamma.cs.unc.edu/HRVO/

    *Registered properties*: none

    *State*: :py:class:`GeometricState`
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0) -> None:
        """
        Contruct a new instance

        :param kinematics:
            The kinematics

        :param radius:
            The radius
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...


class HasProperties:
    """
    This class defines set and get methods to access named :py:class:`Property`
    "properties".
    """

    def get(
        self, name: str
    ) -> bool | int | float | str | numpy.ndarray | list[bool] | list[
            int] | list[float] | list[str] | list[numpy.ndarray]:
        """
        Gets the value of the specified property.

        :param name:
            The name of the property

        :raise:
            std::runtime_error A runtime error if no property is found.

        :return:
            The value of the property
        """

    def set(
        self, name: str,
        value: bool | int | float | str | numpy.ndarray | list[bool]
        | list[int] | list[float] | list[str] | list[numpy.ndarray]
    ) -> None:
        """
        Set the value of a named property. Fails silently if no property can
        be found or if the value has a non-compatible type.

        :param name:
            The name of the property

        :param value:
            The desired value for the property
        """

    @property
    def properties(self) -> dict[str, Property]:
        """
        All properties associated with an owner.
        """


class Kinematics(KinematicsRegister, HasProperties):
    """
    Abstract Kinematics type.

    A kinematics is used to

    - validated twist in the agent's frame as feasible

    - convert between wheel speeds and body twist

    - store maximal linear and angular speed

    - store the number of degrees of freedom
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self, max_speed: float, max_angular_speed: float = 0) -> None:
        ...

    def __setstate__(self, arg0: tuple) -> None:
        ...

    @typing.overload
    def feasible(self, arg0: Twist2) -> Twist2:
        """
        Computes the nearest feasible twist to a desired twist.

        :param twist:
            The desired twist

        :return:
            The same desired twist if feasible else the nearest feasible
            value. How this is defined depends on the concrete sub-class.
        """

    @typing.overload
    def feasible(self, arg0: Twist2, arg1: Twist2, time_step: float) -> Twist2:
        ...

    @property
    def cmd_frame(self) -> Frame:
        """
        The most natural frame for this kinematics: :py:meth:`Frame.relative` in
        case the agent is wheeled, else :py:meth:`Frame.absolute`.
        """

    @property
    def dof(self) -> int:
        """
        The degrees of freedom (between 0 and 3 for planar rigid body
        kinematics)
        """

    @property
    def is_wheeled(self) -> bool:
        """
        Whether the kinematics has wheels.
        """

    @property
    def max_angular_speed(self) -> float:
        """
        The maximal angular speed.
        """

    @max_angular_speed.setter
    def max_angular_speed(self, arg1: float) -> None:
        ...

    @property
    def max_speed(self) -> float:
        """
        The maximal speed.
        """

    @max_speed.setter
    def max_speed(self, arg1: float) -> None:
        ...

    @property
    def type(self) -> str:
        """
        The name associated to the type of an object.
        """


class KinematicsRegister:
    type_properties: typing.ClassVar[dict]
    types: typing.ClassVar[list] = ['2WDiff', '4WOmni', 'Ahead', 'Omni']

    @staticmethod
    def _add_property(arg0: str, arg1: str, arg2: typing.Any, arg3: bool | int
                      | float | str | numpy.ndarray | list[bool] | list[int]
                      | list[float] | list[str] | list[numpy.ndarray],
                      arg4: str, arg5: list[str]) -> None:
        ...

    @staticmethod
    def _register_type(arg0: str, arg1: typing.Any) -> None:
        ...

    @staticmethod
    def make_type(name: str) -> typing.Any:
        """
        Create an object of a sub-class selected by name.

        :param type:
            The associated type name.

        :return:
            An object of a registered sub-class or ``None`` in case the desired name is not found.
        """


class LineSegment:
    """
    A static obstacle of linear shape.
    """

    def __init__(self, p1: Vector2Like, p2: Vector2Like) -> None:
        """
        Constructs a new instance.

        :param p1:
            The first vertex

        :param p2:
            The second vertex
        """

    def __repr__(self) -> str:
        ...

    def distance_from_disc(self,
                           disc: Disc,
                           penetration: bool = False) -> float:
        ...

    def distance_from_point(self, point: Vector2Like) -> float:
        ...

    @property
    def e1(self) -> numpy.ndarray:
        """
        The unit vector along the segment
        """

    @property
    def e2(self) -> numpy.ndarray:
        """
        The unit vector perpendicular to the segment. Oriented to the left
        with respect to `e1`
        """

    @property
    def length(self) -> float:
        """
        The segment length
        """

    @property
    def p1(self) -> numpy.ndarray:
        """
        The position of the first vertex
        """

    @property
    def p2(self) -> numpy.ndarray:
        """
        The position of the second vertex
        """


class Neighbor(Disc):
    """
    A neighbor agent of circular shape.
    """

    def __init__(self,
                 position: Vector2Like,
                 radius: float,
                 velocity: Vector2Like = ...,
                 id: int = 0) -> None:
        """
        Constructs a new instance.

        :param position:
            The position

        :param radius:
            The radius

        :param velocity:
            The velocity

        :param id:
            The identifier
        """

    def __repr__(self) -> str:
        ...

    @property
    def id(self) -> int:
        """
        An identifier for the id of agents or the individual agent. The
        interpretation is up to the behavior.
        """

    @id.setter
    def id(self, arg0: int) -> None:
        ...

    @property
    def velocity(self) -> numpy.ndarray:
        """
        The velocity
        """

    @velocity.setter
    def velocity(self, arg0: Vector2Like) -> None:
        ...


class ORCABehavior(Behavior):
    """
    Optimal Reciprocal Collision Avoidance

    A wrapper of the open-source implementation from
    http://gamma.cs.unc.edu/RVO2/

    *Registered properties*:

    - `time_horizon` (float, :py:attr:`time_horizon`),

    - `effective_center` (float, :py:attr:`is_using_effective_center`),

    *State*: :py:class:`GeometricState`
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0) -> None:
        """
        Contruct a new instance

        :param kinematics:
            The kinematics

        :param radius:
            The radius
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...

    @property
    def is_using_effective_center(self) -> bool:
        """
        Determines if an effective center is being used.

        Using an effective center placed with an offset towards the front,
        allows to consider the kinematics as holonomic instead of a two-
        wheeled differential drive. See

        J. Snape, J. van den Berg, S. J. Guy, and D. Manocha, "Smooth and
        collision-free navigation for multiple robots under differential-drive
        constraints," in 2010 IEEE/RSJ International Conference on
        Intelligent, Robots and Systems, 2010, pp. 4584–4589.

        with ``D=L/2``.

        Only possibly true if the kinematics is wheeled and constrained.

        :return:
            True if using effective center, False otherwise.
        """

    @is_using_effective_center.setter
    def is_using_effective_center(self, arg1: bool) -> None:
        ...

    @property
    def time_horizon(self) -> float:
        """
        The time horizon. collisions predicted to happen after this time
        are ignored
        """

    @time_horizon.setter
    def time_horizon(self, arg1: float) -> None:
        ...


class OmnidirectionalKinematics(Kinematics):
    """
    Unconstrained kinematics (e.g., quad-copters)
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self, max_speed: float, max_angular_speed: float) -> None:
        """
        Constructs a new instance.

        :param max_speed:
            The maximal speed

        :param max_angular_speed:
            The maximal angular speed
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...


class Pose2:
    """
    Two-dimensional pose composed of planar position and orientation.
    Poses are assumed to be the world-fixed frame.
    """

    def __init__(self, position: Vector2Like, orientation: float = 0) -> None:
        ...

    def __repr__(self) -> str:
        ...

    def absolute(self, reference: Pose2) -> Pose2:
        """
        Transform a relative pose to an absolute pose.

        :param reference:
            The reference pose

        :return:
            The absolute pose
        """

    def integrate(self, twist: Twist2, time_step: float) -> Pose2:
        """
        Rotate the pose by an angle.

        :param angle:
            The rotation angle in radians.

        :return:
            The rotated pose.
        """

    def relative(self, reference: Pose2) -> Pose2:
        """
        Transform an absolute pose to a relative pose.

        :param pose:
            The reference pose

        :return:
            The relative pose
        """

    def rotate(self, angle: float) -> Pose2:
        """
        Rotate the pose by an angle.

        :param angle:
            The rotation angle in radians.

        :return:
            The rotated pose.
        """

    @property
    def orientation(self) -> float:
        """
        Orientation in world frame
        """

    @orientation.setter
    def orientation(self, arg0: float) -> None:
        ...

    @property
    def position(self) -> numpy.ndarray:
        """
        Position in world frame
        """

    @position.setter
    def position(self, arg0: Vector2Like) -> None:
        ...


class Property:
    """
    This class defines a property (similar to Python built-in properties),
    i.e., a pair of getter and setter. In addition, it holds (for auto-
    documentation) a default value, a description, and the name of the
    involved types. A property has a value of type :py:class:`PropertyField`, which is
    one of ``bool``, ``int``, ``float``, ``string``, :py:class:`Vector2` or a collection (a vector in C++, a list in
    Python) thereof. Properties are used to configure sub-classes of :py:class:`HasProperties` when using bindings like YAML or
    Python. Properties values are accessed using the methods exposed by
    :py:class:`HasProperties`.
    """

    @property
    def default_value(
        self
    ) -> bool | int | float | str | numpy.ndarray | list[bool] | list[
            int] | list[float] | list[str] | list[numpy.ndarray]:
        """
        The property default value, i.e., the value of the property when the
        object is initialized.
        """

    @property
    def deprecated_names(self) -> list[str]:
        """
        Alternative deprecated names for the property
        """

    @property
    def description(self) -> str:
        """
        A textual description of the property, used for auto documentation.
        """

    @property
    def owner_type_name(self) -> str:
        """
        The name of the property-owner type.
        """

    @property
    def type_name(self) -> str:
        """
        The name of the property value type, used for auto documentation.
        """


class SensingState(EnvironmentState):
    """
    Generic state to hold data from sensors in keyed buffers.
    """

    def __init__(self) -> None:
        """
        Construct an instance
        """

    def get_buffer(self, key: str) -> Buffer:
        """
        Gets the buffer assigned to a key

        :param key:
            The key

        :return:
            The buffer.
        """

    @typing.overload
    def init_buffer(
        self, key: str, description: BufferDescription,
        value: float | float | int | int | int | int | int | int | int | int
    ) -> Buffer:
        """
        Initializes a buffer with a description and a uniform value.

        :param key:
            The key

        :param desc:
            The description of the buffer

        :param value:
            The value to assign to the buffer

        :return:
            The buffer if successfully initialized else a null pointer
        """

    @typing.overload
    def init_buffer(self, key: str, description: BufferDescription) -> Buffer:
        """
        Initializes a buffer with description, with data set to zero.

        :param key:
            The key

        :param desc:
            The description of the buffer

        :return:
            The buffer if successfully initialized else a null pointer
        """

    @typing.overload
    def init_buffer(
        self, key: str, data: list[float] | list[float] | list[int] | list[int]
        | list[int] | list[int] | list[int] | list[int] | list[int] | list[int]
    ) -> Buffer:
        """
        Initializes a flat buffer with data.

        :param key:
            The key

        :param value:
            The data

        :return:
            The buffer if successfully initialized else a null pointer
        """

    def set_buffer(self, key: str, buffer: Buffer) -> None:
        """
        Assign a buffer to a key.

        :param key:
            The new value

        :param value:
            The value
        """

    @property
    def buffers(self) -> BufferMap:
        """
        The buffers.
        """


class SocialMargin:
    """
    This class defines a modulated social margin that may be added to the
    safety margin around :py:class:`Neighbor` for behaviors that uses the :py:class:`GeometricState`. The social margin is assigned to the neighbor's :py:attr:`Neighbor.id` and it is modulated by the neighbor current distance.
    """

    def __init__(self, value: float = 0) -> None:
        """
        Constructs a new instance.

        :param value:
            The default value of social margins
        """

    def get(self,
            type: int | None = None,
            distance: float | None = None) -> float:
        """
        Get the value, ignoring type and distance

        :return:
            The social margin
        """

    def set(self, value: float, type: int | None = None) -> None:
        """
        Set the default value, ignoring type.

        :param value:
            The desired social margin
        """

    @property
    def max_value(self) -> float:
        """
        The maximal possible value across all types
        """

    @property
    def modulation(self) -> SocialMarginModulation:
        """
        The modulation.
        """

    @modulation.setter
    def modulation(self, arg1: SocialMarginModulation) -> None:
        ...


class SocialMarginConstantModulation(SocialMarginModulation):
    """
    A modulation that leaves the margin unchanged
    """

    def __init__(self) -> None:
        ...


class SocialMarginLinearModulation(SocialMarginModulation):
    """
    A modulation that linearly interpolate between 0 and the input margin,
    returning the input margin above `upper_distance`
    """

    def __init__(self, upper_distance: float) -> None:
        """
        Constructs a new instance.

        :param upper_distance:
            The upper distance
        """


class SocialMarginLogisticModulation(SocialMarginModulation):
    """
    A logistic modulation.
    """

    def __init__(self) -> None:
        ...


class SocialMarginModulation:
    """
    Abstract modulation: maps a pair (margin, distance) to a margin.
    """

    def __call__(self, margin: float, distance: float | None = None) -> float:
        ...


class SocialMarginQuadraticModulation(SocialMarginModulation):
    """
    A modulation that quadratically interpolates between 0 and the input
    margin, returning the input margin above `upper_distance`. The slope
    is 0 at distance 0.
    """

    def __init__(self, upper_distance: float) -> None:
        ...


class SocialMarginZeroModulation(SocialMarginModulation):
    """
    A modulation that always return 0.
    """

    def __init__(self) -> None:
        ...


class Target:
    """
    """

    @staticmethod
    def Direction(direction: Vector2Like) -> Target:
        ...

    @staticmethod
    def Orientation(orientation: float, tolerance: float = 0) -> Target:
        ...

    @staticmethod
    def Point(point: Vector2Like, tolerance: float = 0) -> Target:
        ...

    @staticmethod
    def Pose(pose: Pose2,
             position_tolerance: float = 0,
             orientation_tolerance: float = 0) -> Target:
        ...

    @staticmethod
    def Stop(arg0: Vector2Like, arg1: float) -> Target:
        ...

    @staticmethod
    def Twist(twist: Twist2) -> Target:
        ...

    @staticmethod
    def Velocity(velocity: Vector2Like) -> Target:
        ...

    def __init__(self,
                 position: Vector2Like | None = None,
                 orientation: float | None = None,
                 speed: float | None = None,
                 direction: Vector2Like | None = None,
                 angular_speed: float | None = None,
                 position_tolerance: float = 0,
                 orientation_tolerance: float = 0) -> None:
        """
        Constructs a new instance.

        :param position:
            The position

        :param orientation:
            The orientation

        :param speed:
            The speed

        :param direction:
            The direction

        :param angular_speed:
            The angular speed

        :param position_tolerance:
            The position tolerance

        :param orientation_tolerance:
            The orientation tolerance
        """

    def __repr__(self) -> str:
        ...

    @typing.overload
    def satisfied(self, arg0: Vector2Like) -> bool:
        ...

    @typing.overload
    def satisfied(self, arg0: float) -> bool:
        ...

    @typing.overload
    def satisfied(self, arg0: Pose2) -> bool:
        ...

    @property
    def angular_speed(self) -> float | None:
        """
        The angular speed
        """

    @angular_speed.setter
    def angular_speed(self, arg0: float | None) -> None:
        ...

    @property
    def direction(self) -> numpy.ndarray | None:
        """
        The direction
        """

    @direction.setter
    def direction(self, arg0: Vector2Like | None) -> None:
        ...

    @property
    def orientation(self) -> float | None:
        """
        The orientation
        """

    @orientation.setter
    def orientation(self, arg0: float | None) -> None:
        ...

    @property
    def orientation_tolerance(self) -> float:
        """
        The orientation tolerance
        """

    @orientation_tolerance.setter
    def orientation_tolerance(self, arg0: float) -> None:
        ...

    @property
    def position(self) -> numpy.ndarray | None:
        """
        The position
        """

    @position.setter
    def position(self, arg0: Vector2Like | None) -> None:
        ...

    @property
    def position_tolerance(self) -> float:
        """
        The position tolerance
        """

    @position_tolerance.setter
    def position_tolerance(self, arg0: float) -> None:
        ...

    @property
    def speed(self) -> float | None:
        """
        The speed
        """

    @speed.setter
    def speed(self, arg0: float | None) -> None:
        ...

    @property
    def valid(self) -> bool:
        ...


class Twist2:
    """
    Two-dimensional twist composed of planar velocity and angular speed.

    Twist coordinates are in the frame specified by :py:attr:`frame`.
    """
    __hash__: typing.ClassVar[None] = None  # type: ignore

    def __eq__(self, arg0: Twist2) -> bool:  # type: ignore
        ...

    def __init__(self,
                 velocity: Vector2Like,
                 angular_speed: float = 0,
                 frame: Frame = ...) -> None:
        ...

    def __ne__(self, arg0: Twist2) -> bool:  # type: ignore
        ...

    def __repr__(self) -> str:
        ...

    def absolute(self, reference: Pose2) -> Twist2:
        """
        Transform a twist to :py:meth:`Frame.absolute`.

        :param value:
            The original twist

        :return:
            The same twist in :py:meth:`Frame.absolute` (unchanged if already in
            :py:meth:`Frame.absolute`)
        """

    def is_almost_zero(self,
                       epsilon_speed: float = 1e-06,
                       epsilon_angular_speed: float = 1e-06) -> bool:
        """
        Determines if almost zero.

        :param epsilon_speed:
            The epsilon speed

        :param epsilon_angular_speed:
            The epsilon angular speed

        :return:
            True if almost zero, False otherwise.
        """

    def relative(self, reference: Pose2) -> Twist2:
        """
        Transform a twist to :py:meth:`Frame.relative`.

        :param frame:
            The reference pose

        :return:
            The same twist in :py:meth:`Frame.relative` (unchanged if already in
            :py:meth:`Frame.relative`)
        """

    def rotate(self, angle: float) -> Twist2:
        """
        Rotate the twist by an angle.

        :param angle:
            The rotation angle in radians.

        :return:
            The rotated twist.
        """

    def snap_to_zero(self, epsilon: float = 1e-06) -> None:
        """
        Snap components to zero if their module is smaller than epsilon.

        :param epsilon:
            The tolerance
        """

    def interpolate(self, target: Twist2, time_step: float, max_acceleration: float, max_angular_acceleration: float) -> Twist2:
        ...

    @property
    def angular_speed(self) -> float:
        """
        Angular speed
        """

    @angular_speed.setter
    def angular_speed(self, arg0: float) -> None:
        ...

    @property
    def frame(self) -> Frame:
        """
        Frame of reference.
        """

    @frame.setter
    def frame(self, arg0: Frame) -> None:
        ...

    @property
    def velocity(self) -> numpy.ndarray:
        """
        Velocity
        """

    @velocity.setter
    def velocity(self, arg0: Vector2Like) -> None:
        ...


class TwoWheelsDifferentialDriveKinematics(WheeledKinematics):
    """
    Two differential drive wheels (left, right) (e.g., a wheelchair)
    """

    def __getstate__(self) -> tuple:
        ...

    def __init__(self, max_speed: float, axis: float) -> None:
        """
        Constructs a new instance.

        :param max_speed:
            The maximal wheel speed

        :param axis:
            The wheel axis (i.e., the distance between the wheels)
        """

    def __setstate__(self, arg0: tuple) -> None:
        ...


class DynamicTwoWheelsDifferentialDriveKinematics(
        TwoWheelsDifferentialDriveKinematics):

    def __getstate__(self) -> tuple:
        ...

    def __init__(self,
                 max_speed: float,
                 axis: float,
                 max_acceleration: float,
                 moi: float = 1) -> None:
        ...

    def __setstate__(self, arg0: tuple) -> None:
        ...

    @property
    def max_acceleration(self) -> float:
        ...

    @max_acceleration.setter
    def max_acceleration(self, arg1: float) -> None:
        ...

    @property
    def max_angular_acceleration(self) -> float:
        ...

    @max_angular_acceleration.setter
    def max_angular_acceleration(self, arg1: float) -> None:
        ...

    @property
    def moi(self) -> float:
        ...

    @moi.setter
    def moi(self, arg1: float) -> None:
        ...


class WheeledKinematics(Kinematics):
    """
    Abstract wheeled kinematics

    *Registered properties*:

    - `wheel_axis` (float, :py:attr:`axis`)
    """

    def __getstate__(self) -> tuple:
        ...

    def __setstate__(self, arg0: tuple) -> None:
        ...

    def twist(self, arg0: list[float]) -> Twist2:
        """
        Convert wheel speeds to a twist

        :param value:
            The wheel speeds

        :return:
            The corresponding twist
        """

    def wheel_speeds(self, arg0: Twist2) -> list[float]:
        """
        Convert a twist to wheel speeds

        :param value:
            The twist

        :return:
            The corresponding wheel speeds.
        """

    def feasible_wheel_speeds(self, arg0: Twist2) -> list[float]:
        ...

    @property
    def axis(self) -> float:
        """
        The wheel axis.
        """

    @axis.setter
    def axis(self, arg1: float) -> None:
        ...


def behavior_has_geometric_state(arg0: Behavior) -> bool:
    ...


@typing.overload
def dump(behavior: Behavior) -> str:
    """
    Dump a behavior to a YAML-string
    """


@typing.overload
def dump(kinematics: Kinematics) -> str:
    """
    Dump a kinematics to a YAML-string
    """


def load_behavior(value: str) -> typing.Any:
    """
    Load a behavior from a YAML string.

    :return:
      The loaded behavior or ``None`` if loading fails.
    """

def load_behavior_modulation(value: str) -> typing.Any:
    ...

def load_kinematics(value: str) -> typing.Any:
    """
    Load a kinematics from a YAML string.

    :return:
      The loaded kinematics or ``None`` if loading fails.
    """


def load_plugins(plugins: set[pathlib.Path] = set(),
                 directories: dict[set[pathlib.Path], set[pathlib.Path]] = {},
                 include_default: bool = True) -> None:
    ...


def to_absolute(value: Vector2Like, reference: Pose2) -> numpy.ndarray:
    """
    Transform a relative to an absolute vector.

    :param value:
        The relative vector

    :param reference:
        The reference pose

    :return:
        The relative vector
    """


def to_relative(value: Vector2Like, reference: Pose2) -> numpy.ndarray:
    """
    Transform an absolute to a relative vector.

    :param value:
        The absolute vector

    :param reference:
        The reference pose

    :return:
        The absolute vector
    """
