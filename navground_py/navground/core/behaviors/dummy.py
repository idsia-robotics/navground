from typing import Optional

import numpy as np
from navground.core import (Behavior, Kinematics, Vector2,
                            registered_property)


class PyDummyBehavior(Behavior, name="PyDummy"):
    """
    Dummy Behavior that ignore obstacles instead of avoiding them.
    Equivalent to the C++ class navground::core::DummyBehavior.
    Implemented to demonstrate that sub-classing Behavior works in Python

    *Registered properties*:

        - :py:attr:`dummy` [bool]
        - :py:attr:`tired` [bool]

    """

    # Not needed ... defined to have a more complete template
    def __init__(self,
                 kinematics: Optional[Kinematics] = None,
                 radius: float = 0.0):
        Behavior.__init__(self, kinematics, radius)
        self._tired = False

    @registered_property(True, "Am I dummy?")
    def dummy(self) -> bool:
        return True

    @registered_property(False, "Am I tired?")
    def tired(self) -> bool:
        return self._tired

    @tired.setter
    def tired(self, value: bool) -> None:
        self._tired = value

    def desired_velocity_towards_velocity(self, velocity: Vector2,
                                          time_step: float) -> Vector2:
        return velocity

    def desired_velocity_towards_point(self, point: Vector2, speed: float,
                                       time_step: float) -> Vector2:
        delta = point - self.pose.position
        distance = np.linalg.norm(delta)
        if distance:
            return self.desired_velocity_towards_velocity(
                speed * delta / distance, time_step)
        return (0, 0)
