from __future__ import annotations

import numpy as np
from navground.core import Behavior, Kinematics, Vector2, register


class PyDummyBehavior(Behavior, name="PyDummy"):
    """
    Dummy Behavior that ignore obstacles instead of avoiding them.
    Equivalent to the C++ class :cpp:class:`navground::core::DummyBehavior`.
    Implemented to demonstrate that sub-classing :py:class:`Behavior` works in Python

    *Registered properties*:

    - :py:attr:`dummy` [bool] (deprecated synonyms: ``not_so_smart``)
    - :py:attr:`tired` [bool]

    *State*: none

    """

    # Not needed ... defined to have a more complete template
    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0.0):
        super().__init__(kinematics, radius)
        self._tired = False

    @property
    @register(True, "Am I dummy?", None, ["not_so_smart"])
    def dummy(self) -> bool:
        return True

    @property
    @register(False, "Am I tired?")
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
        return np.zeros(2)
