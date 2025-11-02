from __future__ import annotations

from typing import SupportsFloat

import numpy as np
from navground.core import Behavior, Kinematics, Vector2, Vector2Like, zeros2


class PyDummyBehavior(Behavior, name="PyDummy"):
    """
    Dummy Behavior that ignore obstacles instead of avoiding them.
    Equivalent to the C++ class :cpp:class:`navground::core::DummyBehavior`.
    Implemented to demonstrate that sub-classing :py:class:`Behavior` works in Python

    *State*: none
    """

    def __init__(self,
                 kinematics: Kinematics | None = None,
                 radius: float = 0.0):
        Behavior.__init__(self, kinematics, radius)
        self._tired = False

    def desired_velocity_towards_velocity(self, velocity: Vector2Like,
                                          time_step: SupportsFloat) -> Vector2:
        """
        Returns the same velocity.

        :param      velocity:   The velocity
        :param      time_step:  The time step

        :returns:   The velocity
        """
        return np.asarray(velocity)

    def desired_velocity_towards_point(self, point: Vector2Like,
                                       speed: SupportsFloat,
                                       time_step: SupportsFloat) -> Vector2:
        """
        Returns a velocity headed towards the point with
        the given speed.

        :param      point:      The point
        :param      speed:      The speed
        :param      time_step:  The time step

        :returns:   The velocity
        """
        delta = np.asarray(point) - self.pose.position
        distance = np.linalg.norm(delta)
        if distance:
            return self.desired_velocity_towards_velocity(
                float(speed) * delta / distance, float(time_step))
        return zeros2()
