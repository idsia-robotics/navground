import warnings
from typing import Dict

import numpy as np
from navground import core, sim


class PyLidarStateEstimation(sim.Sensor, name="pyLidar"):  # type: ignore
    """
    Python implementation equivalent to the C++ Lidar implementation

    *Registered properties*:

    - :py:attr:`range` (float)
    - :py:attr:`field_of_view` (float)
    - :py:attr:`resolution` (int)
    - :py:attr:`range` (float)

    *State*: :py:class:`SensingState`
    """

    def __init__(self,
                 resolution: int = 11,
                 start_angle: float = -np.pi,
                 field_of_view: float = 2 * np.pi,
                 range_: float = 1.0):
        super().__init__()
        sim.StateEstimation.__init__(self)  # type: ignore
        self._cc = core.CollisionComputation()
        self._start_angle = start_angle
        self._field_of_view = field_of_view
        self._resolution = resolution
        self._range = range_

    @property
    @sim.register(-np.pi, "Start angle")
    def start_angle(self) -> float:
        return self._start_angle

    @start_angle.setter
    def start_angle(self, value: float) -> None:
        self._start_angle = value

    @property
    @sim.register(2 * np.pi, "Field of view")
    def field_of_view(self) -> float:
        return self._field_of_view

    @field_of_view.setter
    def field_of_view(self, value: float) -> None:
        self._field_of_view = max(0.0, value)

    @property
    @sim.register(4.0, "Range")
    def range(self) -> float:
        return self._range

    @range.setter
    def range(self, value: float) -> None:
        self._range = max(0.0, value)

    @property
    @sim.register(11, "Resolution")
    def resolution(self) -> int:
        return self._resolution

    @resolution.setter
    def resolution(self, value: int) -> None:
        self._resolution = max(1, value)

    def update(self, agent: sim.Agent, world: sim.World,
               state: core.EnvironmentState) -> None:
        self._cc.setup(pose=agent.pose,
                       margin=0.0,
                       static_discs=world.discs,
                       line_segments=world.line_obstacles)
        ranges = np.asarray(
            self._cc.get_free_distance_for_sector(
                agent.pose.orientation + self.start_angle,
                length=self.field_of_view,
                resolution=self.resolution - 1,
                max_distance=self.range,
                dynamic=False))
        try:
            state.set("range", ranges, True)  # type: ignore
        except (AttributeError, KeyError):
            warnings.warn(f"Cannot set field sensing of {state}")

    def get_description(self) -> Dict[str, core.BufferDescription]:
        desc = core.BufferDescription([self.resolution], float, 0.0,
                                      self.range)
        return {'range': desc}
