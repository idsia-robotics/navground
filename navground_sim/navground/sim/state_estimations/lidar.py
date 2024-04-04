import warnings
from typing import Dict

import numpy as np
from navground import core, sim


class PyLidarStateEstimation(sim.Sensor, sim.StateEstimation,
                             name="pyLidar"):  # type: ignore
    """
    Python implementation equivalent to the C++ Lidar implementation

    *Registered properties*:

    - :py:attr:`start_angle` (float)
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
        sim.StateEstimation.__init__(self)
        self._cc = core.CollisionComputation()
        self.start_angle = start_angle
        self.field_of_view = field_of_view
        self.resolution = resolution
        self.range = range_

    @property
    @sim.register(-np.pi, "Start angle")
    def start_angle(self) -> float:
        return self._start_angle

    @start_angle.setter
    def start_angle(self, value: float) -> None:
        self._start_angle = value

    @property
    @sim.register(2 * np.pi, "Length")
    def length(self) -> float:
        return self._length

    @length.setter
    def length(self, value: float) -> None:
        self._length = max(0.0, value)

    @property
    @sim.register(4.0, "Range")
    def max_distance(self) -> float:
        return self._range

    @max_distance.setter
    def max_distance(self, value: float) -> None:
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
                length=self.length,
                resolution=self.resolution - 1,
                max_distance=self.max_distance,
                dynamic=False))
        try:
            state.set("range", ranges, True)  # type: ignore
        except (AttributeError, KeyError):
            warnings.warn(f"Cannot set field sensing of {state}")

    def get_description(self) -> Dict[str, core.BufferDescription]:
        desc = core.BufferDescription([self.resolution], float, 0.0,
                                      self.max_distance)
        return {'range': desc}
