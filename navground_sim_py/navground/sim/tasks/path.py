from typing import cast
from navground import core, sim
import numpy as np
from navground.core import schema
import warnings


def get_path(points: list[core.Vector2]) -> core.Path | None:

    try:
        from shapely import geometry as g
    except ImportError:
        warnings.warn('Install shapely to use task \"Path\"', stacklevel=1)
        return None

    delta = np.diff(np.asarray(points), axis=0)
    ds = np.linalg.norm(delta, axis=-1)
    cs = np.cumsum(ds)
    cs = np.insert(cs, 0, 0)
    dx, dy = delta.T
    os = np.arctan2(dy, dx)
    os = np.insert(os, -1, os[-1])
    os = np.unwrap(os)
    ws = np.diff(os) / ds
    ws = np.insert(ws, 0, 0)
    line = g.LineString(points)

    def curve(s: float) -> tuple[core.Vector2, float, float]:
        if s < 0:
            return points[0], cs[0], 0
        if s > cs[-1]:
            return points[-1], cs[-1], 0
        i = np.searchsorted(cs, s, side="right")
        return np.asarray(line.interpolate(s).coords[0]), os[i - 1], ws[i - 1]

    def project(point: core.Vector2Like, a: float, b: float) -> float:
        if a > 0:
            i: int | np.int_ = np.searchsorted(cs, a, side="left")
            pa = [np.asarray(line.interpolate(a).coords[0])]
        else:
            i = 0
            pa = []
        if b < line.length:
            j: int | np.int_ = np.searchsorted(cs, b, side="right")
            pb = [np.asarray(line.interpolate(b).coords[0])]
        else:
            j = len(points)
            pb = []
        sline = g.LineString(pa + points[i:j] + pb)
        return sline.project(g.Point(point)) + a

    loop = cast(bool, np.linalg.norm(points[-1] - points[0]) < 1e-2)

    return core.Path(project, curve, cs[-1], loop)


class PathTask(sim.Task, name="Path"):
    """
       The task to follow a path defined by a list of points.

       *Registered properties*:

       - :py:attr:`points` (list[:py:type:`navground.core.Vector2`])
       - :py:attr:`tolerance` (float)
    """

    def __init__(self, points: list[core.Vector2] = [], tolerance: float = 1):
        """
        Constructs a new instance.

        :param      points:     The points defining the curve
        :param      tolerance:  The goal tolerance
        """
        super().__init__()
        self._points = points
        self._tolerance = tolerance

    @property
    @sim.register([], "points", schema.longer_than(1))
    def points(self) -> list[core.Vector2]:
        """
        :returns:    The points defining the curve
        """
        return self._points

    @points.setter
    def points(self, value: list[core.Vector2]) -> None:
        self._points = value

    @property
    @sim.register(1.0, "tolerance", schema.positive)
    def tolerance(self) -> float:
        """
        :returns:   The goal tolerance
        """
        return self._tolerance

    @tolerance.setter
    def tolerance(self, value: float) -> None:
        self._tolerance = max(0, value)

    def done(self) -> bool:
        return False

    @property
    def path(self) -> core.Path | None:
        """
        :returns:   The path or None if not enough points are provided
        """
        if len(self.points) > 1:
            return get_path(self.points)
        return None

    def prepare(self, agent: sim.Agent, world: sim.World) -> None:
        path = self.path
        if path:
            agent.controller.follow_path(path, self.tolerance)
