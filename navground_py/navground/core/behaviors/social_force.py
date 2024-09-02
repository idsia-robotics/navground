"""
    Helbing, Dirk, and Peter Molnar. 
    "Social force model for pedestrian dynamics." 
    Physical review E 51.5 (1995): 4282.

    https://arxiv.org/pdf/cond-mat/9805244

    Missing (not specified in the paper):
    - attractive forces
    - fluctuation
    - cit. "potentials with a hard core that would be more realistic"
    - gradient in case of divergence
"""

from typing import Callable, Optional, Tuple, cast

import numpy as np
from navground.core import (Behavior, Disc, GeometricState, Kinematics,
                            LineSegment, Neighbor, Vector2, register)

# TODO(Jerome): what about the radius and safety margin?


# value and gradient
# in the paper, called b
def neighbor_distance(position: Vector2, neighbor: Neighbor,
                      step_duration: float) -> Tuple[float, Vector2]:
    delta = position - neighbor.position
    step = neighbor.velocity * step_duration
    n1 = np.linalg.norm(delta, axis=-1)
    n2 = np.linalg.norm(delta - step, axis=-1)
    n3 = np.linalg.norm(step)
    # Can this be negative?
    b = 0.5 * np.sqrt((n1 + n2)**2 - n3**2)
    if b:
        grad = (0.5 / b * (2 * delta + 2 * (delta - step) + 0.5 * n1 / n2 *
                           (delta - step) + 0.5 * n2 / n1 * delta))
    else:
        # TODO
        grad = np.zeros(2)
    return b, grad


def disc_distance(position: Vector2, disc: Disc) -> Tuple[float, Vector2]:
    delta = position - disc.position
    distance = cast(float, np.linalg.norm(delta))
    if distance:
        grad = delta / distance
    else:
        grad = np.zeros(2)
    return distance, grad


def segment_distance(position: Vector2,
                     line: LineSegment) -> Tuple[float, Vector2]:
    delta = position - line.p1
    x = delta.dot(line.e1)
    if x < 0:
        distance = np.linalg.norm(delta)
        grad = delta / distance
    elif x > line.length:
        delta = position - line.p2
        distance = np.linalg.norm(delta)
        grad = delta / distance
    else:
        y = delta.dot(line.e2)
        distance = abs(delta.dot(line.e2))
        grad = line.e2 if y > 0 else -line.e2
    return cast(float, distance), grad


class Potential:
    """A monotonic decreasing function of the distance"""

    def __call__(self, x: float) -> Tuple[float, Vector2]:
        "Return the value and gradient of the potential"
        return 0.0, np.zeros(2)


class ExponentialPotential(Potential):
    """
    V(x) = a exp(-x/r)
    """

    def __init__(self, a: float, r: float):
        self.a = a
        self.r = r

    def __call__(self, x: float) -> Tuple[float, Vector2]:
        v = self.a * np.exp(-x / self.r)
        return v, -v / self.r


# TODO cit "Potentials with a hard core would be more realistic"


class SocialForceBehavior(Behavior, name="SocialForce"):
    """
    Basic social force algorithm from

        Helbing, Dirk, and Peter Molnar.
        "Social force model for pedestrian dynamics."
        Physical review E 51.5 (1995): 4282.

    *Registered properties*:

    - :py:attr:`tau` [float]
    - :py:attr:`phi` [float]
    - :py:attr:`c` [float]
    - :py:attr:`u_a` [float]
    - :py:attr:`u_r` [float]
    - :py:attr:`v_a` [float]
    - :py:attr:`v_r` [float]
    - :py:attr:`step_duration` [float]

    *State*: :py:class:`GeometricState`
    """

    @property
    @register(0.5, "Relaxation time")
    def tau(self) -> float:
        return self._tau

    @tau.setter
    def tau(self, value: float) -> None:
        self._tau = max(0, value)

    @property
    @register(1.0, "Step duration")
    def step_duration(self) -> float:
        return self._step_duration

    @step_duration.setter
    def step_duration(self, value: float) -> None:
        self._step_duration = max(0, value)

    @property
    @register(1.75, "Field of sight half-length")
    def phi(self) -> float:
        return float(np.arccos(self.cos_phi))

    @phi.setter
    def phi(self, value: float) -> None:
        self.cos_phi = np.cos(value)

    @property
    @register(0.5, "Weight of 'non-in-sight' forces")
    def c(self) -> float:
        return self._c

    @c.setter
    def c(self, value: float) -> None:
        self._c = max(0, value)

    @property
    @register(2.1, "Neighbors potential amplitude")
    def v_a(self) -> float:
        if isinstance(self.v, ExponentialPotential):
            return self.v.a
        return 0

    @v_a.setter
    def v_a(self, value: float) -> None:
        if isinstance(self.v, ExponentialPotential):
            self.v.a = value

    @property
    @register(0.3, "Neighbors potential length scale")
    def v_r(self) -> float:
        if isinstance(self.v, ExponentialPotential):
            return self.v.r
        return 0

    @v_r.setter
    def v_r(self, value: float) -> None:
        if isinstance(self.v, ExponentialPotential):
            self.v.r = max(0, value)

    @property
    @register(10, "Obstacles potential amplitude")
    def u_a(self) -> float:
        if isinstance(self.u, ExponentialPotential):
            return self.u.a
        return 0

    @u_a.setter
    def u_a(self, value: float) -> None:
        if isinstance(self.u, ExponentialPotential):
            self.u.a = value

    @property
    @register(0.2, "Obstacles potential length scale")
    def u_r(self) -> float:
        if isinstance(self.u, ExponentialPotential):
            return self.u.r
        return 0

    @u_r.setter
    def u_r(self, value: float) -> None:
        if isinstance(self.u, ExponentialPotential):
            self.u.r = max(0, value)

    def __init__(
            self,
            kinematics: Optional[Kinematics] = None,
            radius: float = 0.0,
            tau: float = 0.5,
            step_duration: float = 1.0,
            phi: float = 1.75,
            c: float = 0.5,
            v: Potential = ExponentialPotential(2.1, 0.3),
            u: Potential = ExponentialPotential(10, 0.2),
    ):
        Behavior.__init__(self, kinematics, radius)
        self._tau = tau
        self._step_duration = step_duration
        self.cos_phi = np.cos(phi)
        self._c = c
        self.v = v
        self.u = u
        self._state = GeometricState()

    def get_environment_state(self) -> GeometricState:
        return self._state

    def potential(self) -> Callable[[Vector2], float]:
        ps = ([
            self.neighbor_potential(neighbor)
            for neighbor in self._state.neighbors
        ] + [
            self.obstacle_potential(obstacle)
            for obstacle in self._state.static_obstacles
        ] + [
            self.segment_potential(line) for line in self._state.line_obstacles
        ])

        def f(position: Vector2) -> float:
            return sum(p(position) for p in ps)

        return f

    def neighbor_potential(self,
                           neighbor: Neighbor) -> Callable[[Vector2], float]:

        def f(position: Vector2) -> float:
            value, _ = neighbor_distance(position, neighbor,
                                         self.step_duration)
            return self.v(value)[0]

        return f

    def neighbor_repulsion_force(self, neighbor: Neighbor) -> Vector2:
        value, grad = neighbor_distance(self.position, neighbor,
                                        self.step_duration)
        _, p_grad = self.v(value)
        return -grad * p_grad

    def obstacle_potential(self, obstacle: Disc) -> Callable[[Vector2], float]:

        def f(position: Vector2) -> float:
            value, _ = disc_distance(position, obstacle)
            return self.u(value)[0]

        return f

    def obstacle_repulsion_force(self, obstacle: Disc) -> Vector2:
        value, grad = disc_distance(self.position, obstacle)
        _, p_grad = self.u(value)
        return -grad * p_grad

    def segment_potential(self,
                          line: LineSegment) -> Callable[[Vector2], float]:

        def f(position: Vector2) -> float:
            value, _ = segment_distance(position, line)
            return self.u(value)[0]

        return f

    def segment_repulsion_force(self, line: LineSegment) -> Vector2:
        value, grad = segment_distance(self.position, line)
        _, p_grad = self.u(value)
        return -grad * p_grad

    def in_sight(self, force: Vector2, e: Vector2) -> np.bool_:
        return np.all(np.dot(e, force) > self.cos_phi * np.linalg.norm(force))

    def weight(self, force: Vector2, e: Vector2) -> float:
        return 1.0 if self.in_sight(force, e) else self.c

    def weighted(self, force: Vector2, e: Vector2, sign: int) -> Vector2:
        return self.weight(sign * force, e) * force

    # TODO attractive effects
    # TODO fluctuations
    #
    def desired_velocity_towards_velocity(self, target_velocity: Vector2,
                                          time_step: float) -> Vector2:
        # acceleration towards desired velocity
        speed = np.linalg.norm(target_velocity)
        if not speed:
            return np.zeros(2)
        e = target_velocity / speed
        force = (target_velocity - self.velocity) / self.tau
        # repulsion from neighbors
        force += sum(
            self.weighted(self.neighbor_repulsion_force(neighbor), e, -1)
            for neighbor in self._state.neighbors)
        force += sum(
            self.weighted(self.obstacle_repulsion_force(obstacle), e, -1)
            for obstacle in self._state.static_obstacles)
        force += sum(
            self.weighted(self.segment_repulsion_force(line), e, -1)
            for line in self._state.line_obstacles)
        desired_velocity = self.actuated_twist.velocity + time_step * force
        # no need to clamp norm ... this will be done the superclass
        # are we sure?
        return desired_velocity

    def desired_velocity_towards_point(self, point: Vector2, speed: float,
                                       time_step: float) -> Vector2:
        # Target is in general an area,
        # then target position is the closed point in that area
        delta = point - self.position
        dist = np.linalg.norm(delta)
        if not dist:
            return np.zeros(2)
        velocity = delta / np.linalg.norm(delta) * speed
        return self.desired_velocity_towards_velocity(velocity, time_step)
