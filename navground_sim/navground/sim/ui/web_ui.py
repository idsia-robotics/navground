import asyncio
import itertools
import json
import logging
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

import websockets
import websockets.server

from .. import Agent, Entity, Obstacle, Wall, World
from .to_svg import (Attributes, Decorate, Rect, bounds_for_world, flat_dict,
                     size)

PoseMsg = Tuple[float, float, float]
EntityMsg = Dict[str, Any]

Callback = Callable[[Any], None]


async def consumer_handler(
        websocket: websockets.server.WebSocketServerProtocol,
        callbacks: List[Callback]) -> None:
    try:
        async for msg in websocket:
            data = json.loads(msg)
            for cb in callbacks:
                cb(data)
    except websockets.exceptions.ConnectionClosedError:
        pass


async def producer_handler(
        websocket: websockets.server.WebSocketServerProtocol,
        queue: asyncio.Queue) -> None:
    while True:
        msg = await queue.get()
        try:
            await websocket.send(msg)
        except websockets.exceptions.ConnectionClosed:
            pass


def pose_msg(agent: Agent) -> PoseMsg:
    return (*agent.position.tolist(), float(agent.orientation))


def wall_msg(wall: Wall) -> EntityMsg:

    return {
        'kind': 'w',
        'points': [wall.line.p1.tolist(),
                   wall.line.p2.tolist()]
    }


def obstacle_msg(obstacle: Obstacle) -> EntityMsg:

    return {
        'kind': 'o',
        'point': obstacle.disc.position.tolist(),
        'radius': float(obstacle.disc.radius)
    }


def agent_msg(agent: Agent) -> EntityMsg:
    x, y = agent.position
    return {
        'kind': 'a',
        'type': agent.type,
        'size': float(agent.radius),
        'pose': pose_msg(agent),
        'color': agent.color
    }


class WebUI:
    """
    Displays the world on HTML client views.
    The state is synchronized through websockets.

    :ivar decorate: A function to decorate entities, called at each update
    :vartype decorate: :py:class:`typing.Mapping[str, str]`

    After creating an instance, you need to call :py:meth:`prepare`
    to finalize the initialization of the websocket server,
    else the HTML clients will not be able to connect.

    >>> ui = WebUI()
    >>> await ui.prepare()
    >>> # initialize the views with the current state of the world
    >>> world = ...
    >>> await ui.init(world)
    >>> # update the world, for example with
    >>> world.update(0.1)
    >>> # synchronize the views
    >>> await ui.update(world)
    >>> # set entity attributes, for example to change the color of the first agent
    >>> await ui.set(world.agents[0], style="fill:red")
    >>> # or analogously
    >>> await ui.set_color(world.agents[0], "red")

    In general, users will not use this class directly but delegate updating the view
    to :py:class:`navground.sim.RealTimeSimulation` or
    :py:class:`navground.sim.RealTimeReplay`, like the following:

    >>> ui = WebUI(host='127.0.0.1')
    >>> await ui.prepare()
    >>> sim = RealTimeSimulation(..., web_ui=ui)
    >>> await sim.run()
    """

    _instances: Dict[int, 'WebUI'] = {}

    def __init__(self,
                 host: str = '0.0.0.0',
                 port: int = 8000,
                 max_rate: float = 30,
                 display_deadlocks: bool = False,
                 display_collisions: bool = False,
                 decorate: Optional[Decorate] = None) -> None:
        """
        Constructs a new instance.

        :param      host:                The host address serving the HTML page.
        :param      port:                The port serving the HTML page.
        :param      max_rate:            The maximum synchronization rate [fps]
        :param      display_deadlocks:   Whether to color deadlocked agents
        :param      display_collisions:  Whether to color agents in collision
        :param      decorate:            A function to decorate entities, called at each update
        """
        self.port = port
        self.host = host
        self.queues: List[asyncio.Queue] = []
        self._callbacks: List[Callback] = []
        if max_rate > 0:
            self.min_period = 1 / max_rate
        else:
            self.min_period = 0
        self.last_update_stamp: Optional[float] = None
        self._prepared = False
        self.display_collisions = display_collisions
        self.display_deadlocks = display_deadlocks
        self.in_collision: Set[int] = set()
        self.in_deadlock: Set[int] = set()
        self.decorate = decorate
        self.server: Optional[websockets.WebSocketServer] = None

    @property
    def is_ready(self) -> bool:
        return self._prepared

    async def prepare(self) -> bool:
        """
        Initialize the websockets server

        :returns:   If the server could be initialized
        """
        if not self._prepared:
            try:
                self.server = await websockets.server.serve(
                    self.handle_ws, self.host, self.port)
            except OSError as e:
                print(e, file=sys.stderr)
                return False
            self._prepared = True
        return True

    @property
    def number_of_client(self) -> int:
        """
        :returns:   The number of connected client views.
        """
        return len(self.queues)

    async def handle_ws(self,
                        websocket: websockets.server.WebSocketServerProtocol,
                        path: str,
                        port: int = 8000) -> None:
        logging.info('Websocket connection opened')
        queue: asyncio.Queue = asyncio.Queue()
        self.queues.append(queue)
        consumer_task = asyncio.ensure_future(
            consumer_handler(websocket, self._callbacks))
        producer_task = asyncio.ensure_future(
            producer_handler(websocket, queue))
        done, pending = await asyncio.wait([consumer_task, producer_task],
                                           return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        self.queues.remove(queue)
        logging.info('Websocket connection closed')

    def __new__(cls, *args: Any, **kwargs: Any) -> 'WebUI':
        # Is a singleton
        port = kwargs.get('port', 8000)
        if port not in cls._instances:
            cls._instances[port] = object.__new__(cls)
        return cls._instances[port]

    async def update_size(self,
                          world: Optional[World] = None,
                          bounds: Optional[Rect] = None) -> None:
        if bounds is None and world is not None:
            bounds = bounds_for_world(world)
        if bounds is None:
            return
        _, _, view_box, _ = size(bounds=bounds)
        msg = ('v', view_box)
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def init(self, world: World, bounds: Optional[Rect] = None) -> None:
        """
        Initialize the client views to display a world

        :param      world:   The world to be displayed
        :param      bounds:  The area to display. If not set, it will displays
                             an area that contains all world entities
                             at their current positions
        """
        reset_msg = ('r', None)
        entities = {}
        for wall in world.walls:
            entities[wall._uid] = wall_msg(wall)
        for obstacle in world.obstacles:
            entities[obstacle._uid] = obstacle_msg(obstacle)
        for agent in world.agents:
            entities[agent._uid] = agent_msg(agent)
        msgs = [reset_msg, ['+', entities]]
        for msg in msgs:
            data = json.dumps(msg)
            for queue in self.queues:
                await queue.put(data)
        await self.update_size(world=world, bounds=bounds)
        await self.update_style(world)

    async def update(self, world: World) -> None:
        """
        Synchronized all client views with the current world state.

        :param      world:  The world
        :type       world:  World

        :returns:   { description_of_the_return_value }
        :rtype:     None
        """
        if not self.queues:
            return
        stamp = time.time()
        if self.last_update_stamp and (
                stamp - self.last_update_stamp) < self.min_period:
            return
        self.last_update_stamp = stamp
        await self.update_poses(world)
        await self.update_style(world)

    async def update_poses(self, world: World) -> None:
        msg = ['m', {a._uid: pose_msg(a) for a in world.agents}]
        await self.send(msg)

    def compute_style(self, world: World) -> Dict[int, Attributes]:
        rs: Dict[int, Attributes] = {}
        if self.decorate:
            for e in itertools.chain(world.walls, world.obstacles,
                                     world.agents):
                r = self.decorate(e)
                if r:
                    rs[e._uid] = r
        if self.display_collisions:
            in_collision = set(
                [a._uid for a in world.get_agents_in_collision(1.0)])
            for e_id in in_collision - self.in_collision:
                if e_id not in rs:
                    rs[e_id] = {}
                rs[e_id]['fill'] = 'red'
            for e_id in self.in_collision - in_collision:
                a = world.get_entity(e_id)
                if isinstance(a, Agent):
                    if e_id not in rs:
                        rs[e_id] = {}
                    rs[e_id]['fill'] = a.color
            self.in_collision = in_collision
        if self.display_deadlocks:
            in_deadlock = set(
                [a._uid for a in world.get_agents_in_deadlock(5.0)])
            for e_id in in_deadlock - self.in_deadlock:
                if e_id not in rs:
                    rs[e_id] = {}
                rs[e_id]['fill'] = 'blue'
            for e_id in self.in_deadlock - in_deadlock:
                a = world.get_entity(e_id)
                if isinstance(a, Agent):
                    if e_id not in rs:
                        rs[e_id] = {}
                    rs[e_id]['fill'] = a.color
            self.in_deadlock = in_deadlock
        return rs

    async def update_style(self, world: World) -> None:
        rs = self.compute_style(world)
        if rs:
            ds = {e: flat_dict(v) for e, v in rs.items()}
            await self.send(['s', ds])

    async def set(self, entity: Entity, **kwargs: Any) -> None:
        """
        Set attributes for an entity

        :param      entity:  The entity
        :param      kwargs:  The attributes (string-valued)
        """
        msg = ['s', {entity._uid: kwargs}]
        await self.send(msg)

    async def set_color(self, entity: Entity, color: str) -> None:
        """
        Sets the color of an entity.

        :param      entity:  The entity
        :param      color:   The color
        """
        await self.set(entity, style=f'fill:{color}')

    async def send(self, msg: Any) -> None:
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def set_background_color(self, color: str) -> None:
        """
        Sets the background color.

        :param      color:  The color
        """
        msg = ['s', {'svg': {'style': f"background-color:{color}"}}]
        await self.send(msg)

    async def stop(self) -> None:
        logging.info('Stopping UI')
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        self.server = None
        self._prepared = False
        logging.info('Stopped')

    def add_callback(self, cb: Callback) -> None:
        self._callbacks.append(cb)

    def remove_callback(self, cb: Callback) -> None:
        self._callbacks.remove(cb)

    def __del__(self):
        if self.server:
            self.server.close()
