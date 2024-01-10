import asyncio
import json
import logging
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

import websockets
import websockets.server

from .. import Agent, Entity, Obstacle, Wall, World
from .to_svg import Rect, size

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
    return (*agent.position.tolist(), agent.orientation)


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
        'radius': obstacle.disc.radius
    }


def agent_msg(agent: Agent) -> EntityMsg:
    x, y = agent.position
    return {
        'kind': 'a',
        'type': agent.type,
        'size': agent.radius,
        'pose': pose_msg(agent),
        'color': agent.color
    }


class WebUI:
    """
    Displays the world on HTML client views.
    The state is synchronized through websockets.

    >>> # initialize and populate a world
    >>> world = ...
    >>> ui = WebUI()
    >>> # initialize the views with the current state of the world
    >>> await ui.init(world)
    >>> # update the world, for example with
    >>> world.update(0.1)
    >>> # synchronize the views
    >>> await ui.update(world)
    >>> # set entity attributes, for example to change the color of the first agent:
    >>> await ui.set(world.agents[0], style="fill:red")

    In case you are using WebUI in a notebook, you should call :py:class:`prepare`
    before instantiating the HTML view, else the HTML view will not connect
    to the websocket server:

    >>> ui = WebUI(host='127.0.0.1')
    >>> await ui.prepare()
    >>> notebook_view(width=250)

    >>> await ui.init(world)


    In general, users will not use this class directly but delegate updating the view
    to :py:class:`navground.sim.RealTimeSimulation` or
    :py:class:`navground.sim.RealTimeReplay`, like the following:

    >>> ui = WebUI(host='127.0.0.1')
    >>> sim = RealTimeSimulation(..., web_ui=ui)
    >>> await sim.run()
    """

    _instances: Dict[int, 'WebUI'] = {}

    def __init__(self,
                 host: str = '0.0.0.0',
                 port: int = 8000,
                 max_rate: float = 30,
                 display_deadlocks: bool = False,
                 display_collisions: bool = False) -> None:
        """
        Constructs a new instance.

        :param      host:                The host address serving the HTML page.
        :param      port:                The port serving the HTML page.
        :param      max_rate:            The maximum synchronization rate [fps]
        :param      display_deadlocks:   Whether to color deadlocked agents
        :param      display_collisions:  Whether to color agents in collision
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
        :returns:   The number of connected clients.
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
        _, _, view_box, _ = size(world=world, bounds=bounds)
        msg = ('v', view_box)
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def init(self, world: World, bounds: Optional[Rect] = None) -> None:
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

    async def update(self, world: World) -> None:
        if not self.queues:
            return
        stamp = time.time()
        if self.last_update_stamp and (
                stamp - self.last_update_stamp) < self.min_period:
            return
        self.last_update_stamp = stamp
        await self.update_poses(world)
        await self.update_colors(world)

    async def update_poses(self, world: World) -> None:
        msg = ['m', {a._uid: pose_msg(a) for a in world.agents}]
        await self.send(msg)

    async def update_colors(self, world: World) -> None:
        rs = {}
        if self.display_collisions:
            in_collision = set(
                [a._uid for a in world.get_agents_in_collision(1.0)])
            for e in in_collision - self.in_collision:
                rs[e] = {'style': 'fill:red'}
            for e in self.in_collision - in_collision:
                a = world.get_entity(e)
                if isinstance(a, Agent):
                    rs[e] = {'style': '' if not a.color else f'fill:{a.color}'}
            self.in_collision = in_collision
        if self.display_deadlocks:
            in_deadlock = set(
                [a._uid for a in world.get_agents_in_deadlock(5.0)])
            for e in in_deadlock - self.in_deadlock:
                if e not in rs:
                    rs[e] = {'style': 'fill:blue'}
            for e in self.in_deadlock - in_deadlock:
                a = world.get_entity(e)
                if isinstance(a, Agent):
                    rs[e] = {'style': '' if not a.color else f'fill:{a.color}'}
            self.in_deadlock = in_deadlock
        if rs:
            await self.send(['s', rs])

    async def set(self, entity: Entity, **kwargs) -> None:
        msg = ['s', {entity._uid: kwargs}]
        await self.send(msg)

    async def send(self, msg: Any) -> None:
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def set_background_color(self, color: str) -> None:
        msg = ['s', {'svg': {'style': f"background-color:{color}"}}]
        await self.send(msg)

    async def stop(self) -> None:
        if self.server:
            r = self.server.close()
            if r:
                await r
        self.server = None

    def add_callback(self, cb: Callback) -> None:
        self._callbacks.append(cb)

    def remove_callback(self, cb: Callback) -> None:
        self._callbacks.remove(cb)
