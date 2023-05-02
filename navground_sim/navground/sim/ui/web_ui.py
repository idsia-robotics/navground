import asyncio
import json
import logging
import time
from typing import Any, Callable, Dict, List, Optional, Tuple
import sys

import websockets
import websockets.server

from .. import Agent, Obstacle, Wall, World, Entity
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
        'pose': pose_msg(agent)
    }


class WebUI:

    _instances: Dict[int, 'WebUI'] = {}

    def __init__(self,
                 host: str = '0.0.0.0',
                 port: int = 8000,
                 max_rate: float = 30) -> None:
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

    @property
    def is_ready(self) -> bool:
        return self._prepared

    async def prepare(self) -> bool:
        if not self._prepared:
            try:
                self.server = await websockets.server.serve(self.handle_ws, self.host,
                                                            self.port)
            except OSError as e:
                print(e, file=sys.stderr)
                return False
            self._prepared = True
        return True

    @property
    def number_of_client(self) -> int:
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

    async def update_world_size(self,
                                world: World,
                                bounds: Optional[Rect] = None) -> None:
        _, _, view_box, _ = size(world, bounds=bounds)
        msg = ('v', view_box)
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def init(self, world: World) -> None:
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

    async def update_poses(self, world: World) -> None:
        if not self.queues:
            return
        stamp = time.time()
        if self.last_update_stamp and (
                stamp - self.last_update_stamp) < self.min_period:
            return
        self.last_update_stamp = stamp
        msg = ['m', {a._uid: pose_msg(a) for a in world.agents}]
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def set(self, entity: Entity, **kwargs) -> None:
        msg = ['s', {entity._uid: kwargs}]
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

    async def set_background_color(self, color: str) -> None:
        msg = ['s', {'svg': {'style': f"background-color:{color}"}}]
        data = json.dumps(msg)
        for queue in self.queues:
            await queue.put(data)

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
