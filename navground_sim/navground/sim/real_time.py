import asyncio
import logging
import threading
import time
from typing import Optional

from . import World

from .recording import Run
from .ui.web_ui import Rect, WebUI


class RealTimeSimulation:

    def __init__(self,
                 world: World,
                 time_step: float,
                 factor: float = 1.0,
                 web_ui: Optional[WebUI] = None,
                 bounds: Optional[Rect] = None):
        self.world = world
        self.web_ui = web_ui
        self.time_step = time_step
        self.factor = factor
        self.slept = 0.0
        self.total = 0.0
        self._initialized = False
        self._stop = False
        self.bounds = bounds
        self.thread: Optional[threading.Thread] = None

    def stop(self):
        self._stop = True

    async def init(self):
        if self.web_ui:
            await self.web_ui.init(self.world)
            await self.web_ui.update_world_size(self.world, bounds=self.bounds)
        self._initialized = True
        self._stop = False

    def _run_in_event_loop(self, loop, until=None):
        asyncio.run_coroutine_threadsafe(self.run(until=until), loop)

    def run_threaded(self, until=None) -> None:
        if self.thread and self.thread.is_alive():
            logging.error('Already running a thread')
            return
        self.thread = threading.Thread(target=self._run_in_event_loop,
                                       args=(asyncio.get_event_loop(), until))
        self.thread.start()

    async def run(self, until=None):
        self._stop = False
        if not self._initialized:
            await self.init()
        start_time = time.monotonic()
        target_time = start_time
        while (not self._stop and (not until or not until())):
            target_time += self.time_step / self.factor
            if not await self.step():
                break
            sleep_time = max(target_time - time.monotonic(), 0.0)
            self.slept += sleep_time
            await asyncio.sleep(sleep_time)
        self.total += time.monotonic() - start_time

    def _step(self) -> bool:
        self.world.run(1, self.time_step)
        return True

    async def step(self):
        if not self._initialized:
            await self.init()
        r = self._step()
        if self.web_ui:
            await self.web_ui.update_poses(self.world)
        return r


class RealTimeReplay(RealTimeSimulation):

    def __init__(self,
                 run: Run,
                 factor: float = 1.0,
                 web_ui: Optional[WebUI] = None):
        self._run = run
        super().__init__(world=run.world,
                         time_step=run.time_step,
                         web_ui=web_ui,
                         factor=factor,
                         bounds=run.bounds)

    def _step(self) -> bool:
        return self._run.do_step()
