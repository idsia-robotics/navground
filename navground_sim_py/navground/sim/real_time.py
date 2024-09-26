import asyncio
import logging
import threading
import time
from typing import Optional, Callable, TYPE_CHECKING

from . import World

if TYPE_CHECKING:
    from .ui.web_ui import Rect, WebUI


Callback = Callable[[], bool]


class RealTimeSimulation:
    """
    Helper class to simulated a :py:class:`navground.sim.World`
    in real-time using an ayncio.

    >>> word = ...
    >>> sim = RealTimeSimulation(world=world, time_step=0.1, web_ui=WebUI())
    >>> await sim.run()

    """

    def __init__(self,
                 world: World,
                 time_step: float,
                 factor: float = 1.0,
                 web_ui: Optional['WebUI'] = None,
                 bounds: Optional['Rect'] = None):
        """
        Constructs a new instance.

        :param      world:      The world to simulate
        :param      time_step:  The time step to apply
        :param      factor:     The real-time factor
        :param      web_ui:     An optional web user interface to sync with
        :param      bounds:     The region to display in the web_ui.
                                If not set, it will estimate it from
                                the current world state.
        """
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
        """
        Signals to stop the simulation
        """
        self._stop = True

    async def init(self):
        """
            Initializes the simulation and the UI
        """
        if self.web_ui:
            await self.web_ui.init(self.world, bounds=self.bounds)
        self._initialized = True
        self._stop = False

    def _run_in_event_loop(self, loop, until=None):
        asyncio.run_coroutine_threadsafe(self.run(until=until), loop)

    def run_threaded(self, until: Optional[Callback]) -> None:
        """
        Run a simulation in a thread.
        Blocks until the simulation is done.

        :param      until:  an optional condition to terminate the simulation

        """
        if self.thread and self.thread.is_alive():
            logging.error('Already running a thread')
            return
        self.thread = threading.Thread(target=self._run_in_event_loop,
                                       args=(asyncio.get_event_loop(), until))
        self.thread.start()

    async def run(self, until: Optional[Callback] = None) -> None:
        """
        Simulate a world.

        :param      until:  an optional condition to terminate the simulation
        """
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

    async def update(self) -> None:
        """
        Updates the UI with the world state.
        """
        if not self._initialized:
            await self.init()
        if self.web_ui:
            # TODO(Jerome): may fail
            await self.web_ui.prepare()
            await self.web_ui.update(self.world)

    async def step(self) -> bool:
        """
        Perform a simulation step.

        :returns:  Whether the step could be performed or not.
        """
        if not self._initialized:
            await self.init()
        r = self._step()
        if self.web_ui:
            await self.web_ui.update(self.world)
        return r
