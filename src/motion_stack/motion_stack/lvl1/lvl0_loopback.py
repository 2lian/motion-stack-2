import asyncio
import time
from copy import deepcopy
from typing import Any, AsyncGenerator, AsyncIterator, Coroutine

import asyncio_for_robotics as afor
import colorama
from asyncio_for_robotics import BaseSub

from motion_stack.utils.joint_state import JState, JStateBuffer
from motion_stack.utils.time import Time

from .core import JointCore, JStateBatch


class LoopBack:
    def __init__(self, joint_core: JointCore) -> None:
        self.core = joint_core
        self.lvl1_input: BaseSub[JStateBatch] = self.core.sensor_sub
        self.lvl1_ouput: BaseSub[JStateBatch] = self.core.motor_output
        self.states: JStateBuffer = JStateBuffer(JState(""))

    def run(self) -> Coroutine[Any, Any, None]:
        i = self.lvl1_ouput.listen()

        async def _run():
            async with asyncio.TaskGroup() as tg:
                tg.create_task(self._consume(i))
                tg.create_task(self._send_back())

        return _run()

    async def _consume(self, iterator: AsyncGenerator[JStateBatch, None]):
        async for msg in iterator:
            names = msg.keys()
            already_there = self.states.accumulated.keys()
            new = names - already_there
            if len(new) > 0:
                print(
                    f"New {colorama.Fore.RED}SIMULATED{colorama.Fore.RESET} joints using loopback: {colorama.Fore.BLUE}{new}{colorama.Fore.RESET}"
                )
            self.states.push(msg)

    async def _send_back(self):
        rate = afor.Rate(60)
        try:
            async for t_ns in rate.listen():
                js = deepcopy(self.states.accumulated)
                stamp = Time.from_parts(nano=time.time_ns())
                for name, j in js.items():
                    j.time = stamp
                    if j.position is None:
                        j.position = 0
                self.lvl1_input._input_data_asyncio(js)
        finally:
            rate.close()
