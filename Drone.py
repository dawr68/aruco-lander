import asyncio
from pickle import NONE
from main import SYS_ADDR
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)


class Drone:
    def __init__(self):
        self.system = System()
        self.altitude = -1.0
        self.terminationTask = None

    async def connect(self, sysAddr):
        await self.system.connect(system_address=sysAddr)
        async for state in self.system.core.connection_state():
            if state.is_connected:
                break
        altTask = asyncio.create_task(self.getAltitude())
        self.terminationTask = asyncio.ensure_future(self.observeIsInAir([altTask]))

    async def startOffbaord(self):
        await self.system.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await self.system.offboard.start()
        except OffboardError as err:
            await self.system.action.land()
            return 1

    async def stopOffboard(self):
        try:
            await self.system.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: {error._result.result}")

    async def setOffboardVelocity(self, velocity):
        await self.system.offboard.set_velocity_body(velocity)

    async def touchdown(self):
        await self.system.action.land()
        await self.terminationTask

    async def getAltitude(self):
        async for position in self.system.telemetry.position():
            self.altitude = position.relative_altitude_m

    async def observeIsInAir(self, runningTasks):
        wasInAir = False

        async for isInAir in self.system.telemetry.in_air():
            if isInAir:
                wasInAir = isInAir

            if wasInAir and not isInAir:
                for task in runningTasks:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
                await asyncio.get_event_loop().shutdown_asyncgens()
                return
