import asyncio
import cv2 as cv
from cv2 import aruco
from mavsdk import System

drone = System()
sysAddr = "udp://:14550"

async def setup():
    await drone.connect(system_address=sysAddr)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone connected at " + sysAddr)
            break


async def main():
    
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)

    if vidCap.isOpened() == False:
        print("Could not open video stream")

    while vidCap.isOpened():
        ret, frame = vidCap.read()

        await printPosition()

        if ret == True:
            (markerCorners, markerIds, rejected) = cv.aruco.detectMarkers(frame, arucoDict)#, parameters=arucoParams)
            if len(markerCorners) > 0:

                cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
                markerIds = markerIds.flatten()
                for (markerCorner, markerID) in zip(markerCorners, markerIds):
                    markerCorners = markerCorner.reshape((4, 2))
                    print("[INFO] Detected marker: {}".format(markerID))

            cv.imshow("Frame", frame)

            key = cv.waitKey(3) & 0xFF
            if key == ord('q'):
                break
        else:
            break

async def printPosition(drone=drone):
    async for position in drone.telemetry.position():
        print(position)


if __name__ == "__main__":
    vidCap = cv.VideoCapture(0)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(setup())
    loop.run_until_complete(main())

    vidCap.release()
    cv.destroyAllWindows()
