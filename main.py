#!/usr/bin/env python3.8
from __future__ import print_function

import roslib
roslib.load_manifest('aruco')
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import asyncio
import cv2
from mavsdk.offboard import VelocityBodyYawspeed

import ArucoFinder
import Drone
import ImageConverter

SYS_ADDR = "udp://:14445" #"udp://:14550"
LAND_ALT = 0.8 #Altitude at which auto-land is performed
MAX_ALT = 10.0 #Maximal altitute at which auto-landing can be performed
DESC_VELO = 0.2 #Descending velocity [m/s]
HOR_VELO = 1 #Maximal horizontal velocity [m/s]
ARUCO_ID = 68 #ID of aruco that should be detecte
THREASHOLD = 0.5 #Threashold for descending after detection


async def main():
    drone = Drone.Drone()
    aruco = ArucoFinder.ArucoFinder()
    ic = ImageConverter.ImageConverter()
    rospy.init_node("aruco_lander", anonymous=True)

    print("Waiting for drone to connect...")
    await drone.connect(SYS_ADDR)
    print(f"Drone connected")

    await asyncio.sleep(1) #wait for telemetry

    if drone.altitude > MAX_ALT:
        print("Current altitude is too high, not landing...")
        return

    await drone.startOffbaord()

    while True:
        vector = aruco.detect(ic.cv_image, ARUCO_ID, True)

        if vector != (-1, -1):
            forwardVelo = 0.0
            rightVelo = 0.0
            downVelo = 0.0

            if vector[0]**2 + vector[1]**2 < THREASHOLD**2:
                downVelo = DESC_VELO
            else:
                downVelo = 0.0

            forwardVelo = vector[1] * HOR_VELO
            rightVelo = vector[0] * HOR_VELO

            await drone.setOffboardVelocity(VelocityBodyYawspeed(forwardVelo, rightVelo, downVelo, 0))
        else:
            await drone.setOffboardVelocity(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        cv2.imshow("Frame", ic.cv_image)
        k = cv2.waitKey(3) & 0xFF
        if k == 27:
            break

        if drone.altitude <= LAND_ALT and drone.altitude >= 0:
            print("Performing final landig at current position")
            await drone.touchdown()
            break

    print("Stopping offboard")
    await drone.stopOffboard()

    print("Exiting...")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    asyncio.run(main())
