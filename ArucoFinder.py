import cv2
from cv2 import aruco

class ArucoFinder:
    def __init__(self):
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.params = aruco.DetectorParameters_create()

    def detect(self, frame, arucoID, draw = False):
        (markerCorners, markerIds, rejected) = cv2.aruco.detectMarkers(frame, self.dict, parameters=self.params)
        if len(markerCorners) > 0:
            if draw:
                cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            
            markerIds = markerIds.flatten()
            for (markerCorner, markerID) in zip(markerCorners, markerIds):
                if arucoID == markerID:
                    return self._calcVector(markerCorner, frame)

        return (-1, -1) #specified aruco not found


    def _calcCenter(self, corners):
        (topLeft, topRight, bottomRight, bottomLeft) = corners[0]
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        return (
            int((topLeft[0] + bottomRight[0]) / 2.0),
            int((topLeft[1] + bottomRight[1]) / 2.0))


    def _calcVector(self, corners, frame):
        fHeight, fWidth = frame.shape[:2]
        center = self._calcCenter(corners)
        return (
            (center[0] - fWidth/2) / (fWidth/2),
            (fHeight/2 - center[1]) / (fWidth/2))