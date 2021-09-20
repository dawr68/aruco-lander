import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:

  def __init__(self):
    self.cv_image = np.zeros((384, 836), np.float32)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/standard_vtol/c920/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)