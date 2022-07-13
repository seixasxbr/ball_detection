#!/usr/bin/env python3

import cv2
#import cv2.aruco as aruco
import math
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from geometry_msgs.msg import FloatingPointError
from cv_bridge import CvBridge, CvBridgeError
import numpy as np




print("oi!/n")
class ArucoIdentifier:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub =rospy.Subscriber('/aperea_cam/raw',Image,self.callback)
    self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    self.parameters =  cv2.aruco.DetectorParameters_create()
    #self.distCoeffs = np.array([0, 0, 0, 0, 0])
    self.distCoeffs = np.array([0.1465167016954302, -0.2847343180128725, 0.00134017721235817, -0.004309553450829512, 0.0])
    #self.cameraMat = np.array([[762.7249337622711, 0.0, 640.5]
    #                    , [0.0, 762.7249337622711, 380.5]
    #                    , [0.0, 0.0, 1.0]])
    self.cameraMat = np.array([[1276.704618338571, 0 , 634.8876509199106],[ 0, 1274.342831275509,379.8318028940378], [ 0, 0, 1]])
    rospy.init_node('image_converter', anonymous = True)
    print("oi2!")
 

  def callback(self,data):
    print("teste1")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
      print(e)
      
   

    #(rows,cols,channels) = cv_image.shape
    #(corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary,
	  #parameters=arucoParams)
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    #if len(corners) > 0:
    #  # flatten the ArUco IDs list
    #	ids = ids.flatten()
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.parameters)
    cv_image = cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds, borderColor=(0, 0, 255))
    print(markerIds)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.1, self.cameraMat, self.distCoeffs )
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    print(tvecs)
def main(args):
  ic = ArucoIdentifier()
  #rospy.int_node('image_converter', anonymous=True)
  print("oi3")
  try:
    rospy.spin()
    print("oi4")
    
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
