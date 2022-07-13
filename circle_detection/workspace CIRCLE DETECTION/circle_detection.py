#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse
from std_msgs.msg import Int16

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/usb_cam/image_raw',Image,self.callback)
    self.Raio = rospy.Publisher("raio",Int16,queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv.imshow("Image window", cv_image)
    #
    #parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
    #parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
    #args = parser.parse_args()
    #cap = cv.VideoCapture(cv_image)

    #ret, frame = cap.read()
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    lower_range = np.array([5, 100, 100])
    upper_range = np.array([18, 255, 255])

    mask = cv.inRange(hsv, lower_range, upper_range)
    cv.imshow("Image masked", mask)
    cv.imwrite("laranja2masked.jpg", mask)
    default_file = 'laranja2masked.jpg'
    filename = default_file
    # Loads an image
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    gray = cv.medianBlur(gray, 5)
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=60, param2=16,
                               minRadius=100, maxRadius=300)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            self.Raio.publish(radius)
            

            print(radius)
            cv.circle(src, center, radius, (255, 0, 255), 3)
    cv.imshow("detected circles", src)
    #
    cv.waitKey(3)
#
#parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
#parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
#args = parser.parse_args()
#
def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
  #try:
    rospy.spin()
  #except KeyboardInterrupt:
   # print("Shutting down")
    

    #cv.waitKey(0)
    #cv.imwrite("resultado.jpg", src)

    #return 0   

    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

