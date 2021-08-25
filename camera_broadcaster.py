 #!/usr/bin/env python3


"""Publish a video as ROS messages from Raspicam in NOETIC.
"""

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

#import camera_info_manager

from cv_bridge import CvBridge, CvBridgeError

bridge1 = CvBridge()
bridge2 = CvBridge()


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=2,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():

   
    
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=2))
   
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            # cv2.imshow("CSI Camera", img)
            img_msg = bridge1.cv2_to_imgmsg(img, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "aperea_raspi_cam"
            dispatcher2.publish(img_msg)
            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")
        
        
def main():
    """Publish a video as ROS messages.
    """
    global dispatcher, dispatcher2
    dispatcher2 = rospy.Publisher('aperea_cam/raw', Image, queue_size =10)
    #parser = argparse.ArgumentParser(description="Convert video into a rosbag.")
    #parser.add_argument("video_file", help="Input video.")
    #parser.add_argument("-c", "--camera", default="camera", help="Camera name.")
    #parser.add_argument("-f", "--frame_id", default="camera",
    #                    help="tf frame_id.")
    #parser.add_argument("--width", type=np.int32, default="1280",
    #                    help="Image width.")
    #parser.add_argument("--height", type=np.int32, default="720",
    #                    help="Image height.")
    #parser.add_argument("--info_url", default="file:/video_stream_opencv/config/camera_info.yaml",
    #                    help="Camera calibration url.")
#
    #args = parser.parse_args()

    #print "Publishing %s." % (args.video_file)
    # Set up node.
    rospy.init_node("camera_publisher", anonymous=True)
    #img_pub = rospy.Publisher("/aperea_cam/image_raw", Image,
    #                          queue_size=10)
    #info_pub = rospy.Publisher("/aperea_cam/camera_info", CameraInfo,
    #                           queue_size=10)

    #info_manager = camera_info_manager.CameraInfoManager(cname=args.camera,
    #                                                     url=args.info_url,
    #                                                     namespace=args.camera)
    #info_manager.loadCameraInfo()
    show_camera()
    rospy.spin()
    
    
if __name__ == '__main__':
    main()