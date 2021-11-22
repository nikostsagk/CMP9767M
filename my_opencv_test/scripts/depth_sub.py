#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
                                          Image, self.image_callback)
        self.green_mask_pub = rospy.Publisher("/green_mask", Image, queue_size=1)
        self.purple_mask_pub = rospy.Publisher("/purple_mask", Image, queue_size=1)
        self.depth_sub = rospy.Subscriber("/thorvald_001/kinect2_front_sensor/sd/image_depth_rect", Image, self.depth)

    def depth(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

        print(cv_image.shape)
        print(np.min(cv_image), np.max(cv_image))

    def image_callback(self, data):
        # imsgmsg to cv2
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Covnert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out anything but green
        green_mask = cv2.inRange(hsv_image, (35, 20, 20), (85, 255, 255))

        # Filter out anything but purple
        purple_mask = cv2.inRange(hsv_image, (100, 20, 20), (165, 255, 255))

        # cv2 to imgmsg
        green_mask_msg = self.bridge.cv2_to_imgmsg(green_mask, "mono8")
        green_mask_msg.header = data.header

        purple_mask_msg = self.bridge.cv2_to_imgmsg(purple_mask, "mono8")
        purple_mask_msg.header = data.header

        # publish masks
        self.green_mask_pub.publish(green_mask_msg)
        self.purple_mask_pub.publish(purple_mask_msg)

def main():
    ic = image_converter()

if __name__ == "__main__":
    try:
        rospy.init_node('image_converter')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

