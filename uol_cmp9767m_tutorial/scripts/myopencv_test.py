#!/usr/bin/env python

import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
                                          Image, self.image_callback)
        self.image_pub = rospy.Publisher("/green_mask", Image, queue_size=1)

    def image_callback(self, data):
        # imsgmsg to cv2
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Covnert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out green
        mask = cv2.inRange(hsv_image, (35, 20, 20), (85, 255, 255))

        # cv2 to imgmsg
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        mask_msg.header = data.header

        # publish mask
        self.image_pub.publish(mask_msg)

def main():
    ic = image_converter()

if __name__ == "__main__":
    try:
        rospy.init_node('image_converter')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

