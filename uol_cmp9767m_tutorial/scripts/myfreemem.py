#!/usr/bin/env python

import psutil

import rospy
from std_msgs.msg import String

def publisher():
    freemem_pub = rospy.Publisher("/freemem", String, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        memory_info = psutil.virtual_memory().available
        freemem_pub.publish(str(memory_info))

        rate.sleep() # slow it down a bit

if __name__ == "__main__":
    try:
        rospy.init_node("freemem_node", anonymous=True)
        publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
