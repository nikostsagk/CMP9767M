#!/usr/bin/env python

import numpy as np

import rospy
import tf
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class MyTFListener:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.laser_subs = [
            message_filters.Subscriber("/thorvald_001/front_scan", LaserScan),
            message_filters.Subscriber("/thorvald_001/back_scan", LaserScan)
        ]
        self.pub = rospy.Publisher("closest_laser_scan", PoseStamped, queue_size=1)

        self.ts = message_filters.ApproximateTimeSynchronizer(self.laser_subs, 1, 0.1)
        self.ts.registerCallback(self.laser_cb)

    def laser_cb(self, msg1, msg2):
        messages = [msg1, msg2]

        fmin = min(msg1.ranges)
        farg_min = np.argmin(msg1.ranges)

        bmin = min(msg2.ranges)
        barg_min = np.argmin(msg2.ranges)

        polar_r, index, m = min((fmin, farg_min, 0), (bmin, barg_min, 1), key=lambda x: x[0])
        polar_theta = messages[m].angle_min + (index * messages[m].angle_increment)

        # polar to cartesian
        x = polar_r * np.cos(polar_theta)
        y = polar_r * np.sin(polar_theta)
        
        msg = PoseStamped()
        msg.header = messages[m].header
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = np.sin(polar_theta/2)
        msg.pose.orientation.w = np.cos(polar_theta/2)

        p_in_base = self.listener.transformPose("thorvald_001/base_link", msg)
        print("Position of the object in the new frame of reference:")
        print(p_in_base)
        self.pub.publish(p_in_base)
    
        

def main():
    mtfl = MyTFListener()

if __name__ == "__main__":
    try:
        rospy.init_node("mytf_listener", anonymous=True)
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
