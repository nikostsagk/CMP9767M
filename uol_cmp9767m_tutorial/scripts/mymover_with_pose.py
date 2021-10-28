#!/usr/bin/env python

import random
import numpy as np

import rospy
import tf
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ThorvaldControllerWithPose:
    def __init__(self):
        self.listener = tf.TransformListener()
        
        # Subscribers
        self.odometry_sub = rospy.Subscriber("/thorvald_001/odometry/base_raw", Odometry, self.odom_cb, queue_size=1)
        self.laser_subs = [
            message_filters.Subscriber("/thorvald_001/front_scan", LaserScan),
            message_filters.Subscriber("/thorvald_001/back_scan", LaserScan)
        ]

        # Publishers
        self.laser_dist_pub = rospy.Publisher("closest_laser_scan", PoseStamped, queue_size=1)
        self.velocity_pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
        self.mileage_pub = rospy.Publisher("/mileage", String, queue_size=1)

        self.ts = message_filters.ApproximateTimeSynchronizer(self.laser_subs, 1, 0.1)
        self.ts.registerCallback(self.laser_cb)

        self.just_spawned = True
        self.mileage = 0

    def laser_cb(self, msg1, msg2):
        messages = [msg1, msg2]

        fmin = min(msg1.ranges)
        farg_min = np.argmin(msg1.ranges)

        bmin = min(msg2.ranges)
        barg_min = np.argmin(msg2.ranges)

        polar_r, index, m = min((fmin, farg_min, 0), (bmin, barg_min, 1), key=lambda x: x[0])
        polar_theta = messages[m].angle_min + (index * messages[m].angle_increment)

        # Control Thorvald
        if (polar_r < 3) and (m == 0):
            # Because we check the min. distance from both lasers, if Thorvald gets too close
            # he is gonna indefinitely stuck in this condition. We should make sure that rotation
            # depends only with the minimum distance from the front laser
            self.go_left()
        else:
            self.go_forward()

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
        self.laser_dist_pub.publish(p_in_base)

    def odom_cb(self, data):
        if self.just_spawned == True:
            self.old_x = data.pose.pose.position.x
            self.old_y = data.pose.pose.position.y
            self.just_spawned = False

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        mileage = np.sqrt((x - self.old_x)**2 + (y - self.old_y)**2)
        self.mileage += mileage

        self.mileage_pub.publish("Miles driven: {:.3f}".format(self.mileage/1609))
        self.old_x = data.pose.pose.position.x
        self.old_y = data.pose.pose.position.y

    def go_forward(self):
        msg = Twist()
        msg.linear.x = 0.8
        self.velocity_pub.publish(msg)

    def go_back(self):
        msg = Twist()
        msg.linear.x = -0.8
        self.velocity_pub.publish(msg)

    def go_left(self):
        msg = Twist()
        msg.angular.z = random.uniform(0, 3.14)
        self.velocity_pub.publish(msg)

    def stop(self, key):
        msg = Twist()
        self.velocity_pub.publish(msg)   

def main():
    tcwp = ThorvaldControllerWithPose()

if __name__ == "__main__":
    try:
        rospy.init_node("mymover_with_pose", anonymous=True)
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
