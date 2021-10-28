#!/usr/bin/env python

import random
import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class ThorvaldController:
    def __init__(self):
        self.velocity_pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.lidar_cb, queue_size=1)
        self.odometry_sub = rospy.Subscriber("/thorvald_001/odometry/base_raw", Odometry, self.odom_cb, queue_size=1)
        self.mileage_pub = rospy.Publisher("/mileage", String, queue_size=1)

        self.just_spawned = True
        self.mileage = 0

    def lidar_cb(self, data):
        # TODO: Merge the 2 lidars to a 360degree one, and use
        # the index of the beam that faces the front of the robot.
        min_dist = min(data.ranges)
        if min_dist > 3:
            self.go_forward()
        else:
            self.go_left()

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
    tc = ThorvaldController()

if __name__ == "__main__":
    try:
        rospy.init_node("thorvald_controller", anonymous=True)
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
