#!/usr/bin/env python

import sys
from pynput import keyboard

import rospy
from geometry_msgs.msg import Twist

def on_shutdown():
    rospy.loginfo("Shutting down...")

class ThorvaldController:
    def __init__(self):
        self.velocity_pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
    
    def go_forward(self):
        msg = Twist()
        msg.linear.x = 2
        self.velocity_pub.publish(msg)
    
    def go_left(self):
        msg = Twist()
        msg.angular.z = 0.1
        self.velocity_pub.publish(msg)
    
    def go_right(self):
        msg = Twist()
        msg.angular.z = -0.1
        self.velocity_pub.publish(msg)
    
    def stop(self, key):
        msg = Twist()
        self.velocity_pub.publish(msg)
    
    def move(self, key):
        try:
            if key == keyboard.KeyCode(char="w"):
                self.go_forward()
            elif key == keyboard.KeyCode(char="a"):
                self.go_left()
            elif key == keyboard.KeyCode(char="d"):
                self.go_right()
            elif key == keyboard.KeyCode(char="q"): # Doesn't work
                rospy.signal_shutdown("Shutting down")
                sys.exit()
        except AttributeError:
            pass

def main():
    tc = ThorvaldController()
    with keyboard.Listener(
        on_press=tc.move,
        on_release=tc.stop) as listener:
        listener.join()

if __name__ == "__main__":
    try:
        rospy.init_node("thorvald_controller", anonymous=True, disable_signals=True)
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
