#!/usr/bin/env python
##########################
#### Author: Tri.B.M
#### Reference: Teleop_twist_keyboard package
### This code teleop keyboard using the Ackerman Mechanism

from __future__ import print_function
import rospy
from std_msgs.msg import Float32
#from pynput.keyboard import Key, Listener
import sys, select, termios, tty
import time

global speed
speed =0
global steering
steering =0

msg = """
------Keyboard teleop
"""
moveBindings = {
        'w':0,
        's':0,
        'a':0,
        'd':0
}


def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [],0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def teleop_car(speed, steering):
    if not rospy.is_shutdown():
        pub_speed = rospy.Publisher('Team1_speed', Float32, queue_size=10)
        pub_steering = rospy.Publisher('Team1_steerAngle', Float32, queue_size=10)
        rospy.init_node('teleop_car', anonymous=True)
        rate = rospy.Rate(20)

        speed = float(speed)
        steering = float(steering)
        rospy.loginfo(speed)
        rospy.loginfo(steering)
        pub_speed.publish(float(speed))
        pub_steering.publish(float(steering))

        rate.sleep()

if __name__ == "__main__":  
        settings = termios.tcgetattr(sys.stdin)     
        try:
                print(msg)
                while(1):
                        key=getKey()
                        if key in moveBindings.keys():
                                if key == 'w':
                                        speed = speed +5
                                if key == 's':
                                        speed = speed -5
                                if key == 'a':
                                        steering = steering -2
                                if key == 'd':
                                        steering = steering +2

                                
                        else:
                                speed =0
                                steering =0
                                if (key == '\x03'):
                                        break
                        teleop_car(speed, steering)

        except rospy.ROSInitException:
                pass
        
                                

