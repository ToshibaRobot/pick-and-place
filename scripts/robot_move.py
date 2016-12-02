#!/usr/bin/env python

PACKAGE = 'pick_and_place'

import rospy
import geometry_msgs
import std_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Bool
object_position = 0.0
gripper_position= 0.0
suction = False

def talker():
    pub = rospy.Publisher('send_string', String, queue_size=10)

def object_handler(data):
    global object_position
    object_position = data.data

def gripper_handler(data):
    global marker_position
    marker_position =data.data

if __name__ == '__main__':
        talker()
        rospy.init_node('robot_move', anonymous=True)
        rospy.Subscriber("/object_detect", Point, object_handler)
        rospy.Subscriber("/marker", Point, marker_handler)
        pub = rospy.Publisher('send_string', String, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if (object_position == gripper_position):
                suction = True

#        pub.publish(hello_str)
        rate.sleep()
