#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String

def callback(message):
    rospy.loginfo("I heard %s", message.data)

rospy.init_node("listener")
sub = rospy.Subscriber("chatter", String, callback)
rospy.spin()
