#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped


class AddHeader:
    def __init__(self):
        self.twist_stamped = TwistStamped()
        self.pub_twist = rospy.Publisher('/jackal0/jackal_velocity_controller/cmd_vel', Twist,queue_size=100)
        self.pub_twist_stamped = rospy.Publisher('/jackal0/jackal_velocity_controller/cmd_vel_stamped', TwistStamped,queue_size=100)
        self.sub_dummytwist = rospy.Subscriber('dummytwist', Twist , self.callback_add_header, queue_size=100)    

    def callback_add_header(self,msg):
        self.twist_stamped.twist = msg
        self.twist_stamped.header.stamp = rospy.Time.now()
        self.pub_twist.publish(msg)
        self.pub_twist_stamped.publish(self.twist_stamped)

if __name__ == '__main__':
    rospy.init_node('add_header')
    AddHeader()
    rospy.spin()
  