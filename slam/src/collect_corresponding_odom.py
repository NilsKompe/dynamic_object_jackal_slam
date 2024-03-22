#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import pathlib,pickle



class CollectCorrespondingOdom:
    def __init__(self):
        self.number_of_each_sample = 4
        self.test_datasize = 700
        self.current_size_odom = 1
        self.current_size_twist = 1
        self.twist_testdata_linear = []
        self.twist_testdata_angular = []
        self.odom_twist_linear = []
        self.odom_twist_angular = []

        self.twist_stamped = TwistStamped()
        self.timestamp_new_twist = rospy.Time.now()
        self.timestamp_new_twist.secs = 9999
        self.sub_twist_stamped = rospy.Subscriber('/jackal0/jackal_velocity_controller/cmd_vel_stamped', TwistStamped , self.callback_twist_stamped, queue_size=100)    
        self.sub_odom = rospy.Subscriber('/jackal0/jackal_velocity_controller/odom', Odometry , self.callback_odom, queue_size=100)    

    def callback_twist_stamped(self,msg):
        rospy.loginfo(len(self.twist_testdata_linear))
        if len(self.odom_twist_angular)  == self.test_datasize and len(self.twist_testdata_angular) == self.test_datasize:
            rospy.loginfo("finished data collecting. Start saving")
            self.sub_twist_stamped.unregister()
            self.sub_odom.unregister()
            self.save_results()
        elif msg.twist.linear.x == 0 and msg.twist.angular.z == 0:
            self.timestamp_new_twist = msg.header.stamp
            self.timestamp_new_twist.secs = self.timestamp_new_twist.secs + 1
            self.current_size_odom = 1
            self.current_size_twist = 1
        elif msg.header.stamp.secs > self.timestamp_new_twist.secs and self.current_size_twist <= self.number_of_each_sample:
            self.current_size_twist = self.current_size_twist + 1

            self.twist_testdata_linear.append(msg.twist.linear.x)
            self.twist_testdata_angular.append(msg.twist.angular.z)
            print("twist_testdata_linear",len(self.twist_testdata_linear))
            print("twist_testdata_angular",len(self.twist_testdata_angular))
            print("twist_testdata_linear",self.twist_testdata_linear)
            print("twist_testdata_angular",self.twist_testdata_angular)

    def callback_odom(self,msg):
        if msg.header.stamp.secs > self.timestamp_new_twist.secs and self.current_size_odom <= self.number_of_each_sample:
            self.current_size_odom = self.current_size_odom + 1
            self.odom_twist_linear.append(msg.twist.twist.linear.x)
            self.odom_twist_angular.append(msg.twist.twist.angular.z)
            print("odom_twist_linear",self.odom_twist_linear)
            print("odom_twist_angular",self.odom_twist_angular)
            print("odom_twist_linear",len(self.odom_twist_linear))
            print("odom_twist_angular",len(self.odom_twist_angular))
    
    def save_results(self):
        results = ["twist_testdata_linear",self.twist_testdata_linear,"twist_testdata_angular",self.twist_testdata_angular,
                   "odom_twist_linear",self.odom_twist_linear,"odom_twist_angular", self.odom_twist_angular]
        with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/twist_and_odom.json","wb") as f:
                pickle.dump(results,f)  
        print("finished saving")

if __name__ == '__main__':
    rospy.init_node('collect_corresponding_odom')
    CollectCorrespondingOdom()
    rospy.spin()
  