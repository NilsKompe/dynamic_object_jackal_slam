#!/usr/bin/env python3

import rospy, rosbag
from geometry_msgs.msg import Twist
import pathlib

class PublishTwist:
    def __init__(self):
        self.test_values = [] 
        self.pub_dummytwist = rospy.Publisher('dummytwist', Twist,queue_size=100)
        self.rostime_now = 0
        
    def publish_test_values(self): #publishes all test values of the rosbag file. Each for 2.5 seconds
        while (True):
            for i in range(0,len(self.test_values)):
                self.rostime_now = rospy.get_time()
                while(float(rospy.get_time()) - 0.5  < float(self.rostime_now)): 
                    self.pub_dummytwist.publish(self.test_values[0])# publish zeroes for 0.5 seconds to indicate new data sample
                self.rostime_now = rospy.get_time()
                while(rospy.get_time() - 2.5 < self.rostime_now):
                    self.pub_dummytwist.publish(self.test_values[i]) #publish test value for 2.5 seconds
                    rospy.sleep(10e-3)

    def read_rosbag(self): #saves all testvalues in one list
        bag = rosbag.Bag(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/rosbag/twist_testdata.bag")      
        for topic, msg, t in bag.read_messages(topics=['/jackal0/jackal_velocity_controller/cmd_vel']):
            self.test_values.append(msg)
        bag.close()
        
    
if __name__ == '__main__':
    rospy.init_node('publish_cmd_vel')
    class_twist = PublishTwist()

    class_twist.read_rosbag()
    class_twist.publish_test_values()
    rospy.spin()
  