#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import pickle, pathlib, time

class Sub:
    def __init__(self):
        self.odom_values = []
        self.time_start = time.perf_counter()
        self.odom_from_joystick_sub = rospy.Subscriber('/jackal0/jackal_velocity_controller/odom', Odometry, self.callback_odom_data, queue_size = 100)
        
    def callback_odom_data(self,msg):
        self.odom_values.append(msg)
        if (self.time_start + 5 <= time.perf_counter()):
            print("finished_subscribing")
            self.odom_from_joystick_sub.unregister()
            with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/process_noise_odom.json","wb") as f:
                pickle.dump(self.odom_values,f)   
            
   
            
    
if __name__ == '__main__':
    rospy.init_node('sub_odom')
    Sub()
    rospy.spin()
  