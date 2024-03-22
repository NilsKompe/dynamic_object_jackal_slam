#!/usr/bin/env python3

import rospy, rosbag
from geometry_msgs.msg import Twist
import pickle, pathlib
from gazebo_msgs.msg import LinkStates
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

class PublishPose:
    def __init__(self):
        self.test_values = []
        self.sub_gazebo_ = rospy.Subscriber('/gazebo/link_states', LinkStates,self.callback_jackal0_pose, queue_size=100)

    def callback_jackal0_pose(self,msg):
        for i in range(0,len(msg.name)):
            if msg.name[i] == "jackal1::base_link":
                print(msg.pose[i])
                # print(np.arctan(msg.pose[i].position.x/msg.pose[i].position.y))
                # print([msg.pose[i].orientation.x,msg.pose[i].orientation.y,msg.pose[i].orientation.z,msg.pose[i].orientation.w])
                # rot = Rotation.from_quat([msg.pose[i].orientation.x,msg.pose[i].orientation.y,msg.pose[i].orientation.z,msg.pose[i].orientation.w]) #convert quaternion into euler
                # rot_euler = rot.as_euler("xyz", degrees=True)
                # print(rot_euler)
                # euler_df = pd.DataFrame(data=rot_euler, index=['x', 'y', 'z'])
                # print("euler",euler_df)
        
    
if __name__ == '__main__':
    rospy.init_node('publish_gazebo_jackal_pose')
    class_publish_pose = PublishPose()
    rospy.spin()
  