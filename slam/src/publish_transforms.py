#!/usr/bin/env python3

import rospy, rosbag
from geometry_msgs.msg import Twist
import pickle, pathlib
from gazebo_msgs.msg import LinkStates
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from tf.msg import tfMessage

class PublishTransform:
    def __init__(self):
        self.test_values = []
        self.sub_gazebo_ = rospy.Subscriber('/tf',tfMessage,self.callback_publish_transform, queue_size=100)

    def callback_publish_transform(self,msg):
        for transform in msg.transforms:
            if  transform.header.frame_id == "jackal0/odom" and transform.child_frame_id == "jackal0/base_link":
            # if  transform.header.frame_id == "map" and transform.child_frame_id == "jackal0/odom":
                print(transform)
    
if __name__ == '__main__':
    rospy.init_node('publish_transform')
    class_publish_transform = PublishTransform()
    rospy.spin()
  