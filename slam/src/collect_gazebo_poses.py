#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from ml_msgs.msg import MarkerDetection
import pathlib,pickle


class CollectPoses:
    def __init__(self):
        self.datasize = rospy.get_param("~datasize") #size of measurements at current jackal position
        self.direction = rospy.get_param("~direction") #direction of measurement. Has to be set in terminal while launching. options: x-,x+,y-,y+
        self.marker_id = rospy.wait_for_message("/detected_markers_base_frame",MarkerDetection).markers[0].marker_id #gets the marker_id of the viewed marker
        self.true_marker_pose = Pose()
        self.jackal_poses = []
        self.sub_gazebo = rospy.Subscriber('/gazebo/link_states', LinkStates,self.callback_gazebo_poses, queue_size=100)#subscriber for all true positions in gazebo

    def callback_gazebo_poses(self,msg):
        for i in range(0,len(msg.name)):
            if msg.name[i] == "jackal0::base_link":
                self.jackal_poses.append(msg.pose[i]) #append current jackal0 pose
                break
        if len(self.jackal_poses) == self.datasize:
            print("collecting jackal poses finished") #if given datasize is reached
            self.sub_gazebo.unregister() #stop measuring
            if self.marker_id != 4: #if marker is not on the second jackal
                for j in range(0,len(msg.name)): 
                    if msg.name[j] == "AruCo_Block_" + str(self.marker_id) + "::chassis":
                        self.true_marker_pose = msg.pose[j] #save true marker pose given by gazebo
                        break
            else: #else marker on jackal
                for j in range(0,len(msg.name)): 
                    if msg.name[j] == "jackal1::base_link":
                        self.true_marker_pose = msg.pose[j] #else save the pose of the second jackal that has the viewed marker on top
                        break
            with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/true_poses.json","ab") as f:
                pickle.dump([self.true_marker_pose,self.direction,self.jackal_poses] ,f) #save true marker pose from gazebo, direction of measurement and all true jackal poses while measuring
            print("finished collecting true poses")

if __name__ == '__main__':
    rospy.init_node('collect_gazebo_poses')
    CollectPoses()
    rospy.spin()
  