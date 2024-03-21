#!/usr/bin/env python3

import rospy
from ml_msgs.msg import MarkerDetection
import pathlib,pickle


class CollectData:
    def __init__(self):
        self.datasize = rospy.get_param("~datasize") #size of measurements at current jackal position
        self.marker_poses = [] #list for all measured marker poses
        #subscriber for marker in jackal0 base_link frame
        self.sub_marker = rospy.Subscriber('/detected_markers_base_frame', MarkerDetection , self.callback_detected_marker, queue_size=100)

    def callback_detected_marker(self,msg):    
        self.marker_poses.append(msg.markers[0].pose) #appends detected marker pose to list of all poses
        if len(self.marker_poses) == self.datasize: #if wanted number of poses has been collected
            self.sub_marker.unregister() #stop measuring 
            with open(str(pathlib.Path(__file__).parent.resolve().parents[0]) + "/data/detected_markers.json","ab") as f:
                pickle.dump(self.marker_poses,f) #and safe measured poses
            print("finished collecting data")


if __name__ == '__main__':
    rospy.init_node('collect_sensor_data')
    CollectData()
    rospy.spin()
  