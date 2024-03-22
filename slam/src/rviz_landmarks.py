#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from ml_msgs.msg import MarkerDetection

class DrawLandmark:
    def __init__(self):
        self.landmark = Marker()
        self.landmark.header.frame_id = "jackal0/base_link"
        self.landmark.type = Marker.ARROW
        self.landmark.header.stamp = rospy.Time.now()
        self.landmark.action = Marker.ADD
        self.landmark.scale.x = 0.1
        self.landmark.scale.y = 0.1
        self.landmark.scale.z = 0.1 
        self.landmark.frame_locked = False
        #landmark.lifetime = 0
        self.counter0 = 0
        self.counter1 = 0
        self.counter2 = 0
        self.counter3 = 0
        self.counter4 = 0
        self.reset = 0
        self.landmark_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self.sub_detected_marker = rospy.Subscriber('detected_markers_base_frame',MarkerDetection, self.callback_draw_landmark,queue_size = 100)
    
    def callback_draw_landmark(self,msg):
        for marker in msg.markers:
            print("msg_received")
            self.landmark.header.stamp = rospy.Time.now()
            self.landmark.pose = marker.pose
            self.landmark.ns = str(marker.marker_id)        

            if marker.marker_id == 0:
                self.landmark.id = self.counter0
                self.counter0 = self.counter0 + 1
                self.landmark.color.a = 1.0
                self.landmark.color.r = 1.0
                self.landmark.color.g = 0.0
                self.landmark.color.b = 0.0
            elif marker.marker_id == 1:
                self.landmark.id = self.counter1
                self.counter1 = self.counter1 + 1
                self.landmark.color.a = 1.0
                self.landmark.color.r = 0.0
                self.landmark.color.g = 1.0
                self.landmark.color.b = 0.0
            elif marker.marker_id == 2:
                self.landmark.id = self.counter2
                self.counter2 = self.counter2 + 1
                self.landmark.color.a = 1.0
                self.landmark.color.r = 0.0
                self.landmark.color.g = 0.0
                self.landmark.color.b = 1.0
            elif marker.marker_id == 3:
                self.landmark.id = self.counter3
                self.counter3 = self.counter3 + 1
                self.landmark.color.a = 1.0
                self.landmark.color.r = 0.0
                self.landmark.color.g = 0.0
                self.landmark.color.b = 0.0
            elif marker.marker_id == 4:
                self.landmark.id = self.counter4
                self.counter4 = self.counter4 + 1  
                self.landmark.color.a = 1.0
                self.landmark.color.r = 1.0
                self.landmark.color.g = 1.0
                self.landmark.color.b = 1.0  
            self.landmark_pub.publish(self.landmark)
            
if __name__ == '__main__':
    rospy.init_node('rviz_landmarks')
    DrawLandmark()
    landmark = Marker()
    rospy.spin()
  
