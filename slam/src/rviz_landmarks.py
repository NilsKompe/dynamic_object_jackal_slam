#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from ml_msgs.msg import MarkerDetection
from geometry_msgs.msg import Pose


def draw_landmark(msg):
    global counter0, counter1,counter2,counter3,counter4, reset

    for marker in msg.markers:
        print("msg_received")
        landmark.header.stamp = rospy.Time.now()
        landmark.pose = marker.pose
        landmark.ns = str(marker.marker_id)        

        if marker.marker_id == 0:
            landmark.id = counter0
            counter0 = counter0 + 1
            landmark.color.a = 1.0
            landmark.color.r = 1.0
            landmark.color.g = 0.0
            landmark.color.b = 0.0
        elif marker.marker_id == 1:
            landmark.id = counter1
            counter1 = counter1 + 1
            landmark.color.a = 1.0
            landmark.color.r = 0.0
            landmark.color.g = 1.0
            landmark.color.b = 0.0
        elif marker.marker_id == 2:
            landmark.id = counter2
            counter2 = counter2 + 1
            landmark.color.a = 1.0
            landmark.color.r = 0.0
            landmark.color.g = 0.0
            landmark.color.b = 1.0
        elif marker.marker_id == 3:
            landmark.id = counter3
            counter3 = counter3 + 1
            landmark.color.a = 1.0
            landmark.color.r = 0.0
            landmark.color.g = 0.0
            landmark.color.b = 0.0
        elif marker.marker_id == 4:
            landmark.id = counter4
            counter4 = counter4 + 1  
            landmark.color.a = 1.0
            landmark.color.r = 1.0
            landmark.color.g = 1.0
            landmark.color.b = 1.0  
        
        
        
    
        # landmark.color.a = 1.0
        # landmark.color.r = 0.0
        # landmark.color.g = 1.0
        # landmark.color.b = 0.0
        # rospy.sleep(1)
        if reset == 0:
            landmark.action = Marker.DELETEALL
            print(landmark.action)
            rospy.sleep(1)
            landmark_pub.publish(landmark)
            reset = 1
            landmark.action = Marker.ADD
        landmark_pub.publish(landmark)

    # while not rospy.is_shutdown():
        
    #     marker_pub.publish(marker)
    #     rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('rviz_landmarks')
    landmark = Marker()
    landmark.header.frame_id = "map"
    landmark.header.frame_id = "jackal0/base_link"
    landmark.type = Marker.ARROW
    landmark.header.stamp = rospy.Time.now()
    landmark.action = Marker.ADD
    landmark.scale.x = 0.1
    landmark.scale.y = 0.1
    landmark.scale.z = 0.1
    # landmark.scale.x = 1
    # landmark.scale.y = 1
    # landmark.scale.z = 1
    landmark.frame_locked = False
    #landmark.lifetime = 0
    reset = 0
    counter0 = 0
    counter1 = 0
    counter2 = 0
    counter3 = 0
    counter4 = 0
    landmark_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
    
    
    rospy.Subscriber('detected_markers_base_frame',MarkerDetection, draw_landmark,queue_size = 100)
    # rospy.Subscriber('ml_landmarks/detected_markers',MarkerDetection, draw_landmark,queue_size = 100)
    rospy.spin()

