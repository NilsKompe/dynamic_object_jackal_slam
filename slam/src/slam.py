#!/usr/bin/env python3
import rospy
import tf2_ros
import tf
from ml_msgs.msg import MarkerDetection, Marker
from geometry_msgs.msg import PointStamped
# import mrekf_slam.src.mrekf.ekf_base.py

class Slam:
    def __init__(self):
        self.pub_transformed_marker = rospy.Publisher("detected_markers_transformed",MarkerDetection,queue_size=10)
        self.sub = rospy.Subscriber('ml_landmarks/detected_markers',MarkerDetection, self.callback_marker_detected)
        self.tf_listener = tf.TransformListener(rospy.Duration(10))
        self.known_marker_ids = []

    def callback_marker_detected(self,msg):
        marker_detection_transformed = MarkerDetection()
        marker_detection_transformed.header = msg.header
        for marker in msg.markers:
            if self.known_marker_ids.__contains__(marker.marker_id):
                # break
                marker_detection_transformed.markers.append(self.transform_point(marker,"jackal0/base_link"))
                marker_detection_transformed.header.frame_id = "jackal0/base_link"
                #update step
            else:
                print("new")
                # self.known_marker_ids.append(marker.marker_id)
                marker_detection_transformed.markers.append(self.transform_point(marker,"map"))
                marker_detection_transformed.header.frame_id = "map" 
                #extend step
            self.pub_transformed_marker.publish(marker_detection_transformed)

    def transform_point(self,marker,frame) -> Marker:
        try:
            marker_point_stamped = PointStamped()
            marker_point_stamped.header.frame_id = "jackal0/front_camera_optical"
            marker_point_stamped.point = marker.pose.position
            # print(frame)
            self.tf_listener.waitForTransform(frame, "jackal0/front_camera_optical", rospy.Time(0),rospy.Duration(4,0))
            marker.pose.position = self.tf_listener.transformPoint(frame, marker_point_stamped).point
            return marker

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print("Received an exception trying to transform a point from \"jackal0/front_camera_optical\" to ",frame)
            rospy.logerr(ex)
            



if __name__ == '__main__':
    rospy.init_node('slam')
    Slam()
    rospy.spin()