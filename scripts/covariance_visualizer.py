#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from mrs_msgs.msg import PoseWithCovarianceArrayStamped

marker_out = Marker()
marker_out.color.a = 0.8
marker_out.color.r = 0.8
marker_out.color.g = 0.5
marker_out.color.b = 0.5

marker_in = Marker()
marker_in.color.a = 0.8
marker_in.color.r = 0.1
marker_in.color.g = 0.5
marker_in.color.b = 0.5

def pose_cb_out(data: PoseWithCovarianceStamped):
    marker_out.header = data.header
    marker_out.ns = "debug_out"
    marker_out.id = 0
    marker_out.pose = data.pose.pose
    marker_out.type = Marker.SPHERE
    marker_out.action = Marker.ADD
    marker_out.scale.x = data.pose.covariance[0]
    marker_out.scale.y = data.pose.covariance[7]
    marker_out.scale.z = data.pose.covariance[14]
    marker_publisher_out.publish(marker_out)

def pose_cb_in(data: PoseWithCovarianceArrayStamped):
    marker_in.header = data.header
    marker_in.ns = "debug_in"
    marker_in.id = 0
    marker_in.pose = data.poses[0].pose
    marker_in.type = Marker.SPHERE
    marker_in.action = Marker.ADD
    marker_in.scale.x = data.poses[0].covariance[0]
    marker_in.scale.y = data.poses[0].covariance[7]
    marker_in.scale.z = data.poses[0].covariance[14]
    marker_publisher_in.publish(marker_in)

if __name__ == '__main__':
    rospy.init_node('cov_debug')
    marker_publisher_out = rospy.Publisher('/cov_marker_output/debug', Marker, queue_size=1)
    marker_publisher_in = rospy.Publisher('/cov_marker_input/debug', Marker, queue_size=1)
    rospy.Subscriber('estimate_with_covariance/debug', PoseWithCovarianceStamped, callback=pose_cb_out)
    rospy.Subscriber('uav2/uvdar/filteredPoses', PoseWithCovarianceArrayStamped, callback=pose_cb_in)

    rospy.spin()