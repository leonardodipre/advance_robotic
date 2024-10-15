#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped, PointStamped
from visualization_msgs.msg import Marker

def pose_callback(msg):
    rospy.loginfo("----- PoseStamped Message -----")
    rospy.loginfo("Frame ID: %s", msg.header.frame_id)
    rospy.loginfo("Timestamp: %s", msg.header.stamp)
    rospy.loginfo("Position -> x: %.3f, y: %.3f, z: %.3f", 
                  msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    rospy.loginfo("Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f", 
                  msg.pose.orientation.x, msg.pose.orientation.y, 
                  msg.pose.orientation.z, msg.pose.orientation.w)
    rospy.loginfo("-------------------------------\n")

def transform_callback(msg):
    rospy.loginfo("----- TransformStamped Message -----")
    rospy.loginfo("Parent Frame: %s", msg.header.frame_id)
    rospy.loginfo("Child Frame: %s", msg.child_frame_id)
    rospy.loginfo("Timestamp: %s", msg.header.stamp)
    rospy.loginfo("Translation -> x: %.3f, y: %.3f, z: %.3f", 
                  msg.transform.translation.x, msg.transform.translation.y, 
                  msg.transform.translation.z)
    rospy.loginfo("Rotation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f", 
                  msg.transform.rotation.x, msg.transform.rotation.y, 
                  msg.transform.rotation.z, msg.transform.rotation.w)
    rospy.loginfo("-------------------------------------\n")

def position_callback(msg):
    rospy.loginfo("----- Vector3Stamped Message -----")
    rospy.loginfo("Frame ID: %s", msg.header.frame_id)
    rospy.loginfo("Timestamp: %s", msg.header.stamp)
    rospy.loginfo("Vector -> x: %.3f, y: %.3f, z: %.3f", 
                  msg.vector.x, msg.vector.y, msg.vector.z)
    rospy.loginfo("------------------------------------\n")

def marker_callback(msg):
    rospy.loginfo("----- Marker Message -----")
    rospy.loginfo("Marker ID: %d", msg.id)
    rospy.loginfo("Namespace: %s", msg.ns)
    rospy.loginfo("Type: %d", msg.type)
    rospy.loginfo("Action: %d", msg.action)
    rospy.loginfo("Position -> x: %.3f, y: %.3f, z: %.3f", 
                  msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    rospy.loginfo("Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f", 
                  msg.pose.orientation.x, msg.pose.orientation.y, 
                  msg.pose.orientation.z, msg.pose.orientation.w)
    rospy.loginfo("Scale -> x: %.3f, y: %.3f, z: %.3f", 
                  msg.scale.x, msg.scale.y, msg.scale.z)
    rospy.loginfo("Color -> r: %.3f, g: %.3f, b: %.3f, a: %.3f", 
                  msg.color.r, msg.color.g, msg.color.b, msg.color.a)
    rospy.loginfo("--------------------------------\n")

def pixel_callback(msg):
    rospy.loginfo("----- PointStamped (Pixel) Message -----")
    rospy.loginfo("Frame ID: %s", msg.header.frame_id)
    rospy.loginfo("Timestamp: %s", msg.header.stamp)
    rospy.loginfo("Pixel Coordinates -> x: %.1f, y: %.1f, z: %.1f", 
                  msg.point.x, msg.point.y, msg.point.z)
    rospy.loginfo("----------------------------------------\n")

def listener():
    rospy.init_node('aruco_listener', anonymous=True)
    rospy.loginfo("----- Aruco Listener Node Initialized -----\n")

    # Subscribers for all published topics
    rospy.Subscriber('pose', PoseStamped, pose_callback)
    rospy.Subscriber('transform', TransformStamped, transform_callback)
    rospy.Subscriber('position', Vector3Stamped, position_callback)
    rospy.Subscriber('marker', Marker, marker_callback)
    rospy.Subscriber('pixel', PointStamped, pixel_callback)

    rospy.loginfo("Subscribed to 'pose', 'transform', 'position', 'marker', and 'pixel' topics.\n")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
