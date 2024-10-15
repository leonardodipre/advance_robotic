#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped, PointStamped

def pose_callback(msg):
    print("Pose of Marker:")
    print("Position: x={:.3f}, y={:.3f}, z={:.3f}".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    print("Orientation: x={:.3f}, y={:.3f}, z={:.3f}, w={:.3f}".format(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

def transform_callback(msg):
    print("Transform of Marker:")
    print("Translation: x={:.3f}, y={:.3f}, z={:.3f}".format(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))
    print("Rotation: x={:.3f}, y={:.3f}, z={:.3f}, w={:.3f}".format(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w))

def position_callback(msg):
    print("Position Vector of Marker:")
    print("Vector: x={:.3f}, y={:.3f}, z={:.3f}".format(msg.vector.x, msg.vector.y, msg.vector.z))

def pixel_callback(msg):
    print("Pixel Coordinates of Marker Center:")
    print("Pixel: x={:.0f}, y={:.0f}".format(msg.point.x, msg.point.y))

def main():
    rospy.init_node('aruco_marker_info_listener', anonymous=True)

    # Update the topics here to match the correct names
    rospy.Subscriber("/aruco_single/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/aruco_single/transform", TransformStamped, transform_callback)
    rospy.Subscriber("/aruco_single/position", Vector3Stamped, position_callback)
    rospy.Subscriber("/aruco_single/pixel", PointStamped, pixel_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
