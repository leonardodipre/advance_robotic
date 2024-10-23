#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from std_msgs.msg import Float32MultiArray
import message_filters

def log_header(msg_type, timestamp):
    rospy.loginfo("\n" + "=" * 50 + f" {msg_type} " + "=" * 50)
    rospy.loginfo(f"Timestamp: {timestamp.to_sec() if isinstance(timestamp, rospy.Time) else timestamp}")

def log_footer():
    rospy.loginfo("=" * 120 + "\n")

def synchronized_callback(pose_msg, velocity_msg, error_msg):
    
    # Log Pose
    log_header("----------------BEGIN --------------", pose_msg.header.stamp)

    log_header("Pose Update", pose_msg.header.stamp)
    rospy.loginfo("Position: x: {0:.3f}, y: {1:.3f}, z: {2:.3f}".format(
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z))
    rospy.loginfo("Orientation: x: {0:.3f}, y: {1:.3f}, z: {2:.3f}, w: {3:.3f}".format(
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w))
    
    log_footer()

    # Log Velocity
    log_header("Velocity Update", rospy.Time.now())
    rospy.loginfo("Linear Velocity: x: {0:.3f}, y: {1:.3f}, z: {2:.3f}".format(
        velocity_msg.linear.x, velocity_msg.linear.y, velocity_msg.linear.z))
    log_footer()

    # Log Pose Error
    log_header("Pose Error Update", rospy.Time.now())
    if len(error_msg.data) >= 3:
        rospy.loginfo("Position Error: x: {0:.3f}, y: {1:.3f}, z: {2:.3f}".format(
            error_msg.data[0], error_msg.data[1], error_msg.data[2]))
    else:
        rospy.logwarn("Pose Error message does not contain enough data.")
    log_footer()
    
    log_header("----------------END --------------", pose_msg.header.stamp)

def listener():
    rospy.init_node('aruco_listener', anonymous=True)
    
    # Create subscribers with message_filters
    pose_sub = message_filters.Subscriber('/pose', PoseStamped)
    velocity_sub = message_filters.Subscriber('/marker_velocity', Twist)
    error_sub = message_filters.Subscriber('/pose_error', Float32MultiArray)
    
    # Synchronize the topics
    ts = message_filters.ApproximateTimeSynchronizer(
        [pose_sub, velocity_sub, error_sub], 
        queue_size=10, 
        slop=0.1, 
        allow_headerless=True
    )
    ts.registerCallback(synchronized_callback)
    
    rospy.loginfo("Aruco Listener Node is running and synchronized to /pose, /marker_velocity, and /pose_error topics.")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
