#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def callback(msg):
    # We expect the message to contain positions and velocities for 6 joints.
    if len(msg.data) != 12:
        rospy.logwarn("Expected 12 data elements (6 positions, 6 velocities), but received %d.", len(msg.data))
        return
    
    # Extract positions and velocities
    positions = msg.data[:6]  # First 6 elements are positions
    velocities = msg.data[6:]  # Next 6 elements are velocities

    # Log the received positions and velocities
    rospy.loginfo("Received positions: %s", positions)
    rospy.loginfo("Received velocities: %s", velocities)

def listener():
    rospy.init_node('motion_command_listener', anonymous=True)
    rospy.Subscriber('/motion_command', Float64MultiArray, callback)
    rospy.loginfo("Subscriber node initialized and listening to /motion_command.")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
