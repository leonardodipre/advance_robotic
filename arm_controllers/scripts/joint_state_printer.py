#!/usr/bin/env python

import rospy
from arm_controllers.msg import ControllerJointState  # Replace with the correct message type

def callback(msg):
    rospy.loginfo("\n\n----- Joint State Info -----")
    for i in range(len(msg.name)):
        rospy.loginfo(f"Joint Name: {msg.name[i]}")
        rospy.loginfo(f"  Command: {msg.command[i]:.4f} radians")
        rospy.loginfo(f"  Command Dot: {msg.command_dot[i]:.4f} rad/s")
        rospy.loginfo(f"  State: {msg.state[i]:.4f} radians")
        rospy.loginfo(f"  State Dot: {msg.state_dot[i]:.4f} rad/s")
        rospy.loginfo(f"  Error: {msg.error[i]:.4f} radians")
        rospy.loginfo(f"  Error Dot: {msg.error_dot[i]:.4f} rad/s")
        rospy.loginfo(f"  Effort Command: {msg.effort_command[i]:.4f} N·m")
        rospy.loginfo(f"  Effort Feedforward: {msg.effort_feedforward[i]:.4f} N·m")
        rospy.loginfo(f"  Effort Feedback: {msg.effort_feedback[i]:.4f} N·m")
    rospy.loginfo("--------------------------------\n")

def listener():
    rospy.init_node('joint_state_printer', anonymous=True)
    rospy.Subscriber("/elfin/gravity_comp_controller/state", ControllerJointState, callback)  # Replace with your topic name
    rospy.spin()

if __name__ == '__main__':
    listener()
