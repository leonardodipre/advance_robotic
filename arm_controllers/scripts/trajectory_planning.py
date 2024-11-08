#!/usr/bin/env python3
import rospy
from arm_controllers.msg import TrajectoryPlan, TrajectoryPhase

def publish_trajectory_plan():
    rospy.init_node('trajectory_plan_publisher')
    pub = rospy.Publisher('/trajectory_plan', TrajectoryPlan, queue_size=1)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        plan_msg = TrajectoryPlan()

        # Define phase 1
        phase1 = TrajectoryPhase()
        phase1.t_start = 0.0
        phase1.t_end = 2.0
        phase1.fixed_joints = [3]  # Joint indices to fix
        phase1.joint_angles = [1.57]  # Angles in radians (pi/2)
        phase1.use_dynamic_goal = False
        phase1.goal_position = []  # Empty since not used
        plan_msg.phases.append(phase1)

        # Define phase 2
        phase2 = TrajectoryPhase()
        phase2.t_start = 2.0
        phase2.t_end = 5.0
        phase2.fixed_joints = []  # No fixed joints
        phase2.joint_angles = []
        phase2.use_dynamic_goal = True  # Use dynamic goal
        phase2.goal_position = []  # Empty since dynamic
        plan_msg.phases.append(phase2)

        # Publish the trajectory plan
        pub.publish(plan_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory_plan()
    except rospy.ROSInterruptException:
        pass
