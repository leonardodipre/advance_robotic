#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math
import threading

# Global variables
current_motion = 4  # Default to fixed motion at startup
custom_angles = [0] * 6  # Default custom angles for all 6 joints
current_positions = [0] * 6  # Store current joint positions
target_positions = [0] * 6  # Target positions for linear motion

def print_menu():
    rospy.loginfo("\nMenu:")
    rospy.loginfo("1 - Sinusoidal Motion")
    rospy.loginfo("2 - Circular Motion")
    rospy.loginfo("3 - Linear Motion")
    rospy.loginfo("4 - Fixed Motion (default)")
    rospy.loginfo("5 - Custom Angle Motion")
    rospy.loginfo("6 - Exit")
   


def get_motion_choice():
    choice = input("Enter your choice (1-6): ")
    try:
        choice = int(choice)
        if choice in [1, 2, 3, 4, 5, 6]:
            return choice
        else:
            rospy.loginfo("Invalid choice. Enter a number between 1 and 6.")
            return None
    except ValueError:
        rospy.loginfo("Invalid input. Enter a number.")
        return None

def sinusoidal_motion(t):
    position = [45 * math.radians(math.sin(math.pi / 2 * t))] * 6
    velocity = [45 * math.radians(math.pi / 2 * math.cos(math.pi / 2 * t))] * 6
    return position, velocity

def circular_motion(t):
    position = [math.sin(t)] * 6  # Example circular motion
    velocity = [math.cos(t)] * 6
    return position, velocity

def linear_transition(current_position, target_position, t):
    if not hasattr(linear_transition, 'prev_time'):
        linear_transition.prev_time = t
        linear_transition.tau = 0

    v_tau = 0.1  # Controls how fast the robot moves from start to target positions
    dt = t - linear_transition.prev_time
    linear_transition.prev_time = t
    linear_transition.tau += dt * v_tau

    if linear_transition.tau >= 1:
        linear_transition.tau = 1  # Cap tau at 1 to ensure it finishes correctly

    q_d = [(1 - linear_transition.tau) * current_position[i] + linear_transition.tau * target_position[i] for i in range(6)]
    q_dot_d = [v_tau * (target_position[i] - current_position[i]) for i in range(6)]
    return q_d, q_dot_d

def straight_line_motion(t):
    global current_positions, target_positions
    return linear_transition(current_positions, target_positions, t)

def fixed_motion():
    position = [0.1, -1, -0.5, 0.0, 1, -0.1]
    velocity = [0] * 6
    return position, velocity

def custom_angle_motion():
    global custom_angles
    position = custom_angles
    velocity = [0] * 6  # No velocity for custom angles
    return position, velocity

def get_custom_angles():
    global custom_angles
    rospy.loginfo("Enter custom angles for each joint (6 values)")
    for i in range(6):
        while True:
            try:
                angle = float(input(f"Enter angle for joint {i+1}: "))
                custom_angles[i] = angle
                break
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a valid number.")

def get_target_positions():
    global target_positions
    rospy.loginfo("Enter target positions for linear motion (6 values) in degree")
    for i in range(6):
        while True:
            try:
                position = float(input(f"Enter target position for joint {i+1}: "))
                target_positions[i] = math.radians(position)  # Convert to radians
                break
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a valid number.")

def publish_motion(pub, motion_type, t):
    global current_positions
    msg = Float64MultiArray()
    positions, velocities = [], []
    
    if motion_type == 1:  # Sinusoidal
        positions, velocities = sinusoidal_motion(t)
    elif motion_type == 2:  # Circular
        positions, velocities = circular_motion(t)
    elif motion_type == 3:  # Linear
        positions, velocities = straight_line_motion(t)
    elif motion_type == 4:  # Fixed position
        positions, velocities = fixed_motion()
    elif motion_type == 5:  # Custom angle position
        positions, velocities = custom_angle_motion()
    
    # Update current positions directly after calculation
    current_positions = positions

    msg.data = positions + velocities
    pub.publish(msg)

def publisher_thread(pub):
    global current_motion
    t = 0
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        publish_motion(pub, current_motion, t)
        t += 0.1  # Increment time
        rate.sleep()

def menu_thread():
    global current_motion
    while not rospy.is_shutdown():
        print_menu()
        choice = get_motion_choice()

        if choice is not None:
            if choice == 6:
                rospy.loginfo("Exiting...")
                rospy.signal_shutdown("User requested exit.")
                break
            elif choice == 5:
                get_custom_angles()
                current_motion = choice
                rospy.loginfo("Custom angles set and motion changed to Custom Angle Motion")
            elif choice == 3:
                get_target_positions()
                current_motion = choice
                rospy.loginfo("Target positions set and motion changed to Linear Motion")
            else:
                current_motion = choice
                rospy.loginfo(f"Motion changed to {current_motion}")

def main():
    global current_motion
    rospy.init_node('motion_menu_publisher', anonymous=True)
    pub = rospy.Publisher('/motion_command', Float64MultiArray, queue_size=10)

    # Start the publisher thread that continuously publishes the motion
    pub_thread = threading.Thread(target=publisher_thread, args=(pub,))
    pub_thread.start()

    # Start the menu thread to allow users to change motion types
    menu_thread()

if __name__ == '__main__':
    main()