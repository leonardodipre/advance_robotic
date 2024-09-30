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

# For acceleration calculations in linear motion
previous_positions = [0] * 6
previous_time = None
desired_accelerations = [0] * 6

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
    position = [math.radians(45 * math.sin(math.pi / 2 * t))] * 6
    velocity = [math.radians(45 * (math.pi / 2) * math.cos(math.pi / 2 * t))] * 6
    acceleration = [math.radians(-45 * (math.pi / 2)**2 * math.sin(math.pi / 2 * t))] * 6
    return position, velocity, acceleration

def circular_motion(t):
    # Example: Circular motion in joint space
    position = [math.radians(45 * math.sin(t))] * 6
    velocity = [math.radians(45 * math.cos(t))] * 6
    acceleration = [math.radians(-45 * math.sin(t))] * 6
    return position, velocity, acceleration

def linear_transition(current_position, target_position, t, dt):
    # Simple linear interpolation with constant acceleration
    # Assuming desired velocity is constant for simplicity
    velocity = 0.1  # rad/s
    position = []
    velocity_cmd = []
    acceleration = []
    for i in range(6):
        # Update position
        pos = current_position[i] + velocity * dt
        # Clamp to target
        if (velocity > 0 and pos > target_position[i]) or (velocity < 0 and pos < target_position[i]):
            pos = target_position[i]
            vel = 0
            acc = 0
        else:
            vel = velocity
            acc = 0  # No acceleration
        position.append(pos)
        velocity_cmd.append(vel)
        acceleration.append(acc)
    return position, velocity_cmd, acceleration

def straight_line_motion(t, dt):
    global current_positions, target_positions, previous_positions, desired_accelerations
    return linear_transition(current_positions, target_positions, t, dt)

def fixed_motion():
    position = [math.radians(angle) for angle in [5.73, -57.3, -28.65, 0.0, 57.3, -5.73]]
    velocity = [0] * 6
    acceleration = [0] * 6
    return position, velocity, acceleration

def custom_angle_motion():
    global custom_angles
    position = [math.radians(angle) for angle in custom_angles]
    velocity = [0] * 6  # No velocity for custom angles
    acceleration = [0] * 6  # No acceleration
    return position, velocity, acceleration

def get_custom_angles():
    global custom_angles
    rospy.loginfo("Enter custom angles for each joint (6 values in degrees)")
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
    rospy.loginfo("Enter target positions for linear motion (6 values in degrees)")
    for i in range(6):
        while True:
            try:
                position = float(input(f"Enter target position for joint {i+1}: "))
                target_positions[i] = math.radians(position)  # Convert to radians
                break
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a valid number.")

def publish_motion(pub, motion_type, t, dt):
    global current_positions
    msg = Float64MultiArray()
    positions, velocities, accelerations = [], [], []
    
    if motion_type == 1:  # Sinusoidal
        positions, velocities, accelerations = sinusoidal_motion(t)
    elif motion_type == 2:  # Circular
        positions, velocities, accelerations = circular_motion(t)
    elif motion_type == 3:  # Linear
        positions, velocities, accelerations = straight_line_motion(t, dt)
    elif motion_type == 4:  # Fixed position
        positions, velocities, accelerations = fixed_motion()
    elif motion_type == 5:  # Custom angle position
        positions, velocities, accelerations = custom_angle_motion()
    
    # Update current positions directly after calculation
    current_positions = positions
    
    # Combine positions, velocities, and accelerations
    msg.data = positions + velocities + accelerations
    pub.publish(msg)

def publisher_thread(pub):
    global current_motion
    t = 0
    dt = 0.1  # Time step in seconds
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        publish_motion(pub, current_motion, t, dt)
        t += dt
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
