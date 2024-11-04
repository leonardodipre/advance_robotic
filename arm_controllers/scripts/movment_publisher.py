#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math
import threading
import numpy as np
from math import sin, cos

# Global variables
current_motion = 1  # Default to fixed motion at startup
custom_angles = [0] * 6  # Default custom angles for all 6 joints
current_positions = [0] * 6  # Store current joint positions
target_positions = [0] * 6  # Target positions for linear motion

task_coords = [0] * 6
task_vel = [0] * 6
# For the new linear loop motion
A_positions = [0] * 6  # Starting joint angles for position A
B_positions = [0] * 6  # Ending joint angles for position B

def print_menu():
    rospy.loginfo("\nMenu:")
    rospy.loginfo("1 - Fixed Motion (default)")
    rospy.loginfo("2 - Sinusoidal Motion")
    rospy.loginfo("3 - Custom Angles")
    rospy.loginfo("4 - Linear Loop Motion between A and B (joint)")
    rospy.loginfo("5 - Task space")
    rospy.loginfo("6 - Set Task Space Coordinates")
    rospy.loginfo("7 - Exit")

def get_motion_choice():
    choice = input("Enter your choice (1-7): ")
    try:
        choice = int(choice)
        if choice in [1, 2, 3, 4, 5, 6, 7]:
            return choice
        else:
            rospy.loginfo("Invalid choice. Enter a number between 1 and 7.")
            return None
    except ValueError:
        rospy.loginfo("Invalid input. Enter a number.")
        return None

def sinusoidal_motion(t):
    position = [math.radians(45 * math.sin(math.pi / 2 * t))] * 6
    velocity = [math.radians(45 * (math.pi / 2) * math.cos(math.pi / 2 * t))] * 6
    acceleration = [math.radians(-45 * (math.pi / 2)**2 * math.sin(math.pi / 2 * t))] * 6
    return position, velocity, acceleration, task_coords, task_vel

def linear_transition(current_position, target_position, t, dt):
    # Simple linear interpolation with constant velocity
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
    return position, velocity_cmd, acceleration, task_coords, task_vel

def straight_line_motion(t, dt):
    global current_positions, target_positions
    return linear_transition(current_positions, target_positions, t, dt)

def fixed_motion():
    position = [math.radians(angle) for angle in [0.0, -25.0, 60.0, 0.0, 65.0, 0.0]]
    velocity = [0] * 6
    acceleration = [0] * 6
    return position, velocity, acceleration, task_coords, task_vel

def task_space_function(t, dt):
    # Trajectory parameters
    A = 0.2          # Amplitude (m)
    omega = 0.1      # Angular frequency (rad/s)
    height = 1.0     # Height above the ground (m)

    x = A * np.sin(omega * t)
    y = A * np.sin(omega * t) * np.cos(omega * t)
    z = height

    # Velocities
    xd = A * omega * np.cos(omega * t)
    yd = A * omega * (np.cos(2 * omega * t) - np.sin(omega * t)**2)
    zd = 0.0

    # Limit velocities
    max_velocity = 0.5  # m/s
    velocity_magnitude = np.sqrt(xd**2 + yd**2)
    if velocity_magnitude > max_velocity:
        scale = max_velocity / velocity_magnitude
        xd *= scale
        yd *= scale

    # Task space coordinates and velocities
    task_coords = [x, y, z, 0.0, 0.0, 0.0]
    task_vel = [xd, yd, zd, 0.0, 0.0, 0.0]

    # Joint space placeholders
    position = [0.0] * 6
    velocity = [0.0] * 6
    acceleration = [0.0] * 6

    return position, velocity, acceleration, task_coords, task_vel

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

def get_AB_positions():
    global A_positions, B_positions
    rospy.loginfo("Enter joint angles for position A (6 values in degrees)")
    A_positions = []
    for i in range(6):
        while True:
            try:
                angle = float(input(f"Enter angle for joint {i+1} for position A: "))
                A_positions.append(math.radians(angle))
                break
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a valid number.")
    rospy.loginfo("Enter joint angles for position B (6 values in degrees)")
    B_positions = []
    for i in range(6):
        while True:
            try:
                angle = float(input(f"Enter angle for joint {i+1} for position B: "))
                B_positions.append(math.radians(angle))
                break
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a valid number.")

def linear_loop_motion(t):
    global A_positions, B_positions
    # Define frequency (cycles per second)
    f = 0.3  # Adjust as needed for desired speed
    omega = 2 * math.pi * f  # Angular frequency

    # Compute s(t)
    s = 0.5 * (1 - math.cos(omega * t))
    ds_dt = 0.5 * omega * math.sin(omega * t)
    d2s_dt2 = 0.5 * omega**2 * math.cos(omega * t)

    # Compute positions, velocities, accelerations
    positions = []
    velocities = []
    accelerations = []
    for i in range(6):
        delta = B_positions[i] - A_positions[i]
        pos = A_positions[i] + s * delta
        vel = ds_dt * delta
        acc = d2s_dt2 * delta
        positions.append(pos)
        velocities.append(vel)
        accelerations.append(acc)
    return positions, velocities, accelerations, task_coords, task_vel

def set_task_space_coordinates():
    global task_coords, task_vel
    rospy.loginfo("Enter Task Space Coordinates (x, y, z):")
    task_coords_input = []
    for coord in ["x", "y", "z"]:
        while True:
            try:
                value = float(input(f"Enter {coord}-coordinate (meters): "))
                task_coords_input.append(value)
                break
            except ValueError:
                rospy.loginfo("Invalid input. Please enter a valid number.")
    
    # Update task_coords with x, y, z values and reset velocities
    task_coords[:3] = task_coords_input
    task_coords[3:] = [0.0, 0.0, 0.0]  # Reset the remaining task_coords
    task_vel = [0.0] * 6
    rospy.loginfo(f"Task Space Coordinates set to x={task_coords[0]}, y={task_coords[1]}, z={task_coords[2]}")

def publish_motion(pub, motion_type, t, dt):
    global current_positions, task_coords, task_vel
    msg = Float64MultiArray()

    if motion_type == 1:  # Fixed Motion
        positions, velocities, accelerations, _, _ = fixed_motion()
    elif motion_type == 2:  # Sinusoidal Motion
        positions, velocities, accelerations, _, _ = sinusoidal_motion(t)
    elif motion_type == 3:  # Custom Angles
        positions, velocities, accelerations, _, _ = straight_line_motion(t, dt)
    elif motion_type == 4:  # Linear Loop Motion
        positions, velocities, accelerations, _, _ = linear_loop_motion(t)
    elif motion_type == 5:  # Task space
        positions, velocities, accelerations, _, _ = task_space_function(t, dt)
    elif motion_type == 6:  # Set Task Space Coordinates
        positions = [0.0] * 6  # All joint positions set to 0
        velocities = [0.0] * 6
        accelerations = [0.0] * 6
    else:
        rospy.loginfo("Unknown motion type. Defaulting to Fixed Motion.")
        positions, velocities, accelerations, _, _ = fixed_motion()

    if motion_type == 6:
        # Only task_coords and task_vel are populated
        msg.data = positions + velocities + accelerations + task_coords + task_vel
    else:
        # Combine positions, velocities, accelerations, task_coords, and task_vel
        msg.data = positions + velocities + accelerations + task_coords + task_vel

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
            if choice == 7:
                rospy.loginfo("Exiting...")
                rospy.signal_shutdown("User requested exit.")
                break
            elif choice == 3:
                get_target_positions()
                current_motion = choice
                rospy.loginfo("Target positions set and motion changed to Custom Angles")
            elif choice == 4:
                get_AB_positions()
                current_motion = choice
                rospy.loginfo("Positions A and B set and motion changed to Linear Loop Motion")
            elif choice == 6:
                set_task_space_coordinates()
                current_motion = choice
                rospy.loginfo("Task Space Coordinates set and motion changed to Set Task Space Coordinates")
            else:
                current_motion = choice
                rospy.loginfo(f"Motion changed to option {choice}")

def main():
    global current_motion
    rospy.init_node('motion_menu_publisher', anonymous=True)
    pub = rospy.Publisher('/motion_command', Float64MultiArray, queue_size=10)

    # Start the publisher thread that continuously publishes the motion
    pub_thread = threading.Thread(target=publisher_thread, args=(pub,))
    pub_thread.daemon = True  # Ensure thread exits when main program does
    pub_thread.start()

    # Start the menu thread to allow users to change motion types
    menu_thread()

if __name__ == '__main__':
    main()
