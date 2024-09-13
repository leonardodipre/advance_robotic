#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math
import threading

# Global variable for current motion type
current_motion = 4  # Default to fixed motion at startup

def print_menu():
    rospy.loginfo("\nMenu:")
    rospy.loginfo("1 - Sinusoidal Motion")
    rospy.loginfo("2 - Circular Motion")
    rospy.loginfo("3 - Linear Motion")
    rospy.loginfo("4 - Fixed Motion (default)")
    rospy.loginfo("5 - Exit")

def get_motion_choice():
    choice = input("Enter your choice (1-5): ")
    try:
        choice = int(choice)
        if choice in [1, 2, 3, 4, 5]:
            return choice
        else:
            rospy.loginfo("Invalid choice. Enter a number between 1 and 5.")
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

def linear_motion(t):
    position = [t] * 6
    velocity = [1.0] * 6
    return position, velocity

def fixed_motion():
    position = [0.1, -1, -0.5, 0.0, 1, -0.1]
    velocity = [0] * 6
    return position, velocity



def publish_motion(pub, motion_type, t):
    msg = Float64MultiArray()
    positions, velocities = [], []
    
    if motion_type == 1:  # Sinusoidal
        positions, velocities = sinusoidal_motion(t)
        #rospy.loginfo("Publishing Sinusoidal Motion")
    elif motion_type == 2:  # Circular
        positions, velocities = circular_motion(t)
        #rospy.loginfo("Publishing Circular Motion")
    elif motion_type == 3:  # Linear
        positions, velocities = linear_motion(t)
        #rospy.loginfo("Publishing Linear Motion")
    elif motion_type == 4:  # Fixed position
        positions, velocities = fixed_motion()
        #rospy.loginfo("Publishing Fixed Motion (Default)")
    
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
    global current_motion  # Use global to update motion type in the publisher thread
    while not rospy.is_shutdown():
        print_menu()
        choice = get_motion_choice()

        if choice is not None:
            if choice == 5:
                rospy.loginfo("Exiting...")
                rospy.signal_shutdown("User requested exit.")
                break
            else:
                current_motion = choice  # Update the motion type
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
