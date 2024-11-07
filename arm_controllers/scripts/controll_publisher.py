#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import threading

def input_thread(mode_container):
    menu_options = """
Select control mode:
1: PD Controller
2: Velocity Controller
3: None
4: Kinematic Controller (Joint space)
5: Kinematic Task Space
6: Cartesian Coordinates
7: Aruco Tracker
8: Task space smooth transition
Enter your choice: """
    while not rospy.is_shutdown():
        try:
            input_mode = int(input(menu_options))
            if 1 <= input_mode <= 8:
                mode_container['mode'] = input_mode
            else:
                print("Invalid mode selected. Please select a number between 1 and 7.")
        except ValueError:
            print("Invalid input. Please enter a number between 1 and 7.")
        except EOFError:
            print("\nInput stream closed. Exiting input thread.")
            break
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Exiting input thread.")
            break

def main():
    rospy.init_node('control_publisher')
    mode_pub = rospy.Publisher('/control_mode', Int32, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    mode_container = {'mode': 1}

    thread = threading.Thread(target=input_thread, args=(mode_container,))
    thread.daemon = True
    thread.start()

    try:
        while not rospy.is_shutdown():
            mode_msg = Int32()
            mode_msg.data = mode_container['mode']
            mode_pub.publish(mode_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
