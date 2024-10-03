#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import threading

def input_thread(mode_container):
    while not rospy.is_shutdown():
        try:
            input_mode = int(input("Select control mode (1: position, 2: position + velocity, 3: PID, 4: Kineptica (Joint, space) , 5:Kinematic task space: "))
            if 1 <= input_mode <= 5:
                mode_container['mode'] = input_mode
            else:
                print("Invalid mode selected. Please select a number between 1 and 5.")
        except ValueError:
            print("Invalid input. Please enter a number between 1 and 4.")
        except EOFError:
            print("\nInput stream closed. Exiting input thread.")
            break
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Exiting input thread.")
            break

def main():
    rospy.init_node('controll_publisher')
    mode_pub = rospy.Publisher('/control_mode', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

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
