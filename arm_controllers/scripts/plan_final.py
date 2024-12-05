#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String
import threading

# Variabile globale per memorizzare il comando corrente
current_command = 0
# Evento per gestire l'uscita del thread
shutdown_event = threading.Event()

def menu(command_pub, prompt_pub):
    global current_command
    while not shutdown_event.is_set():
        print("\nMenu:")
        print("0. Stationary")
        print("1. Move to detected object")
        print("2. Prompt for detection")
        print("3. Push")
        print("4. Trajectory planning")
        print("5. Exit")
        choice = input("Enter your choice: ")

        if choice == '1':
            current_command = 1
            rospy.loginfo("Comando impostato a 1: Move to detected object")
            command_pub.publish(current_command)
        elif choice == '0':
            current_command = 0
            rospy.loginfo("Comando impostato a 0: Stationary")
            command_pub.publish(current_command)
        elif choice == '2':
            current_command = 2
            rospy.loginfo("Comando impostato a 2: Prompt for detection")
            # Ask the user for the prompt
            prompt = input("Enter the detection prompt: ")
            # Publish the prompt to the appropriate topic
            prompt_msg = String()
            prompt_msg.data = prompt
            prompt_pub.publish(prompt_msg)
            rospy.loginfo(f"Prompt '{prompt}' published to /detection/prompt")
        elif choice == '3':
            current_command = 3
            rospy.loginfo("Comando impostato a 3: Push obj")
            command_pub.publish(current_command)
        elif choice == '4':
            current_command = 4
            rospy.loginfo("Comando impostato a 3: Trj planning")
            command_pub.publish(current_command)
        elif choice == '5':
            rospy.loginfo("Exiting the program.")
            shutdown_event.set()
            rospy.signal_shutdown("User requested shutdown.")
        else:
            print("Invalid choice. Please try again.")

def main():
    global current_command
    rospy.init_node('plan_final')
    command_pub = rospy.Publisher('/controller_command', Int32, queue_size=10)
    prompt_pub = rospy.Publisher('/detection/prompt', String, queue_size=10)

    # Start the menu thread
    menu_thread = threading.Thread(target=menu, args=(command_pub, prompt_pub))
    menu_thread.start()

    # Wait for the menu thread to finish
    menu_thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
