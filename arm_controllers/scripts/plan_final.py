#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import threading

# Variabile globale per memorizzare il comando corrente
current_command = 0
# Evento per gestire l'uscita del thread
shutdown_event = threading.Event()

def menu(command_pub):
    global current_command
    while not shutdown_event.is_set():
        print("\nMenu:")
        print("1. Move to detected object")
        print("2. Empty")
        print("4. Exit")
        choice = input("Enter your choice: ")

        if choice == '1':
            current_command = 1
            rospy.loginfo("Comando impostato a 1: Move to detected object")
            command_pub.publish(current_command)
        elif choice == '2':
            current_command = 2
            rospy.loginfo("Comando impostato a 2: Empty")
            command_pub.publish(current_command)
        elif choice == '4':
            rospy.loginfo("Uscita dal programma.")
            shutdown_event.set()
            rospy.signal_shutdown("Uscita richiesta dall'utente.")
        else:
            print("Scelta non valida. Per favore, riprova.")

def main():
    global current_command
    rospy.init_node('plan_final')
    command_pub = rospy.Publisher('/controller_command', Int32, queue_size=10)

    # Inizia il thread del menu
    menu_thread = threading.Thread(target=menu, args=(command_pub,))
    menu_thread.start()

    # Attendi che il thread del menu termini
    menu_thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
