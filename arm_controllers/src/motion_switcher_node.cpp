#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>
#include <unistd.h>

// Function to get a single character from stdin
char getch()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_switcher_node");
    ros::NodeHandle nh;

    ros::Publisher motion_command_pub = nh.advertise<std_msgs::Float64MultiArray>("/motion_command", 10);

    ros::Rate loop_rate(10); // 10 Hz

    bool circular_motion = false; // Start with sinusoidal motion by default

    ROS_INFO("Press s or c for changing the movemnt of the robot");
        

    while (ros::ok())
    {
        char ch = getch(); // Get a single character from keyboard input

        if (ch == 'c' || ch == 'C') {
            circular_motion = true;
        } else if (ch == 's' || ch == 'S') {
            circular_motion = false;
        }

        std_msgs::Float64MultiArray msg;
        msg.data.clear(); // Clear previous data
        if (circular_motion) {
            msg.data.push_back(1.0); // Set to circular motion
            ROS_INFO("Published command: Circular Motion (1.0)");
        } else {
            msg.data.push_back(0.0); // Set to sinusoidal motion
            ROS_INFO("Published command: Sinusoidal Motion (0.0)");
        }

        motion_command_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
