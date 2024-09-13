#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

//small sim for publicer the clock without runing gazebo
int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_clock");
    ros::NodeHandle nh;
    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    ROS_INFO("Clock setup");

    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = ros::Time::now();
        clock_pub.publish(clock_msg);
        rate.sleep();
    }

    return 0;
}
