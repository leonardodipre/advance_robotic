import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool
import math

class CircularMotionMarker:
    def __init__(self):
        rospy.init_node('aruco_movment_node')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Initialize the marker's state
        self.state_msg = ModelState()
        self.state_msg.model_name = 'aruco_marker'

        # Parameters
        self.robot_x = 0.0  # X position of the robot (center of the circle)
        self.robot_y = 0.0  # Y position of the robot
        self.radius = 0.65   # Radius of the circular path
        self.angular_velocity = 0.01  # Speed of rotation (radians per second)
        self.is_moving = False  # Start with marker stationary
        self.theta = 0.0  # Starting angle

        # Subscriber to start/stop motion
        rospy.Subscriber('/start_marker_motion', Bool, self.motion_callback)

    def motion_callback(self, msg):
        self.is_moving = msg.data

    def move_in_circle(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.is_moving:
                # Calculate new X and Y positions for circular motion
                self.state_msg.pose.position.x = self.robot_x + self.radius * math.cos(self.theta)
                self.state_msg.pose.position.y = self.robot_y + self.radius * math.sin(self.theta)
                self.state_msg.pose.position.z = 0.0  # Keep Z position constant

                # Increment theta for the next position
                self.theta += self.angular_velocity / 10

                # Reset theta if it exceeds 2 * pi
                if self.theta >= 2 * math.pi:
                    self.theta -= 2 * math.pi

                # Send the new position to Gazebo
                self.set_state(self.state_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        marker_motion = CircularMotionMarker()
        marker_motion.move_in_circle()
    except rospy.ROSInterruptException:
        pass