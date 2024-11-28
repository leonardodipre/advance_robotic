import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool
import math

class SinusoidalMotionMarker:
    def __init__(self):
        rospy.init_node('aruco_movment_node')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Initialize the marker's state
        self.state_msg = ModelState()
        self.state_msg.model_name = 'aruco_marker'

        # Parameters
        self.robot_x = 0.0  # X position of the robot (center of the path)
        self.robot_y = 0.0  # Y position of the robot
        self.base_radius = 0.5   # Base radius of the path
        self.angular_velocity = 0.1  # Speed of rotation (radians per second)
        self.is_moving = False  # Start with marker stationary
        self.theta = 0.0  # Starting angle

        # Sine wave parameters
        self.amplitude = 0.05   # Amplitude of the sine wave
        self.num_cycles = 1    # Number of sine wave cycles around the circle

        # Subscriber to start/stop motion
        rospy.Subscriber('/start_marker_motion', Bool, self.motion_callback)

    def motion_callback(self, msg):
        self.is_moving = msg.data

    def move_in_sine_wave(self):
        rate = rospy.Rate(50)  # 10 Hz

        while not rospy.is_shutdown():
            if self.is_moving:
                # Calculate new radius with sine wave modulation
                r = self.base_radius + self.amplitude * math.sin(self.num_cycles * self.theta)
                
                # Calculate new X and Y positions
                self.state_msg.pose.position.x = self.robot_x + r * math.cos(self.theta)
                self.state_msg.pose.position.y = self.robot_y + r * math.sin(self.theta)
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
        marker_motion = SinusoidalMotionMarker()
        marker_motion.move_in_sine_wave()
    except rospy.ROSInterruptException:
        pass