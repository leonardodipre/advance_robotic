#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

# Define the obstacle position
obstacle_position = Point(2, 2, 0)  # Example position, adjust accordingly

def calculate_repulsive_force(robot_position):
    # Constants
    influence_distance = 1.0  # Max distance at which the obstacle influences the robot
    scaling_factor = 0.5  # Scaling factor for the force

    # Calculate the distance to the obstacle
    distance = ((robot_position.x - obstacle_position.x) ** 2 +
                (robot_position.y - obstacle_position.y) ** 2 +
                (robot_position.z - obstacle_position.z) ** 2) ** 0.5

    force = Vector3()

    if distance < influence_distance:
        # Calculate the direction vector from the obstacle to the robot
        force.x = robot_position.x - obstacle_position.x
        force.y = robot_position.y - obstacle_position.y
        force.z = robot_position.z - obstacle_position.z

        # Normalize the force vector and scale it
        force_magnitude = (force.x ** 2 + force.y ** 2 + force.z ** 2) ** 0.5
        force.x = (force.x / force_magnitude) * scaling_factor
        force.y = (force.y / force_magnitude) * scaling_factor
        force.z = (force.z / force_magnitude) * scaling_factor

    return force

def publish_forces(force):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.1  # Adjust the scale based on your visualization needs
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = 0  # Robot's current position
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1  # Neutral orientation
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(force.x, force.y, force.z))
    pub.publish(marker)

rospy.init_node('potential_fields_static')
pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

# Example robot position for demonstration
robot_position = Point(1, 1, 0)
force = calculate_repulsive_force(robot_position)
publish_forces(force)

rospy.spin()
