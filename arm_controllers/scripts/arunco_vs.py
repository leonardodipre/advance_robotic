#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped, Twist
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf_conversions
import os

from arm_controllers.msg import ArucoTracker  # Import your custom message

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node('aruco_detector_node')
        
        # Parameters
        self.marker_size = rospy.get_param('~marker_size', 0.3)  # Marker size in meters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.reference_frame = rospy.get_param('~reference_frame', self.camera_frame)
        self.use_rectified_images = rospy.get_param('~image_is_rectified', True)
        self.publish_position = rospy.get_param('~publish_position', False)
        
        # Desired pose parameters to point with the end-effector
        self.desired_position = np.array([
            rospy.get_param('~desired_x', 0.0),
            rospy.get_param('~desired_y', 0.0),
            rospy.get_param('~desired_z', 0.5)
        ])
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        self.transform_pub = rospy.Publisher('transform', TransformStamped, queue_size=10)
        if self.publish_position:
            self.position_pub = rospy.Publisher('position', Vector3Stamped, queue_size=10)
        self.error_pub = rospy.Publisher('pose_error', Float32MultiArray, queue_size=10)
        self.control_pub = rospy.Publisher('control_input', Float32MultiArray, queue_size=10)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.vel_pub = rospy.Publisher('marker_velocity', Twist, queue_size=10)  # Velocity publisher
        
        # Correct the publisher by using the correct message type: ArucoTracker
        self.aruco_tracker_pub = rospy.Publisher('aruco_tracker', ArucoTracker, queue_size=10)  # Publisher for ArucoTracker
        
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        
        # CvBridge
        self.bridge = CvBridge()
        
        # Camera Parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cam_info_received = False

        # ArUco Detection Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        # Frame counter
        self.frame_count = 0
        
        # For velocity calculation
        self.prev_time = None
        self.prev_position = None

        rospy.loginfo("ArucoDetectorNode initialized")

    def cam_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        self.cam_info_received = True
        rospy.loginfo("Camera info received")
        self.cam_info_sub.unregister()  # Unsubscribe after receiving camera info

    def image_callback(self, msg):
        if not self.cam_info_received:
            rospy.logdebug("Waiting for camera info")
            return

        # Convert ROS image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("Failed to convert image: %s" % e)
            return

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        # Draw detected markers for visualization
        aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Process detected markers
        if ids is not None:
            for i in range(len(ids)):
                rospy.logdebug("Marker detected: ID=%s" % ids[i])  # Detailed log for each detected marker
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                self.process_marker_pose(ids[i], rvecs[i], tvecs[i], corners[i], msg.header)
        else:
            rospy.logdebug("No markers detected")  # Log when no markers are detected

        if self.frame_count % 10 == 0:
            rospy.loginfo("Processed frame %s: Markers detected: %s" % (
                self.frame_count, len(ids) if ids is not None else 0))
        self.frame_count += 1

    def process_marker_pose(self, marker_id, rvec, tvec, corner, header):
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        # Compute the quaternion from the rotation matrix
        quat = tf_conversions.transformations.quaternion_from_matrix(
            np.vstack((
                np.hstack((rotation_matrix, np.array([[0], [0], [0]]))),
                [0, 0, 0, 1]
            ))
        )

        # Create PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = tvec[0][0]
        pose.pose.position.y = tvec[0][1]
        pose.pose.position.z = tvec[0][2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose)

        # Initialize velocity to zero in case it cannot be calculated
        velocity = np.array([0.0, 0.0, 0.0])

        # Calculate velocity if previous position is available
        if self.prev_position is not None:
            current_position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            current_time = rospy.Time.now().to_sec()

            # Calculate time difference
            delta_time = current_time - self.prev_time

            if delta_time > 0:
                # Calculate velocity
                velocity = (current_position - self.prev_position) / delta_time

                # Publish velocity as a Twist message
                twist_msg = Twist()
                twist_msg.linear.x = velocity[0]
                twist_msg.linear.y = velocity[1]
                twist_msg.linear.z = velocity[2]
                self.vel_pub.publish(twist_msg)

        # Update previous position and time
        self.prev_position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        self.prev_time = rospy.Time.now().to_sec()

        # Compute position error
        current_position = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ])

        # Calculate the error between the current position of the end-effector and the desired position
        error_position = self.desired_position - current_position

        # Only position error is considered
        error = error_position

        # Publish the error
        error_msg = Float32MultiArray()
        error_msg.data = error.tolist()
        self.error_pub.publish(error_msg)

        # Publish position + velocity using the ArucoTracker message
        tracker_msg = ArucoTracker()  # Corrected message creation
        tracker_msg.header.stamp = rospy.Time.now()

        # Set position
        tracker_msg.position.x = tvec[0][0]
        tracker_msg.position.y = tvec[0][1]
        tracker_msg.position.z = tvec[0][2]

        # Set velocity (this will be zero if we cannot calculate it)
        tracker_msg.velocity.x = velocity[0]
        tracker_msg.velocity.y = velocity[1]
        tracker_msg.velocity.z = velocity[2]

        # Publish the message
        self.aruco_tracker_pub.publish(tracker_msg)

def main():
    try:
        node = ArucoDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
