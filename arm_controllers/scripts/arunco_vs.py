#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf_conversions
import os

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node('aruco_detector_node')
        
        # Parameters
        self.marker_size = rospy.get_param('~marker_size', 0.05)  # Marker size in meters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.reference_frame = rospy.get_param('~reference_frame', self.camera_frame)
        self.use_rectified_images = rospy.get_param('~image_is_rectified', True)
        self.publish_position = rospy.get_param('~publish_position', False)
        
        # Desired pose parameters
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

        """
        # Save the image for debugging
        if not os.path.exists('debug_images'):
            os.makedirs('debug_images')
        image_filename = 'debug_images/image_{}.jpg'.format(rospy.Time.now())
        cv2.imwrite(image_filename, cv_image)
        """

        # Process detected markers
        if ids is not None:
            for i in range(len(ids)):
                rospy.logdebug("Marker detected: ID=%s" % ids[i])  # Detailed log for each detected marker
                # Process marker
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                self.process_marker_pose(ids[i], rvecs[i], tvecs[i], corners[i], msg.header)
        else:
            rospy.logdebug("No markers detected")  # Log when no markers are detected

        # To avoid cluttering the log, consider adding a counter to control how often this message appears
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

        # Publish TransformStamped message
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = f"aruco_marker_{marker_id[0]}"
        transform.transform.translation = pose.pose.position
        transform.transform.rotation = pose.pose.orientation
        self.transform_pub.publish(transform)
        self.br.sendTransform(transform)

        # **Optional:** Publish position if needed
        if self.publish_position:
            position = Vector3Stamped()
            position.header = pose.header
            position.vector.x = pose.pose.position.x
            position.vector.y = pose.pose.position.y
            position.vector.z = pose.pose.position.z
            self.position_pub.publish(position)

        # Publish visualization marker
        self.publish_marker(pose, marker_id)

        # Compute position error
        current_position = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ])
        error_position = self.desired_position - current_position

        # **Optional:** Compute orientation error (if needed)
        # Uncomment and adjust the following lines if orientation error is required
        """
        desired_orientation = np.array([0, 0, 0, 1])  # Example desired orientation (no rotation)
        current_orientation = np.array([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ])
        # Convert quaternions to Euler angles
        _, _, desired_yaw = euler_from_quaternion(desired_orientation)
        _, _, current_yaw = euler_from_quaternion(current_orientation)
        error_yaw = desired_yaw - current_yaw
        # Wrap yaw error to [-pi, pi]
        error_yaw = (error_yaw + np.pi) % (2 * np.pi) - np.pi
        # Combine position and orientation errors
        error = np.hstack((error_position, [error_yaw]))
        """
        # **For now, only position error is considered**
        error = error_position

        # Publish the error
        error_msg = Float32MultiArray()
        error_msg.data = error.tolist()
        self.error_pub.publish(error_msg)

        # **Optional:** Publish control input based on the error
        # Example: Simple Proportional Controller
        kp = np.array([1.0, 1.0, 1.0])  # Proportional gains for x, y, z
        control_input = kp * error_position  # P-controller

        control_msg = Float32MultiArray()
        control_msg.data = control_input.tolist()
        self.control_pub.publish(control_msg)

    def publish_marker(self, pose, marker_id):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "aruco_markers"
        marker.id = marker_id[0]
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.1  # Arrow length
        marker.scale.y = 0.02  # Arrow width
        marker.scale.z = 0.02  # Arrow height
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

def main():
    try:
        node = ArucoDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
