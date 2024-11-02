#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped, Twist, PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf_conversions
from arm_controllers.msg import ArucoTracker  # Import your custom message

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node('aruco_detector_node')
        
        # Parameters
        self.marker_size = rospy.get_param('~marker_size', 0.1)  # Marker size in meters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.reference_frame = rospy.get_param('~reference_frame', self.camera_frame)
        self.use_rectified_images = rospy.get_param('~image_is_rectified', True)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        self.transform_pub = rospy.Publisher('transform', TransformStamped, queue_size=10)
        self.position_pub = rospy.Publisher('position', Vector3Stamped, queue_size=10)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.pixel_pub = rospy.Publisher('pixel', PointStamped, queue_size=10)
        self.image_pub = rospy.Publisher('result', Image, queue_size=10)
        self.debug_pub = rospy.Publisher('debug', Image, queue_size=10)
        self.vel_pub = rospy.Publisher('marker_velocity', Twist, queue_size=10)
        self.aruco_tracker_pub = rospy.Publisher('aruco_tracker', ArucoTracker, queue_size=10)
        
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        
        # CvBridge
        self.bridge = CvBridge()
        
        # Camera Parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cam_info_received = False

        # ArUco Detection Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

        self.parameters = aruco.DetectorParameters_create()
        
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
        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose of each marker
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                rospy.logdebug("Marker detected: ID=%s" % ids[i])
                rospy.loginfo("Marker Detected")
                self.process_marker_pose(ids[i], rvecs[i], tvecs[i], corners[i], msg.header)
        else:
            rospy.logdebug("No markers detected")
            rospy.loginfo("Marker NON Detected")

        # Publish result image with markers drawn
        if self.image_pub.get_num_connections() > 0:
            try:
                result_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                result_msg.header = msg.header
                self.image_pub.publish(result_msg)
            except CvBridgeError as e:
                rospy.logerr("Failed to convert image: %s" % e)

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

        # Create PoseStamped message in the camera frame
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[0][1]
        pose_msg.pose.position.z = tvec[0][2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # Publish pose
        self.pose_pub.publish(pose_msg)

        # Publish TransformStamped
        transform_stamped = TransformStamped()
        transform_stamped.header = pose_msg.header
        transform_stamped.child_frame_id = "aruco_marker_{}".format(marker_id)
        transform_stamped.transform.translation.x = tvec[0][0]
        transform_stamped.transform.translation.y = tvec[0][1]
        transform_stamped.transform.translation.z = tvec[0][2]
        transform_stamped.transform.rotation = pose_msg.pose.orientation

        self.transform_pub.publish(transform_stamped)

        # Send transform to TF
        self.br.sendTransform(transform_stamped)

        # Publish position as Vector3Stamped
        position_msg = Vector3Stamped()
        position_msg.header = pose_msg.header
        position_msg.vector.x = tvec[0][0]
        position_msg.vector.y = tvec[0][1]
        position_msg.vector.z = tvec[0][2]
        self.position_pub.publish(position_msg)

        # Publish marker for visualization in RViz
        marker = Marker()
        marker.header = pose_msg.header
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.001  # Thin cube to represent the marker plane
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(3.0)
        self.marker_pub.publish(marker)

        # Publish pixel coordinates
        pixel_msg = PointStamped()
        pixel_msg.header = pose_msg.header
        # Compute center of the marker in pixel coordinates
        c = corner[0]
        pixel_msg.point.x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0
        pixel_msg.point.y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0
        pixel_msg.point.z = 0
        self.pixel_pub.publish(pixel_msg)

        # Velocity calculation
        current_time = rospy.Time.now().to_sec()
        current_position = np.array([tvec[0][0], tvec[0][1], tvec[0][2]])

        if self.prev_time is not None:
            delta_time = current_time - self.prev_time
            if delta_time > 0:
                velocity = (current_position - self.prev_position) / delta_time
                # Publish velocity as Twist message
                twist_msg = Twist()
                twist_msg.linear.x = velocity[0]
                twist_msg.linear.y = velocity[1]
                twist_msg.linear.z = velocity[2]
                self.vel_pub.publish(twist_msg)
        else:
            velocity = np.array([0.0, 0.0, 0.0])

        self.prev_position = current_position
        self.prev_time = current_time

        # Publish ArucoTracker message
        tracker_msg = ArucoTracker()
        tracker_msg.header = pose_msg.header
        tracker_msg.position.x = tvec[0][0]
        tracker_msg.position.y = tvec[0][1]
        tracker_msg.position.z = tvec[0][2]
        tracker_msg.orientation.x = quat[0]
        tracker_msg.orientation.y = quat[1]
        tracker_msg.orientation.z = quat[2]
        tracker_msg.orientation.w = quat[3]
        tracker_msg.velocity.x = velocity[0]
        tracker_msg.velocity.y = velocity[1]
        tracker_msg.velocity.z = velocity[2]
        self.aruco_tracker_pub.publish(tracker_msg)

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    node = ArucoDetectorNode()
    node.main()
