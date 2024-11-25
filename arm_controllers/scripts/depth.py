#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
import tf_conversions

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node('aruco_detector_node', anonymous=True)
        rospy.loginfo("Initializing Aruco Detector Node")

        # Parameters
        self.marker_size = rospy.get_param('~marker_size', 0.1)  # Marker size in meters
        self.camera_frame = rospy.get_param('~camera_frame', 'relasense_color_optical_frame')
        self.base_frame = rospy.get_param('~base_frame', 'elfin_base_link')

        rospy.loginfo(f"Camera Frame: {self.camera_frame}")
        rospy.loginfo(f"Base Frame: {self.base_frame}")

        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)

        # New subscriber for /detection/object_poses
        self.object_poses_sub = rospy.Subscriber('/detection/object_poses', PoseArray, self.object_poses_callback)


        # Publishers
        self.pose_pub = rospy.Publisher('aruco_pose', PoseStamped, queue_size=10)
        self.transformed_pose_pub = rospy.Publisher('transformed_aruco_pose', PoseStamped, queue_size=10)
        self.transformed_object_pose_pub = rospy.Publisher('transformed_object_poses', PoseArray, queue_size=10)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # TF buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # CV Bridge
        self.bridge = CvBridge()

        # Camera Intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cam_info_received = False

        # ArUco Dictionary and Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()

    def cam_info_callback(self, msg):
        if not self.cam_info_received:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.cam_info_received = True
            self.cam_info_sub.unregister()  # Unsubscribe after receiving camera info
            rospy.loginfo("Camera info received and camera_info subscriber unsubscribed.")

    def image_callback(self, msg):
        if not self.cam_info_received:
            rospy.logwarn("Waiting for camera info...")
            return  # Wait until camera info is available

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for rvec, tvec in zip(rvecs, tvecs):
                self.publish_pose(msg.header, rvec, tvec)
        else:
            rospy.logdebug("No ArUco markers detected in the image.")

    def publish_pose(self, header, rvec, tvec):
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = tf_conversions.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, tvec.T)), [0, 0, 0, 1]))
        )

        # Create PoseStamped message in camera frame
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[0][1]
        pose_msg.pose.position.z = tvec[0][2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # Publish the pose relative to the camera frame
        self.pose_pub.publish(pose_msg)
        rospy.loginfo("ArUco marker pose published in camera frame.")

        # Attempt to transform the pose to the base frame
        try:
            # Wait for the transform to become available (up to 1 second)
            if self.tf_buffer.can_transform(self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(1.0)):
                # Lookup the latest available transform
                transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time(0))

                # Transform the pose
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)

                # Publish the transformed pose
                self.transformed_pose_pub.publish(pose_transformed)
                rospy.loginfo("Transformed ArUco marker pose published in base frame.")
            else:
                rospy.logwarn("Transform from camera frame to base frame not available.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform pose: {e}")

    def object_poses_callback(self, msg):
        # msg is of type PoseArray
        transformed_pose_array = PoseArray()
        transformed_pose_array.header = msg.header
        transformed_pose_array.header.frame_id = self.base_frame

        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose

            # Transform pose to base frame
            try:
                # Wait for the transform to become available (up to 1 second)
                if self.tf_buffer.can_transform(self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(1.0)):
                    # Lookup the latest available transform
                    transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time(0))

                    # Transform the pose
                    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                    transformed_pose_array.poses.append(pose_transformed.pose)
                else:
                    rospy.logwarn("Transform from camera frame to base frame not available for object poses.")
                    return
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Failed to transform object pose: {e}")
                return

        # Publish the transformed PoseArray
        self.transformed_object_pose_pub.publish(transformed_pose_array)
        rospy.loginfo("Transformed object poses published in base frame.")

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = ArucoDetectorNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
