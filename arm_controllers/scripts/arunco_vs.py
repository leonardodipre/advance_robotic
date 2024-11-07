#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
import tf_conversions

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node('aruco_detector_node', anonymous=True)
        #sleep(5)  # Delay to allow other nodes to start and publish transforms

        self.marker_size = rospy.get_param('~marker_size', 0.1)  # Marker size in meters
        self.camera_frame = rospy.get_param('~camera_frame', 'relasense_color_optical_frame')
        self.base_frame = rospy.get_param('~base_frame', 'elfin_base_link')

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)

        self.pose_pub = rospy.Publisher('aruco_pose', PoseStamped, queue_size=10)
        self.transformed_pose_pub = rospy.Publisher('transformed_aruco_pose', PoseStamped, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cam_info_received = False
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()

    def cam_info_callback(self, msg):
        if not self.cam_info_received:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.cam_info_received = True
            self.cam_info_sub.unregister()  # No need to listen to further messages
            rospy.loginfo("Camera info received")

    def image_callback(self, msg):
        if not self.cam_info_received:
            return  # Wait until camera info is available

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
            if ids is not None:
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                for rvec, tvec in zip(rvecs, tvecs):
                    self.publish_pose(msg.header, rvec, tvec)
            else:
                rospy.logdebug("No markers detected")
        except CvBridgeError as e:
            rospy.logerr("Failed to convert image: %s" % e)

    def publish_pose(self, header, rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = tf_conversions.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, tvec.T)), [0, 0, 0, 1])))

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

        self.pose_pub.publish(pose_msg)

        # Wait for the transform to be available
        while not self.tf_buffer.can_transform(self.base_frame, pose_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0)) and not rospy.is_shutdown():
            rospy.logwarn("Waiting for transform to become available...")
            #time.sleep(0.5)  # wait for half a second before checking again

        # Once available, do the transformation
        try:
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time(0)))
            self.transformed_pose_pub.publish(pose_transformed)
            rospy.loginfo("Transformed ArUco marker pose published")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('Failed to transform pose: {}'.format(e))

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    node = ArucoDetectorNode()
    node.main()
