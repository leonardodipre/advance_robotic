import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped, PointStamped
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
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        self.transform_pub = rospy.Publisher('transform', TransformStamped, queue_size=10)
        self.position_pub = rospy.Publisher('position', Vector3Stamped, queue_size=10)
        self.marker_pub = rospy.Publisher('marker', Marker, queue_size=10)
        self.pixel_pub = rospy.Publisher('pixel', PointStamped, queue_size=10)
        
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

        rospy.loginfo("ArucoDetectorNode initialized")

    def cam_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        self.cam_info_received = True
        rospy.loginfo("Camera info received")
        self.cam_info_sub.unregister()  # Unsubscribe after receiving camera info

    def image_callback(self, msg):
        rospy.loginfo("Image received")
        if not self.cam_info_received:
            rospy.loginfo("Waiting for camera info")
            return
        
        # Convert ROS image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("Failed to convert image: %s" % e)
            return
        
        # Detect ArUco markers
        rospy.loginfo("Detecting markers")
        corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        
        # Draw detected markers for visualization
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Save the image for debugging
        if not os.path.exists('debug_images'):
            os.makedirs('debug_images')
        image_filename = 'debug_images/image_{}.jpg'.format(rospy.Time.now())
        cv2.imwrite(image_filename, cv_image)
        rospy.loginfo("Saved image to {}".format(image_filename))
        
        if ids is not None:
            rospy.loginfo("Detected markers: %s" % ids)
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                # Pass the specific corner for the current marker
                self.publish_marker_pose(ids[i], rvecs[i], tvecs[i], corners[i], msg.header)
        else:
            rospy.loginfo("No markers detected")

    def publish_marker_pose(self, marker_id, rvec, tvec, corner, header):
        rospy.loginfo("Publishing pose for marker: %s" % marker_id)
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Create a 4x4 transformation matrix
        transformation_matrix = np.identity(4)
        transformation_matrix[:3, :3] = rotation_matrix

        # Compute the quaternion from the transformation matrix
        quat = tf_conversions.transformations.quaternion_from_matrix(transformation_matrix)
        
        # Create and send the TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.reference_frame
        transform.child_frame_id = f"aruco_marker_{marker_id[0]}"  # Assuming marker_id is array-like
        transform.transform.translation.x = tvec[0][0]
        transform.transform.translation.y = tvec[0][1]
        transform.transform.translation.z = tvec[0][2]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.br.sendTransform(transform)
        self.transform_pub.publish(transform)  # Publish to 'transform' topic
        
        # Publish pose
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = tvec[0][0]
        pose.pose.position.y = tvec[0][1]
        pose.pose.position.z = tvec[0][2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose)

        # Publish position
        position = Vector3Stamped()
        position.header = header
        position.vector.x = tvec[0][0]
        position.vector.y = tvec[0][1]
        position.vector.z = tvec[0][2]
        self.position_pub.publish(position)

        # Publish marker for visualization
        marker = Marker()
        marker.header = header
        marker.ns = "aruco_markers"
        marker.id = marker_id[0]  # Assuming marker_id is array-like
        marker.type = Marker.CUBE  # You can choose different shapes
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = self.marker_size
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque
        self.marker_pub.publish(marker)

        # Publish pixel coordinates (e.g., center of the marker)
        if corner is not None and len(corner) > 0:
            # Calculate the center of the marker in image pixels
            corner = corner.reshape((4, 2))
            center_x = np.mean(corner[:, 0])
            center_y = np.mean(corner[:, 1])
            pixel = PointStamped()
            pixel.header = header
            pixel.point.x = center_x
            pixel.point.y = center_y
            pixel.point.z = 0.0  # Assuming 2D pixel coordinates
            self.pixel_pub.publish(pixel)

def main():
    try:
        node = ArucoDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
