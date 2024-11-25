#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np

class YoloDepthDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('yolo_depth_detector', anonymous=True)

        # Parameters
        self.bridge = CvBridge()
        self.color_image_topic = rospy.get_param('~color_image_topic', '/camera/color/image_raw')
        self.depth_image_topic = rospy.get_param('~depth_image_topic', '/camera/depth/image_raw')
        self.color_info_topic = rospy.get_param('~color_info_topic', '/camera/color/camera_info')
        self.depth_info_topic = rospy.get_param('~depth_info_topic', '/camera/depth/camera_info')
        self.annotated_output_topic = rospy.get_param('~annotated_output_topic', '/detection/image_with_boxes')
        self.position_output_topic = rospy.get_param('~position_output_topic', '/detection/positions')
        self.centers_output_topic = rospy.get_param('~centers_output_topic', '/detection/centers')

        # Load YOLO model
        rospy.loginfo("Loading YOLOv5 model...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.1  # Confidence threshold
        rospy.loginfo("YOLOv5 model loaded.")

        # Publishers
        self.annotated_image_pub = rospy.Publisher(self.annotated_output_topic, Image, queue_size=1)
        self.position_pub = rospy.Publisher(self.position_output_topic, PointStamped, queue_size=10)
        self.centers_pub = rospy.Publisher(self.centers_output_topic, PointStamped, queue_size=10)

        # Camera intrinsic parameters
        self.color_K = None
        self.depth_K = None

        # Depth image
        self.depth_image = None

        # Subscribers
        rospy.Subscriber(self.color_image_topic, Image, self.image_callback)
        rospy.Subscriber(self.color_info_topic, CameraInfo, self.color_info_callback)
        rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback)
        rospy.Subscriber(self.depth_info_topic, CameraInfo, self.depth_info_callback)

        rospy.loginfo("YOLO Depth Detector initialized.")
        rospy.loginfo("Subscribed to color image: %s", self.color_image_topic)
        rospy.loginfo("Subscribed to depth image: %s", self.depth_image_topic)
        rospy.loginfo("Subscribed to color camera info: %s", self.color_info_topic)
        rospy.loginfo("Subscribed to depth camera info: %s", self.depth_info_topic)
        rospy.loginfo("Publishing annotated images to %s", self.annotated_output_topic)
        rospy.loginfo("Publishing 3D positions to %s", self.position_output_topic)
        rospy.loginfo("Publishing center points to %s", self.centers_output_topic)

    def color_info_callback(self, msg):
        if self.color_K is None:
            self.color_K = np.array(msg.K).reshape(3, 3)
            rospy.loginfo("Color camera intrinsic parameters received.")
            rospy.loginfo("K_color: \n%s", self.color_K)

    def depth_info_callback(self, msg):
        if self.depth_K is None:
            self.depth_K = np.array(msg.K).reshape(3, 3)
            rospy.loginfo("Depth camera intrinsic parameters received.")
            rospy.loginfo("K_depth: \n%s", self.depth_K)

    def depth_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Ensure depth is in meters
            if self.depth_image.dtype != np.float32:
                # Modify this line based on your depth image units
                # For example, if depth is in millimeters:
                self.depth_image = self.depth_image.astype(np.float32) / 1000.0
        except CvBridgeError as e:
            rospy.logerr("Error converting depth image: %s", e)

    def image_callback(self, image_msg):
        if self.color_K is None or self.depth_K is None or self.depth_image is None:
            rospy.logwarn("Waiting for camera parameters and depth image...")
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Flip the image vertically (180 degrees rotation) for detection if needed
            cv_image_flipped = cv2.rotate(cv_image, cv2.ROTATE_180)
            image_for_detection = cv_image_flipped  # Use flipped image for detection

            # Perform YOLO detection on the image
            results = self.model([image_for_detection])

            # Extract detection results
            detections = results.xyxy[0]  # Bounding boxes with (x1, y1, x2, y2, conf, cls)

            # Create a copy of the original image for annotation
            annotated_image = cv_image.copy()  # Use original image for annotations

            img_height, img_width = cv_image.shape[:2]
            depth_height, depth_width = self.depth_image.shape[:2]

            # Iterate through detections and process bounding boxes
            for idx, (*box_rotated, conf, cls) in enumerate(detections):
                x1_rot, y1_rot, x2_rot, y2_rot = map(int, box_rotated)

                # Map bounding box coordinates back to original image's coordinate system
                x1 = img_width - x2_rot
                x2 = img_width - x1_rot
                y1 = img_height - y2_rot
                y2 = img_height - y1_rot

                # Ensure coordinates are within image bounds
                x1 = max(0, min(x1, img_width - 1))
                x2 = max(0, min(x2, img_width - 1))
                y1 = max(0, min(y1, img_height - 1))
                y2 = max(0, min(y2, img_height - 1))

                label = f"{self.model.names[int(cls)]}: {conf:.2f}"

                # Draw bounding box on the image
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Compute center of bounding box in image coordinates
                u_center = int((x1 + x2) / 2)
                v_center = int((y1 + y2) / 2)

                # Draw the center point on the image
                cv2.circle(annotated_image, (u_center, v_center), 5, (0, 0, 255), -1)

                # Log the bounding box details
                rospy.loginfo(f"Detected {label} with BBox [x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}]")
                rospy.loginfo(f"Center of {label}: (u: {u_center}, v: {v_center})")

                # Publish the center point in pixel coordinates
                center_msg = PointStamped()
                center_msg.header = image_msg.header  # Use the same header as the image
                center_msg.point.x = u_center
                center_msg.point.y = v_center
                center_msg.point.z = 0  # Depth will be obtained from depth image
                self.centers_pub.publish(center_msg)

                # Map pixel coordinates from color image to depth image
                u_depth, v_depth = self.map_color_to_depth(u_center, v_center, img_width, img_height, depth_width, depth_height)

                # Verify that (u_depth, v_depth) are within the depth image
                if 0 <= u_depth < depth_width and 0 <= v_depth < depth_height:
                    # Get the depth z from the depth image
                    z = self.depth_image[v_depth, u_depth]

                    if np.isnan(z) or z <= 0:
                        rospy.logwarn(f"Invalid depth at pixel ({u_depth}, {v_depth}): z={z}")
                        continue

                    rospy.loginfo(f"Depth at pixel ({u_depth}, {v_depth}): z={z:.3f} meters")

                    # Compute X, Y, Z in camera frame using depth camera intrinsics
                    X = (u_depth - self.depth_K[0, 2]) * z / self.depth_K[0, 0]
                    Y = (v_depth - self.depth_K[1, 2]) * z / self.depth_K[1, 1]
                    Z = z

                    # Create the PointStamped message with the 3D coordinates
                    point_3d = PointStamped()
                    point_3d.header = image_msg.header
                    point_3d.point.x = X
                    point_3d.point.y = Y
                    point_3d.point.z = Z

                    # Publish the 3D point
                    self.position_pub.publish(point_3d)
                    rospy.loginfo(f"Published 3D position: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} meters")
                else:
                    rospy.logwarn(f"Depth pixel out of bounds: u={u_depth}, v={v_depth}")
                    continue

            # Convert the annotated image back to ROS Image message
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                annotated_msg.header = image_msg.header  # Preserve the original header
                # Publish the annotated image
                self.annotated_image_pub.publish(annotated_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error while converting annotated image: %s", e)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error: %s", e)

    def map_color_to_depth(self, u_color, v_color, color_width, color_height, depth_width, depth_height):
        # For aligned images with same resolution
        if color_width == depth_width and color_height == depth_height:
            return u_color, v_color
        else:
            # Scale coordinates based on resolution differences
            u_depth = int(u_color * depth_width / color_width)
            v_depth = int(v_color * depth_height / color_height)
            return u_depth, v_depth

if __name__ == '__main__':
    try:
        YoloDepthDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
