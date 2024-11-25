#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
from geometry_msgs.msg import PointStamped

class YoloDetector:
    def __init__(self):
        # Initialize ROS node parameters
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/color/camera_info')
        self.annotated_output_topic = rospy.get_param('~annotated_output_topic', '/detection/image_with_boxes')
        self.centers_output_topic = rospy.get_param('~centers_output_topic', '/detection/centers')

        # Load the YOLO model from Ultralytics
        rospy.loginfo("Loading YOLOv5 model...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.5  # Confidence threshold
        rospy.loginfo("YOLOv5 model loaded.")

        # Publishers
        self.annotated_image_pub = rospy.Publisher(self.annotated_output_topic, Image, queue_size=1)
        self.centers_pub = rospy.Publisher(self.centers_output_topic, PointStamped, queue_size=10)

        # Camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        # Subscriber for camera info
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        # Subscriber to the input image topic
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        rospy.loginfo("YOLO Detector initialized.")
        rospy.loginfo("Subscribed to color image: %s", self.image_topic)
        rospy.loginfo("Subscribed to camera info: %s", self.camera_info_topic)
        rospy.loginfo("Publishing annotated images to %s", self.annotated_output_topic)
        rospy.loginfo("Publishing center points to %s", self.centers_output_topic)

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            rospy.loginfo("Camera intrinsic parameters received:")
            rospy.loginfo("fx: %f, fy: %f, cx: %f, cy: %f", self.fx, self.fy, self.cx, self.cy)
            self.camera_info_received = True
            # Optionally, unsubscribe from camera info after receiving it
            # rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback).unregister()

    def image_callback(self, image_msg):
        if not self.camera_info_received:
            rospy.logwarn("Camera intrinsic parameters not yet received. Skipping frame.")
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Flip the image vertically (180 degrees rotation) for detection
            cv_image_flipped = cv2.rotate(cv_image, cv2.ROTATE_180)

            # Perform YOLO detection on the flipped color image
            results = self.model([cv_image_flipped])

            # Extract detection results
            detections = results.xyxy[0]  # Bounding boxes with (x1, y1, x2, y2, conf, cls)

            # Create a copy of the original (unflipped) image for annotation
            annotated_image = cv_image.copy()

            # Iterate through detections and process bounding boxes
            for idx, (*box_rotated, conf, cls) in enumerate(detections):
                x1_rot, y1_rot, x2_rot, y2_rot = map(int, box_rotated)

                img_height, img_width = cv_image.shape[:2]

                # Map bounding box coordinates back to original image's coordinate system
                x1_orig = img_width - x2_rot
                x2_orig = img_width - x1_rot
                y1_orig = img_height - y2_rot
                y2_orig = img_height - y1_rot

                # Ensure coordinates are within image bounds
                x1_orig = max(0, min(x1_orig, img_width - 1))
                x2_orig = max(0, min(x2_orig, img_width - 1))
                y1_orig = max(0, min(y1_orig, img_height - 1))
                y2_orig = max(0, min(y2_orig, img_height - 1))

                label = f"{self.model.names[int(cls)]}: {conf:.2f}"

                # Draw bounding box on the original (unflipped) image
                cv2.rectangle(annotated_image, (x1_orig, y1_orig), (x2_orig, y2_orig), (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x1_orig, y1_orig - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Compute center of bounding box in original image's coordinates
                u_center = int((x1_orig + x2_orig) / 2)
                v_center = int((y1_orig + y2_orig) / 2)

                # Draw the center point on the image
                cv2.circle(annotated_image, (u_center, v_center), 5, (0, 0, 255), -1)

                # Log the bounding box details
                rospy.loginfo(f"Detected {label} with BBox [x1: {x1_orig}, y1: {y1_orig}, x2: {x2_orig}, y2: {y2_orig}]")
                rospy.loginfo(f"Center of {label}: (u: {u_center}, v: {v_center})")

                # Map pixel coordinates to camera frame coordinates
                x_cam = (u_center - self.cx) / self.fx
                y_cam = (v_center - self.cy) / self.fy
                z_cam = 1.0  # Normalized depth since actual depth is unknown

                # Create PointStamped message for the center
                point_msg = PointStamped()
                point_msg.header = image_msg.header  # Use the same header as the image
                point_msg.point.x = x_cam
                point_msg.point.y = y_cam
                point_msg.point.z = z_cam

                # Publish the center point
                self.centers_pub.publish(point_msg)

                # Log the camera frame coordinates
                rospy.loginfo(f"Center of {label} in camera frame: x={x_cam:.2f}, y={y_cam:.2f}, z={z_cam:.2f}")

            # Convert the annotated original OpenCV image back to ROS Image message
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
            rospy.logerr("Error processing image: %s", e)

def main():
    rospy.init_node('yolo_detector')
    YoloDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
