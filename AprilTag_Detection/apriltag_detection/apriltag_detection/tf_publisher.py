import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import apriltag
import cv2
import numpy as np
import tf2_ros
import tf_transformations

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')
        self.bridge = CvBridge()

        # Initialize AprilTag detector with correct family
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

        # Camera parameters
        self.tagsize = 0.173  # Tag size in meters
        self.f_mm = 3.67  # Focal length in mm
        self.sensor_width_mm = 3.68  # Assuming sensor width is around 3.68 mm
        self.image_width_px = 160  # Resized image width
        self.image_height_px = 120  # Resized image height
        self.fx = (self.f_mm / self.sensor_width_mm) * self.image_width_px
        self.fy = self.fx
        self.cx = self.image_width_px // 2
        self.cy = self.image_height_px // 2

        # ROS subscribers and TF broadcaster
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the frame
        detections = self.detector.detect(gray)

        for detection in detections:
            tag_id = detection.tag_id
            corners = detection.corners  # Get corners directly from detection object
            object_points = np.array([
                [-self.tagsize / 2, -self.tagsize / 2, 0],
                [ self.tagsize / 2, -self.tagsize / 2, 0],
                [ self.tagsize / 2,  self.tagsize / 2, 0],
                [-self.tagsize / 2,  self.tagsize / 2, 0]
            ])
            image_points = np.array(corners, dtype='double')

            camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            dist_coeffs = np.zeros(4)  # Assuming no distortion

            # Solve PnP to get rotation and translation vectors
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            if retval:
                # Convert rotation vector to a quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rotation_matrix
                quaternion = tf_transformations.quaternion_from_matrix(transform_matrix)

                # Publish TF transform
                apriltag_to_camera_tf = TransformStamped()
                apriltag_to_camera_tf.header.stamp = self.get_clock().now().to_msg()
                apriltag_to_camera_tf.header.frame_id = 'camera_frame'  # Parent frame
                apriltag_to_camera_tf.child_frame_id = f'apriltag_{tag_id}'  # Child frame based on tag ID

                # Set translation (tvec)
                apriltag_to_camera_tf.transform.translation.x = tvec[0][0]
                apriltag_to_camera_tf.transform.translation.y = tvec[1][0]
                apriltag_to_camera_tf.transform.translation.z = tvec[2][0]

                # Set rotation (converted quaternion from rvec)
                apriltag_to_camera_tf.transform.rotation.x = quaternion[0]
                apriltag_to_camera_tf.transform.rotation.y = quaternion[1]
                apriltag_to_camera_tf.transform.rotation.z = quaternion[2]
                apriltag_to_camera_tf.transform.rotation.w = quaternion[3]

                # Send the transform
                self.tf_broadcaster.sendTransform(apriltag_to_camera_tf)

def main(args=None):
    rclpy.init(args=args)

    april_tag_detector = AprilTagDetector()

    rclpy.spin(april_tag_detector)

    april_tag_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

