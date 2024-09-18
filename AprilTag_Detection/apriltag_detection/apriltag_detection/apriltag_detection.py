import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import apriltag

class WebcamAndAprilTagNode(Node):
    def __init__(self):
        super().__init__('webcam_and_april_tag_node')
        self.bridge = CvBridge()

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)  # 0 is the ID of the default webcam

        # Check if the webcam is opened correctly
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            exit()

        # Initialize AprilTag detector
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))  # Use Detector from apriltag
        
        # Camera parameters (modify focal length based on your camera)
        self.tagsize = 0.173  # Tag size in meters
        self.f_mm = 3.67  # Focal length in mm
        self.sensor_width_mm = 3.68  # Assuming sensor width is around 3.68 mm
        self.image_width_px = 640  # Resized image width
        self.image_height_px = 480  # Resized image height
        self.fx = (self.f_mm / self.sensor_width_mm) * self.image_width_px
        self.fy = self.fx
        self.cx = self.image_width_px // 2
        self.cy = self.image_height_px // 2

        # ROS publishers
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'april_tag_pose', 10)

        # Timer to regularly publish images
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            # Resize the frame to 640x480 pixels
            frame_resized = cv2.resize(frame, (640, 480))

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags in the frame
            detections = self.detector.detect(gray)

            for detection in detections:
                corners = detection.corners  # Get corners of the tag
                corners = corners.astype(int)
                for i in range(4):
                    start_point = tuple(corners[i])
                    end_point = tuple(corners[(i + 1) % 4])
                    cv2.line(frame_resized, start_point, end_point, (0, 255, 0), 2)
                
                # Draw the ID of the AprilTag
                tag_id = str(detection.tag_id)
                center_x = int((corners[0][0] + corners[2][0]) / 2)
                center_y = int((corners[0][1] + corners[2][1]) / 2)
                cv2.putText(frame_resized, tag_id, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Show the video with detections
            cv2.imshow('Webcam', frame_resized)
            cv2.waitKey(1)

            # Convert the resized frame to a ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(frame_resized, encoding='bgr8')

            # Set the timestamp and frame ID
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'

            # Publish the image
            self.image_publisher.publish(ros_image)
        else:
            self.get_logger().error("Error: Could not read frame from webcam.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the frame
        detections = self.detector.detect(gray)

        for detection in detections:
            corners = detection.corners  # Get the tag's corners
            object_points = np.array([
                [-self.tagsize / 2, -self.tagsize / 2, 0],
                [self.tagsize / 2, -self.tagsize / 2, 0],
                [self.tagsize / 2, self.tagsize / 2, 0],
                [-self.tagsize / 2, self.tagsize / 2, 0]
            ])
            image_points = np.array(corners)

            camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            dist_coeffs = np.zeros(4)

            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            if retval:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = f'tag_ID_{detection.tag_id}'  # Tag ID as frame_id
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # Setting orientation (just a rotation vector for now)
                pose_msg.pose.orientation.x = rvec[0][0]
                pose_msg.pose.orientation.y = rvec[1][0]
                pose_msg.pose.orientation.z = rvec[2][0]
                pose_msg.pose.orientation.w = 1.0  # Set w manually since solvePnP only gives rvec

                self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    webcam_and_april_tag_node = WebcamAndAprilTagNode()

    rclpy.spin(webcam_and_april_tag_node)

    webcam_and_april_tag_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


