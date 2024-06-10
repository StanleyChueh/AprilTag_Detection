import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import transforms3d.quaternions as quat
import transforms3d.affines as aff
import numpy as np

class PoseUpdaterNode(Node):
    def __init__(self):
        super().__init__('pose_updater_node')
        
        self.april_tag_pose = None
        self.camera_pose = None
        
        self.april_tag_pose_updated = False
        self.camera_pose_updated = False

        self.april_tag_pose_subscriber = self.create_subscription(
            PoseStamped, 'april_tag_pose', self.april_tag_pose_callback, 10)

        self.camera_pose_subscriber = self.create_subscription(
            PoseStamped, 'camera_pose', self.camera_pose_callback, 10)

        self.amcl_pose_update_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose_update_pose', 10)

        self.timer = self.create_timer(0.1, self.update_amcl_pose)

    def april_tag_pose_callback(self, msg):
        self.get_logger().info('Received april_tag_pose')
        self.april_tag_pose = msg
        self.april_tag_pose_updated = True

    def camera_pose_callback(self, msg):
        self.get_logger().info('Received camera_pose')
        self.camera_pose = msg
        self.camera_pose_updated = True

    def update_amcl_pose(self):
        if not (self.april_tag_pose_updated and self.camera_pose_updated):
            return
        
        # Reset update flags
        self.april_tag_pose_updated = False
        self.camera_pose_updated = False

        # Convert camera pose in AprilTag frame to transformation matrix
        camera_pose_mat = self.pose_to_matrix(self.camera_pose.pose)
        
        # Invert the camera pose to get the AprilTag pose in the camera frame
        camera_pose_mat_inv = np.linalg.inv(camera_pose_mat)
        
        # Convert AprilTag pose in camera frame to transformation matrix
        april_tag_pose_mat = self.pose_to_matrix(self.april_tag_pose.pose)

        # Combine transformations
        relative_transform = np.dot(april_tag_pose_mat, camera_pose_mat_inv)

        # Convert relative transform to pose
        relative_pose = self.matrix_to_pose(relative_transform)

        # Create a new PoseWithCovarianceStamped message
        updated_amcl_pose = PoseWithCovarianceStamped()
        updated_amcl_pose.header.stamp = self.get_clock().now().to_msg()
        updated_amcl_pose.header.frame_id = 'map'
        updated_amcl_pose.pose.pose = relative_pose
        updated_amcl_pose.pose.covariance = [0.0] * 36  # Set covariance to zero or appropriate values

        # Publish the updated pose
        self.get_logger().info('Publishing amcl_pose_update_pose')
        self.amcl_pose_update_publisher.publish(updated_amcl_pose)

    def pose_to_matrix(self, pose):
        q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        t = [pose.position.x, pose.position.y, pose.position.z]
        transform = aff.compose(t, quat.quat2mat(q), [1, 1, 1])
        return transform

    def matrix_to_pose(self, mat):
        t, R, _, _ = aff.decompose(mat)
        q = quat.mat2quat(R)
        pose = PoseStamped().pose
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation.w = q[0]
        pose.orientation.x = q[1]
        pose.orientation.y = q[2]
        pose.orientation.z = q[3]
        return pose

def main(args=None):
    rclpy.init(args=args)
    pose_updater_node = PoseUpdaterNode()
    rclpy.spin(pose_updater_node)
    pose_updater_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
