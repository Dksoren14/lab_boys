from threading import Lock

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoSensorNode(Node):

    def __init__(self):
        super().__init__('claus_sim_aruco_sensor')

        self.g_pose = [0.0, 0.0, 0.0]
        self.g_orientation = 0.0

        self.lock = Lock()
        self.bridge = CvBridge()

        self.config = get_config()

        self.camera_matrix = create_camera_matrix(
            width=self.config["res_x"],
            height=self.config["res_y"],
            horizontal_fov_deg=self.config["horizontal_fov_deg"]
        )

        self.dist_coeffs = np.zeros(5, dtype=np.float32)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            self.config["aruco_dict_type"]
        )

        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.object_points = create_marker_object_points(
            self.config["marker_size_IRL"]
        )

        self.T_base_cam = create_t_base_cam(self.config)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.image_topic = "/front_realsense/image"

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Pose,
            '/aruco_marker_pose',
            10
        )

        self.last_image_received = False
        self.last_odom_received = False

        self.create_timer(2.0, self.camera_watchdog)

        self.get_logger().info("Simulation ArUco sensor node started")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info("Subscribing to odom topic: /odom")
        self.get_logger().info("Publishing marker pose to: /aruco_marker_pose")


    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        with self.lock:
            self.g_pose = [
                position.x,
                position.y,
                position.z
            ]

            self.g_orientation = yaw
            self.last_odom_received = True


    def image_callback(self, msg):
        self.last_image_received = True

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        self.detect_markers(frame)


    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            with self.lock:
                g_pose = self.g_pose.copy()
                g_orientation = self.g_orientation
                odom_ready = self.last_odom_received

            if not odom_ready:
                self.get_logger().warn("No odom received yet. Marker pose may be wrong.")

            for i in range(len(ids)):
                marker_id = int(ids[i][0])

                corner_points = corners[i][0]
                image_points = np.array(corner_points, dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    R, _ = cv2.Rodrigues(rvec)

                    T_cam_marker = np.eye(4, dtype=np.float32)
                    T_cam_marker[:3, :3] = R
                    T_cam_marker[:3, 3] = tvec.flatten()

                    T_world_marker = matrix_transformation(
                        self.T_base_cam,
                        T_cam_marker,
                        g_pose,
                        g_orientation
                    )

                    self.publish_marker_pose(T_world_marker)

                    self.get_logger().info(
                        f"Marker {marker_id}: "
                        f"world x={T_world_marker[0, 3]:.3f}, "
                        f"y={T_world_marker[1, 3]:.3f}, "
                        f"z={T_world_marker[2, 3]:.3f}"
                    )

                center_x = int(corner_points[:, 0].mean())
                center_y = int(corner_points[:, 1].mean())

                cv2.circle(
                    frame,
                    (center_x, center_y),
                    3,
                    (0, 0, 255),
                    -1
                )


        cv2.imshow("Sim ArUco Marker", frame)
        cv2.waitKey(1)


    def publish_marker_pose(self, T_world_marker):
        pose_msg = Pose()

        pose_msg.position.x = float(T_world_marker[0, 3])
        pose_msg.position.y = float(T_world_marker[1, 3])
        pose_msg.position.z = float(T_world_marker[2, 3])

        R = T_world_marker[:3, :3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        pose_msg.orientation.x = float(qx)
        pose_msg.orientation.y = float(qy)
        pose_msg.orientation.z = float(qz)
        pose_msg.orientation.w = float(qw)

        self.marker_pub.publish(pose_msg)


    def camera_watchdog(self):
        if not self.last_image_received:
            self.get_logger().warn(
                f"No image received yet. Check that the topic exists: {self.image_topic}"
            )

        if not self.last_odom_received:
            self.get_logger().warn(
                "No odom received yet. Check that /odom is publishing."
            )


def get_config():
    return {
        "res_x": 1280,
        "res_y": 720,
        "horizontal_fov_deg": 87.0,

        "aruco_dict_type": cv2.aruco.DICT_5X5_100,
        "marker_size_IRL": 0.162,
        "camera_position_base": [0.465, 0.0, 0.8765],
        "camera_pitch_down_deg": 60.0
    }


def create_camera_matrix(width, height, horizontal_fov_deg):
    horizontal_fov_rad = np.deg2rad(horizontal_fov_deg)

    fx = width / (2.0 * np.tan(horizontal_fov_rad / 2.0))
    fy = fx

    cx = width / 2.0
    cy = height / 2.0

    camera_matrix = np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,  0,  1]
    ], dtype=np.float32)

    return camera_matrix


def create_marker_object_points(marker_length):
    half_len = marker_length / 2

    object_points = np.array([
        [-half_len,  half_len, 0],
        [ half_len,  half_len, 0],
        [ half_len, -half_len, 0],
        [-half_len, -half_len, 0]
    ], dtype=np.float32)

    return object_points


def create_t_base_cam(config):
    camera_position = np.array(
        config["camera_position_base"],
        dtype=np.float32
    )

    pitch_down = np.deg2rad(config["camera_pitch_down_deg"])

    c = np.cos(pitch_down)
    s = np.sin(pitch_down)

    R_base_cam = np.array([
        [0, -s,  c],
        [-1, 0,  0],
        [0, -c, -s]
    ], dtype=np.float32)

    T_base_cam = np.eye(4, dtype=np.float32)
    T_base_cam[:3, :3] = R_base_cam
    T_base_cam[:3, 3] = camera_position

    return T_base_cam


def matrix_transformation(T_base_cam, T_cam_marker, g_pose, g_orientation):
    R_world_base = np.array([
        [np.cos(g_orientation), -np.sin(g_orientation), 0],
        [np.sin(g_orientation),  np.cos(g_orientation), 0],
        [0, 0, 1]
    ], dtype=np.float32)

    T_world_base = np.eye(4, dtype=np.float32)
    T_world_base[:3, :3] = R_world_base
    T_world_base[:3, 3] = g_pose

    T_world_marker = T_world_base @ T_base_cam @ T_cam_marker

    return T_world_marker


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return np.arctan2(siny_cosp, cosy_cosp)


def rotation_matrix_to_quaternion(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(
            1.0 + R[0, 0] - R[1, 1] - R[2, 2]
        )
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(
            1.0 + R[1, 1] - R[0, 0] - R[2, 2]
        )
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:
        s = 2.0 * np.sqrt(
            1.0 + R[2, 2] - R[0, 0] - R[1, 1]
        )
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)

    node = ArucoSensorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()