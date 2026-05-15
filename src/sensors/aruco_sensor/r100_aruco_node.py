from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    qos_profile_sensor_data
)

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, CameraInfo

from interfaces.msg import ArucoPose
from interfaces.msg import BaseState

from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoSensorNode(Node):

    def __init__(self):
        super().__init__('claus_aruco_sensor')

        self.g_pose = [0.0, 0.0, 0.0]
        self.g_orientation = 0.0

        self.lock = Lock()
        self.bridge = CvBridge()

        self.config = get_config()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            self.config["aruco_dict_type"]
        )

        self.detector_params = cv2.aruco.DetectorParameters_create()

        marker_length = self.config["marker_size_IRL"]
        half_len = marker_length / 2.0

        self.object_points = np.array([
            [-half_len,  half_len, 0],
            [ half_len,  half_len, 0],
            [ half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)

        self.t_base_cam = create_t_base_cam(self.config)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.base_global_position_sub = self.create_subscription(
            PoseArray,
            '/world/car_world/dynamic_pose/info',
            self.global_callback,
            qos
        )

        self.base_global_orientation_sub = self.create_subscription(
            BaseState,
            '/lab_boys/out/base_state',
            self.orientation_callback,
            qos
        )

        self.image_sub = self.create_subscription(
            Image,
            '/front/realsense/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/front/realsense/color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.marker_pub = self.create_publisher(
            ArucoPose,
            '/aruco_marker_pose',
            10
        )

        self.show_debug_window = False

        self.get_logger().info('Aruco sensor node started')
        self.get_logger().info('Waiting for image and camera calibration topics')

    def global_callback(self, msg):
        if len(msg.poses) == 0:
            return

        with self.lock:
            self.g_pose = [
                msg.poses[0].position.x,
                msg.poses[0].position.y,
                msg.poses[0].position.z
            ]

    def orientation_callback(self, msg):
        with self.lock:
            self.g_orientation = msg.orientation[2]

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ], dtype=np.float32)

        self.dist_coeffs = np.array(msg.d, dtype=np.float32)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn(
                'Waiting for camera calibration...',
                throttle_duration_sec=2.0
            )
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'Could not convert ROS image to OpenCV frame: {error}'
            )
            return

        self.detect_markers(frame)

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params
        )

        if ids is None:
            return

        with self.lock:
            g_pose = self.g_pose.copy()
            g_orientation = self.g_orientation

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

            if not success:
                continue

            rotation_matrix, _ = cv2.Rodrigues(rvec)

            t_cam_marker = np.eye(4, dtype=np.float32)
            t_cam_marker[:3, :3] = rotation_matrix
            t_cam_marker[:3, 3] = tvec.flatten()

            t_world_marker = matrix_transformation(
                self.t_base_cam,
                t_cam_marker,
                g_pose,
                g_orientation
            )

            self.publish_marker_pose(t_world_marker, marker_id)

            if self.show_debug_window:
                center_x = int(corner_points[:, 0].mean())
                center_y = int(corner_points[:, 1].mean())
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)

        if self.show_debug_window:
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.imshow('ArUco Marker', frame)
            cv2.waitKey(1)

    def publish_marker_pose(self, t_world_marker, marker_id):
        pose_msg = ArucoPose()

        pose_msg.pose = [
            float(t_world_marker[0, 3]),
            float(t_world_marker[1, 3]),
            float(t_world_marker[2, 3])
        ]

        rotation_matrix = t_world_marker[:3, :3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)

        pose_msg.orientation = [
            float(qx),
            float(qy),
            float(qz),
            float(qw)
        ]

        pose_msg.id = int(marker_id)

        self.marker_pub.publish(pose_msg)

        self.get_logger().info(
            f'Marker {marker_id}: '
            f'x={pose_msg.pose[0]:.3f}, '
            f'y={pose_msg.pose[1]:.3f}, '
            f'z={pose_msg.pose[2]:.3f}'
        )


def get_config():
    return {
        "res_x": 1280,
        "res_y": 720,
        "fps": 30,
        "aruco_dict_type": cv2.aruco.DICT_5X5_100,
        "marker_size": 800,
        "marker_size_IRL": 0.162,
        "camera_position_base": [0.465, 0.0, 0.8765],
        "camera_pitch_down_deg": 60.0
    }


def create_t_base_cam(config):
    camera_position = np.array(
        config["camera_position_base"],
        dtype=np.float32
    )

    pitch_down = np.deg2rad(config["camera_pitch_down_deg"])

    c = np.cos(pitch_down)
    s = np.sin(pitch_down)

    r_base_cam = np.array([
        [0, -s,  c],
        [-1, 0,  0],
        [0, -c, -s]
    ], dtype=np.float32)

    t_base_cam = np.eye(4, dtype=np.float32)
    t_base_cam[:3, :3] = r_base_cam
    t_base_cam[:3, 3] = camera_position

    return t_base_cam


def matrix_transformation(t_base_cam, t_cam_marker, g_pose, g_orientation):
    r_world_base = np.array([
        [np.cos(g_orientation), -np.sin(g_orientation), 0],
        [np.sin(g_orientation),  np.cos(g_orientation), 0],
        [0, 0, 1]
    ], dtype=np.float32)

    t_world_base = np.eye(4, dtype=np.float32)
    t_world_base[:3, :3] = r_world_base
    t_world_base[:3, 3] = g_pose

    t_world_marker = t_world_base @ t_base_cam @ t_cam_marker

    return t_world_marker


def rotation_matrix_to_quaternion(rotation_matrix):
    trace = (
        rotation_matrix[0, 0]
        + rotation_matrix[1, 1]
        + rotation_matrix[2, 2]
    )

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s

    elif (
        rotation_matrix[0, 0] > rotation_matrix[1, 1]
        and rotation_matrix[0, 0] > rotation_matrix[2, 2]
    ):
        s = 2.0 * np.sqrt(
            1.0
            + rotation_matrix[0, 0]
            - rotation_matrix[1, 1]
            - rotation_matrix[2, 2]
        )

        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s

    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = 2.0 * np.sqrt(
            1.0
            + rotation_matrix[1, 1]
            - rotation_matrix[0, 0]
            - rotation_matrix[2, 2]
        )

        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s

    else:
        s = 2.0 * np.sqrt(
            1.0
            + rotation_matrix[2, 2]
            - rotation_matrix[0, 0]
            - rotation_matrix[1, 1]
        )

        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
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