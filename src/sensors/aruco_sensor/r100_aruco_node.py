from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from interfaces.msg import ArucoPose
from interfaces.msg import BaseState
import cv2
import pyrealsense2 as rs
import numpy as np


class ArucoSenorNode(Node):

    def __init__(self):
        super().__init__('claus_aruco_sensor')
        self.g_pose = [0.0, 0.0, 0.0]
        self.g_orientation = 0.0

        self.lock = Lock()

        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
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

        self.marker_pub = self.create_publisher(ArucoPose, '/aruco_marker_pose', 10)
        


    def global_callback(self, msg):
        with self.lock:
            self.g_pose = [msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z]

    def orientation_callback(self, msg):
        with self.lock:
            self.g_orientation = msg.orientation[2]

    def publish_marker_pose(self, T_world_marker, marker_id):
        pose_msg = ArucoPose()
    
        #pose_msg.position.x = float(T_world_marker[0, 3])
        #pose_msg.position.y = float(T_world_marker[1, 3])
        #pose_msg.position.z = float(T_world_marker[2, 3])

        pose_msg.pose = [
            float(T_world_marker[0, 3]),
            float(T_world_marker[1, 3]),
            float(T_world_marker[2, 3])
        ]

        R = T_world_marker[:3, :3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        #pose_msg.orientation.x = float(qx)
        #pose_msg.orientation.y = float(qy)
        #pose_msg.orientation.z = float(qz)
        #pose_msg.orientation.w = float(qw)

        pose_msg.orientation = [float(qx), float(qy), float(qz), float(qw)]


        pose_msg.id = marker_id

        self.marker_pub.publish(pose_msg)

       
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

def detect_markers(config, node):
    aruco_dict = cv2.aruco.getPredefinedDictionary(config["aruco_dict_type"])
    detector_params = cv2.aruco.DetectorParameters_create()

    pipe = rs.pipeline()
    cfg = rs.config()

    cfg.enable_stream(rs.stream.color, config["res_x"], config["res_y"], rs.format.bgr8, config["fps"])

    pipe.start(cfg)

    profile = pipe.get_active_profile()
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = color_profile.get_intrinsics()
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ])
    dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)

    marker_length = config["marker_size_IRL"]
    half_len = (marker_length / 2)
    object_points = np.array([
        [-half_len, half_len, 0],
        [half_len, half_len, 0],
        [half_len, -half_len, 0],
        [-half_len, -half_len, 0]
    ], dtype=np.float32)
    T_base_cam = create_t_base_cam(config)

    try:
        while True:
            frames = pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=detector_params)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners)

                with node.lock:
                    g_pose = node.g_pose
                    g_orientation = node.g_orientation

    
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])

                    corner_points = corners[i][0]
                    image_points = np.array(corner_points, dtype=np.float32)

                    success, rvec, tvec = cv2.solvePnP(
                        object_points,
                        image_points,
                        camera_matrix,
                        dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:
                        R, _ = cv2.Rodrigues(rvec)

                        T_cam_marker = np.eye(4, dtype=np.float32)
                        T_cam_marker[:3, :3] = R
                        T_cam_marker[:3, 3] = tvec.flatten()
                        T_world_marker = matrix_transformation(T_base_cam, T_cam_marker, g_pose, g_orientation)
                        node.publish_marker_pose(T_world_marker, marker_id)
                    center_x = int(corner_points[:, 0].mean())
                    center_y = int(corner_points[:, 1].mean())

                    cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)

            cv2.imshow("ArUco Marker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipe.stop()
        cv2.destroyAllWindows()

def create_t_base_cam(config):
    camera_position = np.array(config["camera_position_base"], dtype=np.float32)
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

def rotation_matrix_to_quaternion(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return qx, qy, qz, qw

def start_ros(node):
        try:
            rclpy.spin(node)

        except KeyboardInterrupt:
            node.get_logger().info('Node interrupted by user')
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSenorNode()
    Thread(target=start_ros, args=(node,), daemon=True).start()
    config = get_config()
    detect_markers(config, node)


if __name__ == "__main__":
    main()