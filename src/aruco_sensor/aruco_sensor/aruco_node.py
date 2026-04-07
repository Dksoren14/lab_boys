#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import cv2
import pyrealsense2 as rs
import numpy as np

class ArucoSenorNode(Node):

    def __init__(self):
        super().__init__('claus_aruco_sensor')
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


    def global_callback(self, msg):
        g_pose =[msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z]






def get_config():
    return {
        "res_x": 1280,
        "res_y": 720,
        "fps": 30,
        "aruco_dict_type": cv2.aruco.DICT_5X5_100,
        "marker_ids": [20, 21, 22, 23, 24, 25, 26, 27, 28],
        "marker_size": 800,
        "marker_size_IRL": 0.162
    }

def detect_markers(config):
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

    print("Object points:")
    print(object_points)
    try:
        while True:
            frames = pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=detector_params)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners)

                for i in range(len(ids)):
                    marker_id = ids[i][0]
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

                        print("Transformation matrix:")
                        print(T_cam_marker)




                    center_x = int(corner_points[:, 0].mean())
                    center_y = int(corner_points[:, 1].mean())



                    cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
                    coo = f"({center_x}x, {center_y}y)"
                    Marker_ID = f"ID {marker_id}"

                    cv2.putText(frame, coo,(center_x + 10, center_y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame, Marker_ID, (center_x + 10, center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("ArUco Marker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    config = get_config()
    detect_markers(config)