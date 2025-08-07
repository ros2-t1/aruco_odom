#!/usr/bin/env python3
import numpy as np
import cv2
import math
from typing import Tuple

# ---- ROS2 ----
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as SciR

# =========================
# 유사변환 & 보정 파라미터
# =========================
SIMILARITY_S = 1.1731463819258798
SIMILARITY_R = np.array([[ 0.02926172, -0.99957178],
                         [-0.99957178, -0.02926172]])
SIMILARITY_T = np.array([-0.00178071,  0.01315473])
THETA_RAD    = math.radians(-88.32318748498734)
DET_R_SIGN   = -1

# =========================
# 카메라 파라미터 (1920x1080)
# =========================
CAMERA_MATRIX = np.array([
    [1185.96684, 0,        999.31995],
    [0,          890.7003, 569.28861],
    [0,          0,        1]
], dtype=np.float32)

DIST_COEFFS = np.array([
    -9.413361e-02, -8.374589e-04, 3.176887e-04, -3.987077e-04, 3.289896e-03,
    0.0, 0.0, 0.0
], dtype=np.float32)

MARKER_LENGTH = 0.1
AXIS_LENGTH   = 0.05

# =========================
# 유틸
# =========================
def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def convert_position(ax: float, ay: float) -> Tuple[float, float]:
    p = np.array([ax, ay])
    mp = (SIMILARITY_R @ p) * SIMILARITY_S + SIMILARITY_T
    return float(mp[0]), float(mp[1])

def convert_yaw(yaw_cam_rad: float) -> float:
    if DET_R_SIGN >= 0:
        return wrap(yaw_cam_rad + THETA_RAD)
    else:
        return wrap(-yaw_cam_rad + THETA_RAD)

def calculate_pose_from_corners(corners, camera_matrix, dist_coeffs, marker_length):
    pts = corners.reshape((4, 2)).astype(np.float32)
    center_2d = np.mean(pts, axis=0)
    direction_vec = pts[1] - pts[0]
    yaw_image = math.atan2(direction_vec[1], direction_vec[0])

    half_size = marker_length / 2.0
    object_points = np.array([
        [-half_size, -half_size, 0],
        [ half_size, -half_size, 0],
        [ half_size,  half_size, 0],
        [-half_size,  half_size, 0]
    ], dtype=np.float32)

    success, rvec, tvec = cv2.solvePnP(object_points, pts, camera_matrix, dist_coeffs)
    if not success:
        return None, None, None

    x_cam = float(tvec[0][0])
    y_cam = float(tvec[1][0])
    return x_cam, y_cam, yaw_image

# =========================
# ArUco 검출 클래스
# =========================
class ArucoDetector:
    def __init__(self, camera_matrix, dist_coeffs, marker_length=0.051, axis_length=0.05):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length
        self.axis_length = axis_length

    def detect_and_annotate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        corners, ids, rejected = self.detector.detectMarkers(binary)
        detected_markers = []
        pose_data = None

        if ids is not None and len(ids) > 0:
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                pts = corner.reshape((4, 2)).astype(int)
                top_left = (int(pts[0][0]), int(pts[0][1]))

                cv2.putText(frame, f"ID: {marker_id}", top_left,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                x_cam, y_cam, yaw_img = calculate_pose_from_corners(
                    corner, self.camera_matrix, self.dist_coeffs, self.marker_length
                )

                if x_cam is not None:
                    distance = math.sqrt(x_cam**2 + y_cam**2)
                    cv2.putText(frame, f"X:{x_cam:.2f} Y:{y_cam:.2f}",
                                (cx + 10, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(frame, f"D:{distance:.2f} Yaw:{math.degrees(yaw_img):.1f}°",
                                (cx + 10, cy + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    direction_end = (int(cx + 50 * math.cos(yaw_img)),
                                     int(cy + 50 * math.sin(yaw_img)))
                    cv2.arrowedLine(frame, (cx, cy), direction_end, (0, 255, 255), 3)

                    if pose_data is None:
                        pose_data = (x_cam, y_cam, yaw_img)

                detected_markers.append({
                    "id": marker_id,
                    "center_2d": (cx, cy),
                    "position_3d": (x_cam, y_cam, 0.0) if x_cam is not None else None,
                    "yaw_image": yaw_img if yaw_img is not None else None
                })

        return frame, detected_markers, pose_data

# =========================
# ROS2 노드: /odom 발행
# =========================
class ArucoToOdomNode(Node):
    def __init__(self, cam_index=2):
        super().__init__('aruco_to_odom')

        # --- 카메라 ---
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {cam_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera FPS: {fps}")

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera opened at {w}x{h}")

        self.detector = ArucoDetector(CAMERA_MATRIX, DIST_COEFFS, MARKER_LENGTH, AXIS_LENGTH)
        self.odom_pub = self.create_publisher(Odometry, '/odom_aruco', qos_profile_sensor_data)

        self.prev_stamp = None
        self.prev_pose  = None

        self.timer = self.create_timer(0.02, self.loop)

    def loop(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera frame read failed")
            return

        annotated, markers, pose_data = self.detector.detect_and_annotate(frame)
        stamp = self.get_clock().now().to_msg()

        if pose_data is None or len(markers) == 0:
            cv2.imshow("Aruco", cv2.resize(annotated, (960, 540)))
            cv2.waitKey(1)
            return

        x_cam, y_cam, yaw_img = pose_data
        x, y = convert_position(x_cam, y_cam)
        yaw = convert_yaw(yaw_img)

        self._publish_odom(stamp, x, y, yaw)
        cv2.imshow("Aruco", cv2.resize(annotated, (960, 540)))
        cv2.waitKey(1)

    def _publish_odom(self, stamp, x, y, yaw):
        q = SciR.from_euler('z', yaw).as_quat()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        if self.prev_stamp is not None and self.prev_pose is not None:
            dt = (stamp.sec + stamp.nanosec*1e-9) - (self.prev_stamp.sec + self.prev_stamp.nanosec*1e-9)
            if dt > 1e-4:
                vx = (x - self.prev_pose[0]) / dt
                vy = (y - self.prev_pose[1]) / dt
                wyaw = wrap(yaw - self.prev_pose[2]) / dt
                odom.twist.twist.linear.x  = float(vx)
                odom.twist.twist.linear.y  = float(vy)
                odom.twist.twist.angular.z = float(wyaw)

        odom.pose.covariance[0]  = 0.2
        odom.pose.covariance[7]  = 0.2
        odom.pose.covariance[35] = 0.5
        odom.twist.covariance[0] = 0.04
        odom.twist.covariance[7] = 0.04
        odom.twist.covariance[35]= 0.05

        self.odom_pub.publish(odom)

        self.prev_stamp = stamp
        self.prev_pose  = (x, y, yaw)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

# =========================
# main
# =========================
def main():
    rclpy.init()
    node = ArucoToOdomNode(cam_index=2)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
