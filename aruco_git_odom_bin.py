# #!/usr/bin/env python3
# import numpy as np
# import cv2
# import math
# from typing import Tuple

# # ---- ROS2 ----
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data
# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# from scipy.spatial.transform import Rotation as SciR

# # =========================
# # 유사변환 & 보정 파라미터
# #  - 필요 시 값만 교체해서 사용
# # =========================
# SIMILARITY_S = 1.1731463819258798
# SIMILARITY_R = np.array([[ 0.02926172, -0.99957178],
#                          [-0.99957178, -0.02926172]])
# SIMILARITY_T = np.array([-0.00178071,  0.01315473])
# THETA_RAD    = math.radians(-88.32318748498734)
# DET_R_SIGN   = -1   # -1: 미러 반전 포함, +1: 미러 없음

# # 카메라 tvec의 어떤 두 축을 평면 (ax, ay)로 사용할지
# # 보통 카메라 tvec: x=우, y=아래, z=앞.  (x,y)를 쓰면 (0,1), (x,z)를 쓰면 (0,2)
# ARUCO_XY_AXES = (0, 1)

# # =========================
# # 카메라 파라미터 (1920x1080)
# # =========================
# CAMERA_MATRIX = np.array([
#     [1185.96684, 0,        999.31995],
#     [0,          890.7003, 569.28861],
#     [0,          0,        1]
# ], dtype=np.float32)

# # plumb_bob(k1,k2,p1,p2,k3,k4,k5,k6)
# DIST_COEFFS = np.array([
#     -9.413361e-02, -8.374589e-04, 3.176887e-04, -3.987077e-04, 3.289896e-03,
#     0.0, 0.0, 0.0
# ], dtype=np.float32)

# MARKER_LENGTH = 0.051   # 51 mm
# AXIS_LENGTH   = 0.05

# # =========================
# # 유틸
# # =========================
# def wrap(a: float) -> float:
#     return (a + math.pi) % (2 * math.pi) - math.pi

# def euler_from_quat_xyzw(q: np.ndarray) -> Tuple[float, float, float]:
#     # q = [x, y, z, w]
#     r = SciR.from_quat(q)
#     roll, pitch, yaw = r.as_euler('xyz', degrees=False)
#     return roll, pitch, yaw

# def normalize_quaternion(q: np.ndarray) -> np.ndarray:
#     n = np.linalg.norm(q)
#     if n <= 1e-12:
#         return np.array([0,0,0,1.0], dtype=float)
#     return q / n

# def convert_position(ax: float, ay: float) -> Tuple[float, float]:
#     """Aruco 평면 (ax,ay) -> odom/map (mx,my)"""
#     p = np.array([ax, ay])
#     mp = (SIMILARITY_R @ p) * SIMILARITY_S + SIMILARITY_T
#     return float(mp[0]), float(mp[1])

# def convert_yaw(yaw_cam_rad: float) -> float:
#     """카메라 기준 yaw -> odom/map 기준 yaw"""
#     if DET_R_SIGN >= 0:
#         return wrap(yaw_cam_rad + THETA_RAD)
#     else:
#         return wrap(-yaw_cam_rad + THETA_RAD)

# # =========================
# # ArUco 검출 클래스 (원본 + rvecs/tvecs 반환 추가)
# # =========================
# class ArucoDetector:
#     def __init__(self, camera_matrix, dist_coeffs, marker_length=0.051, axis_length=0.05):
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
#         self.aruco_params = cv2.aruco.DetectorParameters()
#         self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
#         self.camera_matrix = camera_matrix
#         self.dist_coeffs = dist_coeffs
#         self.marker_length = marker_length
#         self.axis_length = axis_length

#     def detect_and_annotate(self, frame):
#         # 1) 흑백 + Otsu thresholding (binary 처리)
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

#         # 2) ArUco 마커 검출 (binary 이미지 사용)
#         corners, ids, rejected = self.detector.detectMarkers(binary)
#         detected_markers = []
#         rvecs = None
#         tvecs = None

#         if ids is not None and len(ids) > 0:
#             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#                 corners, self.marker_length, self.camera_matrix, self.dist_coeffs
#             )
#             for i, corner in enumerate(corners):
#                 marker_id = int(ids[i][0])
#                 pts = corner.reshape((4, 2)).astype(int)
#                 top_left = (int(pts[0][0]), int(pts[0][1]))

#                 cv2.putText(frame, f"ID: {marker_id}", top_left,
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
#                 cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

#                 cx = int(np.mean(pts[:, 0]))
#                 cy = int(np.mean(pts[:, 1]))
#                 cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

#                 tvec = tvecs[i][0]  # (tx, ty, tz)
#                 dist = float(np.linalg.norm(tvec))

#                 cv2.putText(frame, f"X:{tvec[0]:.2f} Y:{tvec[1]:.2f} Z:{tvec[2]:.2f}",
#                             (cx + 10, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#                 cv2.putText(frame, f"D:{dist:.2f} m", (cx + 10, cy + 40),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#                 cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
#                                   rvecs[i], tvecs[i], self.axis_length)

#                 detected_markers.append({
#                     "id": marker_id,
#                     "center_2d": (cx, cy),
#                     "position_3d": (float(tvec[0]), float(tvec[1]), float(tvec[2])),
#                     "distance": dist
#                 })

#         return frame, detected_markers, rvecs, tvecs

# # =========================
# # ROS2 노드: TF(/tf) + /odom 발행
# # =========================
# class ArucoToOdomNode(Node):
#     def __init__(self, cam_index=2):
#         super().__init__('aruco_to_odom')

#         # --- 파라미터 (런타임 토글) ---
#         self.declare_parameter('publish_tf', True)
#         self.declare_parameter('publish_odom', True)
#         self.publish_tf   = bool(self.get_parameter('publish_tf').value)
#         self.publish_odom = bool(self.get_parameter('publish_odom').value)
#         self.add_on_set_parameters_callback(self._on_set_params)

#         # --- 카메라 ---
#         self.cap = cv2.VideoCapture(cam_index)
#         if not self.cap.isOpened():
#             self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
#         if not self.cap.isOpened():
#             raise RuntimeError(f"Failed to open camera index {cam_index}")

#         # MJPG + 1920x1080
#         # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#         self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

#         w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#         h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#         self.get_logger().info(f"Camera opened at {w}x{h}")
#         if (w, h) != (1920, 1080):
#             self.get_logger().warn("Camera is not 1920x1080; intrinsics may mismatch.")

#         # --- ArUco ---
#         self.detector = ArucoDetector(CAMERA_MATRIX, DIST_COEFFS, MARKER_LENGTH, AXIS_LENGTH)

#         # --- ROS pub/TF ---
#         self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
#         self.tf_brd   = TransformBroadcaster(self)

#         # 이전 상태(속도 계산)
#         self.prev_stamp = None
#         self.prev_pose  = None  # (x, y, yaw)

#         # 타이머 루프
#         self.timer = self.create_timer(0.02, self.loop)  # 50Hz try

#     # 파라미터 런타임 반영
#     def _on_set_params(self, params):
#         for p in params:
#             if p.name == 'publish_tf' and p.type_ == Parameter.Type.BOOL:
#                 self.publish_tf = bool(p.value)
#                 self.get_logger().info(f"publish_tf set to {self.publish_tf}")
#             if p.name == 'publish_odom' and p.type_ == Parameter.Type.BOOL:
#                 self.publish_odom = bool(p.value)
#                 self.get_logger().info(f"publish_odom set to {self.publish_odom}")
#         return SetParametersResult(successful=True)

#     def loop(self):
#         ok, frame = self.cap.read()
#         if not ok or frame is None:
#             self.get_logger().warn("Camera frame read failed")
#             return

#         # ArUco 검출 (내부에서 binary 처리 수행)
#         annotated, markers, rvecs, tvecs = self.detector.detect_and_annotate(frame)
#         stamp = self.get_clock().now().to_msg()

#         if rvecs is None or tvecs is None or len(markers) == 0:
#             # 검출 실패 시: 마지막 포즈 유지 발행(원치 않으면 주석 처리)
#             if self.prev_pose is not None:
#                 x, y, yaw = self.prev_pose
#                 self._publish_tf_and_odom(stamp, x, y, yaw)
#             # 화면 표시
#             cv2.imshow("Aruco", cv2.resize(annotated, (960, 540)))
#             cv2.waitKey(1)
#             return

#         # 첫 번째 마커 기준
#         rvec = rvecs[0, 0, :]
#         tvec = tvecs[0, 0, :].astype(float)

#         # 카메라 회전 → 쿼터니언(x,y,z,w) → yaw
#         rmat, _ = cv2.Rodrigues(rvec)
#         q_xyzw = SciR.from_matrix(rmat).as_quat().astype(float)  # [x,y,z,w]
#         q_xyzw = normalize_quaternion(q_xyzw)
#         _, _, yaw_cam = euler_from_quat_xyzw(q_xyzw)

#         # 카메라 tvec -> 평면 (ax, ay) 추출
#         ax = float(tvec[ ARUCO_XY_AXES[0] ])
#         ay = float(tvec[ ARUCO_XY_AXES[1] ])

#         # === 변환 적용: (ax,ay,yaw_cam) → (x,y,yaw) in odom ===
#         x, y = convert_position(ax, ay)
#         yaw  = convert_yaw(yaw_cam)

#         # 발행
#         self._publish_tf_and_odom(stamp, x, y, yaw)

#         # 화면 표시
#         cv2.imshow("Aruco", cv2.resize(annotated, (960, 540)))
#         cv2.waitKey(1)

#     def _publish_tf_and_odom(self, stamp, x, y, yaw):
#         q = SciR.from_euler('z', yaw).as_quat()  # [x,y,z,w]

#         # TF: odom -> base_footprint
#         if self.publish_tf:
#             t = TransformStamped()
#             t.header.stamp = stamp
#             t.header.frame_id = 'odom'
#             t.child_frame_id  = 'base_footprint'
#             t.transform.translation.x = float(x)
#             t.transform.translation.y = float(y)
#             t.transform.translation.z = 0.0
#             t.transform.rotation.x = float(q[0])
#             t.transform.rotation.y = float(q[1])
#             t.transform.rotation.z = float(q[2])
#             t.transform.rotation.w = float(q[3])
#             self.tf_brd.sendTransform(t)

#         # /odom
#         if self.publish_odom:
#             odom = Odometry()
#             odom.header.stamp = stamp
#             odom.header.frame_id = 'odom'
#             odom.child_frame_id  = 'base_footprint'
#             odom.pose.pose.position.x = float(x)
#             odom.pose.pose.position.y = float(y)
#             odom.pose.pose.position.z = 0.0
#             odom.pose.pose.orientation.x = float(q[0])
#             odom.pose.pose.orientation.y = float(q[1])
#             odom.pose.pose.orientation.z = float(q[2])
#             odom.pose.pose.orientation.w = float(q[3])

#             # 속도(차분)
#             if self.prev_stamp is not None and self.prev_pose is not None:
#                 dt = (stamp.sec + stamp.nanosec*1e-9) - (self.prev_stamp.sec + self.prev_stamp.nanosec*1e-9)
#                 if dt > 1e-4:
#                     vx = (x - self.prev_pose[0]) / dt
#                     vy = (y - self.prev_pose[1]) / dt
#                     wyaw = wrap(yaw - self.prev_pose[2]) / dt
#                     odom.twist.twist.linear.x  = float(vx)
#                     odom.twist.twist.linear.y  = float(vy)
#                     odom.twist.twist.angular.z = float(wyaw)

#             # 대략적 공분산
#             odom.pose.covariance[0]  = 0.01  # x
#             odom.pose.covariance[7]  = 0.01  # y
#             odom.pose.covariance[35] = 0.02  # yaw
#             odom.twist.covariance[0] = 0.04
#             odom.twist.covariance[7] = 0.04
#             odom.twist.covariance[35]= 0.05

#             self.odom_pub.publish(odom)  # ✅ 수정된 부분

#     def destroy_node(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
#         super().destroy_node()

# # =========================
# # main
# # =========================
# def main():
#     rclpy.init()
#     node = ArucoToOdomNode(cam_index=2)
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

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
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as SciR

# =========================
# 유사변환 & 보정 파라미터
#  - 필요 시 값만 교체해서 사용
# =========================
SIMILARITY_S = 1.1731463819258798
SIMILARITY_R = np.array([[ 0.02926172, -0.99957178],
                         [-0.99957178, -0.02926172]])
SIMILARITY_T = np.array([-0.00178071,  0.01315473])
THETA_RAD    = math.radians(-88.32318748498734)
DET_R_SIGN   = -1   # -1: 미러 반전 포함, +1: 미러 없음

# =========================
# 카메라 파라미터 (1920x1080)
# =========================
CAMERA_MATRIX = np.array([
    [1185.96684, 0,        999.31995],
    [0,          890.7003, 569.28861],
    [0,          0,        1]
], dtype=np.float32)

# plumb_bob(k1,k2,p1,p2,k3,k4,k5,k6)
DIST_COEFFS = np.array([
    -9.413361e-02, -8.374589e-04, 3.176887e-04, -3.987077e-04, 3.289896e-03,
    0.0, 0.0, 0.0
], dtype=np.float32)

MARKER_LENGTH = 0.1   # 51 mm
AXIS_LENGTH   = 0.05

# =========================
# 유틸
# =========================
def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def convert_position(ax: float, ay: float) -> Tuple[float, float]:
    """Aruco 평면 (ax,ay) -> odom/map (mx,my)"""
    p = np.array([ax, ay])
    mp = (SIMILARITY_R @ p) * SIMILARITY_S + SIMILARITY_T
    return float(mp[0]), float(mp[1])

def convert_yaw(yaw_cam_rad: float) -> float:
    """카메라 기준 yaw -> odom/map 기준 yaw"""
    if DET_R_SIGN >= 0:
        return wrap(yaw_cam_rad + THETA_RAD)
    else:
        return wrap(-yaw_cam_rad + THETA_RAD)

def calculate_pose_from_corners(corners, camera_matrix, dist_coeffs, marker_length):
    """
    ArUco 마커의 코너 두 점을 이용해서 x, y, yaw 계산
    corners: 마커의 4개 코너점 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    순서: 좌상(0) -> 우상(1) -> 우하(2) -> 좌하(3)
    """
    # 코너 점들을 numpy 배열로 변환
    pts = corners.reshape((4, 2)).astype(np.float32)
    
    # 마커 중심점 계산
    center_2d = np.mean(pts, axis=0)
    
    # 마커의 방향 벡터 계산 (좌상 -> 우상)
    direction_vec = pts[1] - pts[0]  # 우상 - 좌상
    
    # 2D 상에서의 yaw 각도 계산 (이미지 좌표계)
    yaw_image = math.atan2(direction_vec[1], direction_vec[0])
    
    # 3D 포즈 추정을 위한 객체 포인트 (마커의 실제 크기)
    half_size = marker_length / 2.0
    object_points = np.array([
        [-half_size, -half_size, 0],  # 좌상
        [ half_size, -half_size, 0],  # 우상
        [ half_size,  half_size, 0],  # 우하
        [-half_size,  half_size, 0]   # 좌하
    ], dtype=np.float32)
    
    # PnP를 이용한 포즈 추정
    success, rvec, tvec = cv2.solvePnP(
        object_points, pts, camera_matrix, dist_coeffs
    )
    
    if not success:
        return None, None, None
    
    # tvec에서 x, y 추출 (z축은 거리)
    x_cam = float(tvec[0][0])
    y_cam = float(tvec[1][0])
    z_cam = float(tvec[2][0])
    
    # 거리 기반으로 스케일 조정 (선택사항)
    # distance = np.linalg.norm(tvec)
    
    return x_cam, y_cam, yaw_image

# =========================
# ArUco 검출 클래스 (코너 기반으로 수정)
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
        # 1) 흑백 + Otsu thresholding (binary 처리)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # 2) ArUco 마커 검출 (binary 이미지 사용)
        corners, ids, rejected = self.detector.detectMarkers(binary)
        detected_markers = []
        pose_data = None

        if ids is not None and len(ids) > 0:
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                pts = corner.reshape((4, 2)).astype(int)
                top_left = (int(pts[0][0]), int(pts[0][1]))

                # 마커 표시
                cv2.putText(frame, f"ID: {marker_id}", top_left,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                # 중심점
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                # 코너 기반 포즈 계산
                x_cam, y_cam, yaw_img = calculate_pose_from_corners(
                    corner, self.camera_matrix, self.dist_coeffs, self.marker_length
                )
                
                if x_cam is not None:
                    # 거리 계산
                    distance = math.sqrt(x_cam**2 + y_cam**2)
                    
                    # 화면에 정보 표시
                    cv2.putText(frame, f"X:{x_cam:.2f} Y:{y_cam:.2f}",
                                (cx + 10, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(frame, f"D:{distance:.2f} Yaw:{math.degrees(yaw_img):.1f}°",
                                (cx + 10, cy + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # 방향 벡터 표시
                    direction_end = (int(cx + 50 * math.cos(yaw_img)), 
                                   int(cy + 50 * math.sin(yaw_img)))
                    cv2.arrowedLine(frame, (cx, cy), direction_end, (0, 255, 255), 3)
                    
                    # 첫 번째 마커의 포즈 데이터 저장
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
# ROS2 노드: TF(/tf) + /odom 발행
# =========================
class ArucoToOdomNode(Node):
    def __init__(self, cam_index=2):
        super().__init__('aruco_to_odom')

        # --- 파라미터 (런타임 토글) ---
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_odom', True)
        self.publish_tf   = bool(self.get_parameter('publish_tf').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.add_on_set_parameters_callback(self._on_set_params)

        # --- 카메라 ---
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {cam_index}")

        # MJPG + 1920x1080
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera FPS: {fps}")

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera opened at {w}x{h}")
        if (w, h) != (1920, 1080):
            self.get_logger().warn("Camera is not 1920x1080; intrinsics may mismatch.")

        # --- ArUco ---
        self.detector = ArucoDetector(CAMERA_MATRIX, DIST_COEFFS, MARKER_LENGTH, AXIS_LENGTH)

        # --- ROS pub/TF ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.tf_brd   = TransformBroadcaster(self)

        # 이전 상태(속도 계산)
        self.prev_stamp = None
        self.prev_pose  = None  # (x, y, yaw)

        # 타이머 루프
        self.timer = self.create_timer(0.02, self.loop)  # 50Hz try

    # 파라미터 런타임 반영
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'publish_tf' and p.type_ == Parameter.Type.BOOL:
                self.publish_tf = bool(p.value)
                self.get_logger().info(f"publish_tf set to {self.publish_tf}")
            if p.name == 'publish_odom' and p.type_ == Parameter.Type.BOOL:
                self.publish_odom = bool(p.value)
                self.get_logger().info(f"publish_odom set to {self.publish_odom}")
        return SetParametersResult(successful=True)

    def loop(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera frame read failed")
            return

        # ArUco 검출 (코너 기반)
        annotated, markers, pose_data = self.detector.detect_and_annotate(frame)
        stamp = self.get_clock().now().to_msg()

        if pose_data is None or len(markers) == 0:
            # 검출 실패 시: 마지막 포즈 유지 발행(원치 않으면 주석 처리)
            if self.prev_pose is not None:
                x, y, yaw = self.prev_pose
                self._publish_tf_and_odom(stamp, x, y, yaw)
            # 화면 표시
            cv2.imshow("Aruco", cv2.resize(annotated, (960, 540)))
            cv2.waitKey(1)
            return

        # 포즈 데이터 추출
        x_cam, y_cam, yaw_img = pose_data

        # === 변환 적용: (x_cam, y_cam, yaw_img) → (x, y, yaw) in odom ===
        # 위치 변환
        x, y = convert_position(x_cam, y_cam)
        
        # 각도 변환
        yaw = convert_yaw(yaw_img)

        # 발행
        self._publish_tf_and_odom(stamp, x, y, yaw)

        # 화면 표시
        cv2.imshow("Aruco", cv2.resize(annotated, (960, 540)))
        cv2.waitKey(1)

    def _publish_tf_and_odom(self, stamp, x, y, yaw):
        q = SciR.from_euler('z', yaw).as_quat()  # [x,y,z,w]

        # TF: odom -> base_footprint
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id  = 'base_footprint'
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.tf_brd.sendTransform(t)

        # /odom
        if self.publish_odom:
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

            # 속도(차분)
            if self.prev_stamp is not None and self.prev_pose is not None:
                dt = (stamp.sec + stamp.nanosec*1e-9) - (self.prev_stamp.sec + self.prev_stamp.nanosec*1e-9)
                if dt > 1e-4:
                    vx = (x - self.prev_pose[0]) / dt
                    vy = (y - self.prev_pose[1]) / dt
                    wyaw = wrap(yaw - self.prev_pose[2]) / dt
                    odom.twist.twist.linear.x  = float(vx)
                    odom.twist.twist.linear.y  = float(vy)
                    odom.twist.twist.angular.z = float(wyaw)

            # 대략적 공분산
            odom.pose.covariance[0]  = 0.01  # x
            odom.pose.covariance[7]  = 0.01  # y
            odom.pose.covariance[35] = 0.02  # yaw
            odom.twist.covariance[0] = 0.04
            odom.twist.covariance[7] = 0.04
            odom.twist.covariance[35]= 0.05

            self.odom_pub.publish(odom)

        # 상태 저장
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