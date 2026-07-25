#!/usr/bin/env python3
"""
YOLO 기반 조난자(사람) 탐지 노드.

참고: https://github.com/THENAEUN/go2-cave-survivor-detection
capstone2 프로젝트의 기존 asus_camera(camera/image_raw, camera/depth/image_raw)에 맞게
토픽명만 이 프로젝트에 맞춰 수정했습니다.
"""

import os
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO


# ============================================================
# 사용자 설정
# ============================================================

# RGB 및 Depth 토픽 (capstone2 go2_config의 asus_camera 실제 토픽명)
IMAGE_TOPIC = "/camera/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/depth/camera_info"

# 로봇의 Gazebo Ground Truth 위치 (champ_base/scripts/ground_truth_odom_bridge.py가
# 구독하는 것과 같은, CHAMP 다리-운동학 오도메트리를 우회하는 원본 토픽)
GROUND_TRUTH_ODOM_TOPIC = "/odom/ground_truth"

# 계산된 조난자 좌표 발행 토픽
SURVIVOR_POSITION_TOPIC = "/survivor_position"

# 좌표계 이름 (이 프로젝트는 map == odom == Gazebo world 좌표계이므로 map을 쓴다.
# 자세한 이유는 view_map.launch.py 주석 참고)
WORLD_FRAME = "map"
BASE_FRAME = "base_link"
DEFAULT_CAMERA_FRAME = "camera_depth_optical_frame"

# Go2 이동 명령 토픽
CMD_VEL_TOPIC = "/cmd_vel"

# 사람 탐지 신뢰도 기준
CONFIDENCE_THRESHOLD = 0.55

# 연속 탐지 횟수
DETECT_CONFIRM_FRAMES = 3

# YOLO 추론 최대 빈도
MAX_INFERENCE_FPS = 3.0

# YOLO 입력 이미지 크기
INFERENCE_IMAGE_SIZE = 640

# 한 번 탐지되면 정지 상태 유지
LATCH_STOP = True

# 탐지 결과 창 표시
SHOW_WINDOW = True

# 유효한 Depth 범위
MIN_DEPTH_M = 0.10
MAX_DEPTH_M = 20.0

# 바운딩 박스 중심에서 거리 측정에 사용할 영역 비율
DEPTH_ROI_RATIO = 0.30

# 거리 계산에 필요한 최소 유효 픽셀 수
MIN_VALID_DEPTH_PIXELS = 10

# 조난자와 이 거리 이하가 되면 정지
STOP_DISTANCE_M = 1.0


def find_model_path() -> str:
    """사용할 YOLO 모델 파일을 찾는다."""

    project_dir = Path.home() / "capstone2"
    env_model = os.getenv("YOLO_MODEL")

    candidates = [
        Path(env_model).expanduser() if env_model else None,
        project_dir / "yolo11n.pt",
        project_dir / "yolov8n.pt",
        project_dir / "go2_ws" / "yolo11n.pt",
    ]

    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return str(candidate)

    return "yolo11n.pt"


class YoloSurvivorDetector(Node):
    def __init__(self):
        super().__init__("yolo_survivor_detector")

        self.model_path = find_model_path()

        self.get_logger().info(
            f"YOLO 모델을 불러옵니다: {self.model_path}"
        )

        self.model = YOLO(self.model_path)

        # 가장 최근에 수신한 Depth 영상
        self.latest_depth_image: Optional[np.ndarray] = None

        # Depth 카메라 내부 파라미터
        self.camera_fx: Optional[float] = None
        self.camera_fy: Optional[float] = None
        self.camera_cx: Optional[float] = None
        self.camera_cy: Optional[float] = None
        self.camera_frame_id = DEFAULT_CAMERA_FRAME

        # 로봇의 최신 world(=map) 기준 위치와 자세
        self.latest_ground_truth: Optional[Odometry] = None

        # base_link와 카메라 optical frame 사이 TF 조회
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # RGB 영상 구독
        self.image_subscriber = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Depth 영상 구독
        self.depth_subscriber = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        # Depth 카메라 내부 파라미터 구독
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10,
        )

        # 로봇의 world(=map) 기준 위치와 자세 구독
        self.ground_truth_subscriber = self.create_subscription(
            Odometry,
            GROUND_TRUTH_ODOM_TOPIC,
            self.ground_truth_callback,
            10,
        )

        # 정지 명령 발행
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10,
        )

        # 계산된 조난자의 world(=map) 좌표 발행
        self.survivor_position_publisher = self.create_publisher(
            PointStamped,
            SURVIVOR_POSITION_TOPIC,
            10,
        )

        # 정지 상태일 때 0 속도를 반복 발행
        self.stop_timer = self.create_timer(
            0.05,
            self.stop_timer_callback,
        )

        self.detect_streak = 0
        self.stop_active = False
        self.last_inference_time = 0.0
        self.last_distance_log_time = 0.0
        self.window_available = SHOW_WINDOW

        self.get_logger().info(
            f"RGB 토픽 대기 중: {IMAGE_TOPIC}"
        )
        self.get_logger().info(
            f"Depth 토픽 대기 중: {DEPTH_TOPIC}"
        )
        self.get_logger().info(
            f"사람이 {DETECT_CONFIRM_FRAMES}회 연속 탐지되고 "
            f"거리가 {STOP_DISTANCE_M:.2f} m 이하가 되면 "
            "Go2를 정지합니다."
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Depth 카메라 내부 파라미터를 저장한다."""

        if len(msg.k) < 9:
            return

        first_time = self.camera_fx is None

        self.camera_fx = float(msg.k[0])
        self.camera_fy = float(msg.k[4])
        self.camera_cx = float(msg.k[2])
        self.camera_cy = float(msg.k[5])

        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id

        if first_time:
            self.get_logger().info(
                "Depth CameraInfo 수신: "
                f"frame={self.camera_frame_id}, "
                f"fx={self.camera_fx:.3f}, fy={self.camera_fy:.3f}, "
                f"cx={self.camera_cx:.3f}, cy={self.camera_cy:.3f}"
            )

    def ground_truth_callback(self, msg: Odometry):
        """로봇의 최신 world(=map) 기준 위치와 자세를 저장한다."""

        self.latest_ground_truth = msg

    def depth_callback(self, msg: Image):
        """32FC1 Depth 영상을 NumPy 배열로 저장한다.

        cv_bridge의 컴파일된 boost 확장이 이 환경의 NumPy 2.x와
        ABI가 맞지 않아 임포트는 되어도 호출 시 크래시하므로,
        cv_bridge 없이 raw bytes를 직접 numpy로 해석한다.
        """

        try:
            self.latest_depth_image = np.frombuffer(
                msg.data,
                dtype=np.float32,
            ).reshape(msg.height, msg.width)

        except Exception as error:
            self.get_logger().error(
                f"Depth 영상을 변환하지 못했습니다: {error}"
            )

    @staticmethod
    def quaternion_rotation_matrix(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> np.ndarray:
        """Quaternion을 3x3 회전행렬로 변환한다."""

        norm = np.sqrt(x * x + y * y + z * z + w * w)

        if norm < 1.0e-12:
            return np.eye(3, dtype=np.float64)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def get_robot_world_position(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        """로봇 base_link의 world(=map) 좌표를 반환한다."""

        if self.latest_ground_truth is None:
            return None

        position = self.latest_ground_truth.pose.pose.position
        return (float(position.x), float(position.y), float(position.z))

    def calculate_survivor_world_position(
        self,
        pixel_u: int,
        pixel_v: int,
        depth_m: float,
    ) -> Optional[Tuple[float, float, float]]:
        """
        조난자의 Depth 픽셀을 카메라 3차원 좌표로 변환하고,
        다시 base_link와 world(=map) 좌표로 변환한다.
        """

        if (
            self.camera_fx is None
            or self.camera_fy is None
            or self.camera_cx is None
            or self.camera_cy is None
            or self.latest_ground_truth is None
        ):
            return None

        if depth_m <= 0.0 or not np.isfinite(depth_m):
            return None

        # Depth optical frame 좌표계: X=화면 오른쪽, Y=화면 아래, Z=카메라 정면
        camera_x = (float(pixel_u) - self.camera_cx) * depth_m / self.camera_fx
        camera_y = (float(pixel_v) - self.camera_cy) * depth_m / self.camera_fy
        camera_z = depth_m

        point_camera = np.array([camera_x, camera_y, camera_z], dtype=np.float64)

        try:
            # 카메라 optical frame의 점을 base_link 기준으로 변환 (고정 TF)
            camera_to_base = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                self.camera_frame_id,
                Time(),
            )

        except TransformException as error:
            self.get_logger().debug(
                f"카메라 TF를 조회하지 못했습니다: {error}"
            )
            return None

        translation = camera_to_base.transform.translation
        rotation = camera_to_base.transform.rotation

        camera_to_base_rotation = self.quaternion_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w,
        )

        point_base = camera_to_base_rotation @ point_camera + np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64,
        )

        # /odom/ground_truth의 pose가 이미 world(=map) 기준 base_link 자세다.
        robot_pose = self.latest_ground_truth.pose.pose
        robot_position = robot_pose.position
        robot_orientation = robot_pose.orientation

        base_to_world_rotation = self.quaternion_rotation_matrix(
            robot_orientation.x,
            robot_orientation.y,
            robot_orientation.z,
            robot_orientation.w,
        )

        point_world = base_to_world_rotation @ point_base + np.array(
            [robot_position.x, robot_position.y, robot_position.z],
            dtype=np.float64,
        )

        return (float(point_world[0]), float(point_world[1]), float(point_world[2]))

    def publish_survivor_position(self, position: Tuple[float, float, float]):
        """조난자의 world(=map) 좌표를 PointStamped로 발행한다."""

        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = WORLD_FRAME
        message.point.x, message.point.y, message.point.z = position

        self.survivor_position_publisher.publish(message)

    def estimate_depth_measurement(
        self,
        bbox: Tuple[float, float, float, float],
        rgb_shape,
    ) -> Optional[Tuple[float, int, int]]:
        """
        사람 바운딩 박스 중심 영역의 Depth 중앙값과
        Depth 영상 기준 중심 픽셀 좌표를 반환한다.

        반환값: (distance_m, center_x, center_y)
        """

        if self.latest_depth_image is None:
            return None

        depth_image = self.latest_depth_image

        if depth_image.ndim != 2:
            return None

        rgb_height, rgb_width = rgb_shape[:2]
        depth_height, depth_width = depth_image.shape

        x1, y1, x2, y2 = bbox

        scale_x = depth_width / float(rgb_width)
        scale_y = depth_height / float(rgb_height)

        x1 = int(x1 * scale_x)
        x2 = int(x2 * scale_x)
        y1 = int(y1 * scale_y)
        y2 = int(y2 * scale_y)

        x1 = max(0, min(x1, depth_width - 1))
        x2 = max(0, min(x2, depth_width))
        y1 = max(0, min(y1, depth_height - 1))
        y2 = max(0, min(y2, depth_height))

        if x2 <= x1 or y2 <= y1:
            return None

        box_width = x2 - x1
        box_height = y2 - y1

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        roi_width = max(4, int(box_width * DEPTH_ROI_RATIO))
        roi_height = max(4, int(box_height * DEPTH_ROI_RATIO))

        roi_x1 = max(0, center_x - roi_width // 2)
        roi_x2 = min(depth_width, center_x + roi_width // 2)

        roi_y1 = max(0, center_y - roi_height // 2)
        roi_y2 = min(depth_height, center_y + roi_height // 2)

        depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]

        if depth_roi.size == 0:
            return None

        valid_mask = (
            np.isfinite(depth_roi)
            & (depth_roi >= MIN_DEPTH_M)
            & (depth_roi <= MAX_DEPTH_M)
        )

        valid_depth_values = depth_roi[valid_mask]

        if valid_depth_values.size < MIN_VALID_DEPTH_PIXELS:
            return None

        return float(np.median(valid_depth_values)), center_x, center_y

    def image_callback(self, msg: Image):
        current_time = time.monotonic()
        minimum_interval = 1.0 / MAX_INFERENCE_FPS

        if (
            current_time - self.last_inference_time
            < minimum_interval
        ):
            return

        self.last_inference_time = current_time

        try:
            rgb_frame = np.frombuffer(
                msg.data,
                dtype=np.uint8,
            ).reshape(msg.height, msg.width, 3)

            # 카메라는 rgb8로 퍼블리시하므로 OpenCV/YOLO가 기대하는
            # BGR 순서로 채널을 뒤집는다 (cv_bridge 미사용).
            frame = rgb_frame[:, :, ::-1]

        except Exception as error:
            self.get_logger().error(
                "ROS RGB 이미지를 OpenCV 이미지로 "
                f"변환하지 못했습니다: {error}"
            )
            return

        try:
            results = self.model.predict(
                source=frame,
                classes=[0],
                conf=CONFIDENCE_THRESHOLD,
                imgsz=INFERENCE_IMAGE_SIZE,
                verbose=False,
                # 이 환경의 pip CUDA/cuDNN 조합이 서로 버전이 맞지 않아
                # (CUDNN_STATUS_SUBLIBRARY_VERSION_MISMATCH) GPU 추론이
                # 실패한다. nano 모델은 CPU로도 충분히 빠르므로 CPU 고정.
                device="cpu",
            )

        except Exception as error:
            self.get_logger().error(
                f"YOLO 추론 중 오류가 발생했습니다: {error}"
            )
            return

        result = results[0]

        detections = []
        highest_confidence = 0.0

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())

                if (
                    class_id != 0
                    or confidence < CONFIDENCE_THRESHOLD
                ):
                    continue

                coordinates = box.xyxy[0].cpu().numpy()

                x1, y1, x2, y2 = [
                    float(value)
                    for value in coordinates
                ]

                depth_measurement = self.estimate_depth_measurement(
                    bbox=(x1, y1, x2, y2),
                    rgb_shape=frame.shape,
                )

                if depth_measurement is None:
                    distance_m = None
                    survivor_world_position = None
                else:
                    distance_m, depth_pixel_u, depth_pixel_v = depth_measurement
                    survivor_world_position = self.calculate_survivor_world_position(
                        pixel_u=depth_pixel_u,
                        pixel_v=depth_pixel_v,
                        depth_m=distance_m,
                    )

                detections.append(
                    {
                        "bbox": (x1, y1, x2, y2),
                        "confidence": confidence,
                        "distance_m": distance_m,
                        "world_position": survivor_world_position,
                    }
                )

                highest_confidence = max(
                    highest_confidence,
                    confidence,
                )

        person_count = len(detections)
        person_found = person_count > 0

        valid_detections = [
            detection
            for detection in detections
            if detection["distance_m"] is not None
        ]

        closest_detection = (
            min(valid_detections, key=lambda detection: detection["distance_m"])
            if valid_detections
            else None
        )

        closest_distance_m = (
            closest_detection["distance_m"]
            if closest_detection is not None
            else None
        )

        closest_survivor_world_position = (
            closest_detection["world_position"]
            if closest_detection is not None
            else None
        )

        if person_found:
            self.detect_streak += 1
        else:
            self.detect_streak = 0

            if not LATCH_STOP:
                self.stop_active = False

        # 탐지 중 거리값을 약 1초마다 출력
        if (
            person_found
            and current_time - self.last_distance_log_time >= 1.0
        ):
            self.last_distance_log_time = current_time

            if closest_distance_m is not None:
                robot_world_position = self.get_robot_world_position()
                coordinate_text = ""

                if (
                    robot_world_position is not None
                    and closest_survivor_world_position is not None
                ):
                    coordinate_text = (
                        ", 로봇 좌표=("
                        f"{robot_world_position[0]:.2f}, "
                        f"{robot_world_position[1]:.2f}, "
                        f"{robot_world_position[2]:.2f}), "
                        "조난자 좌표=("
                        f"{closest_survivor_world_position[0]:.2f}, "
                        f"{closest_survivor_world_position[1]:.2f}, "
                        f"{closest_survivor_world_position[2]:.2f})"
                    )

                self.get_logger().info(
                    "조난자 탐지 중: "
                    f"인원={person_count}, "
                    f"가장 가까운 거리={closest_distance_m:.2f} m, "
                    f"연속 탐지={self.detect_streak}/"
                    f"{DETECT_CONFIRM_FRAMES}"
                    f"{coordinate_text}"
                )
            else:
                self.get_logger().info(
                    "조난자 탐지 중: "
                    f"인원={person_count}, "
                    "거리값 없음, "
                    f"연속 탐지={self.detect_streak}/"
                    f"{DETECT_CONFIRM_FRAMES}"
                )

        # 연속 탐지 조건을 만족한 조난자의 world(=map) 좌표 발행
        if (
            self.detect_streak >= DETECT_CONFIRM_FRAMES
            and closest_survivor_world_position is not None
        ):
            self.publish_survivor_position(closest_survivor_world_position)

        if (
            not self.stop_active
            and self.detect_streak >= DETECT_CONFIRM_FRAMES
            and closest_distance_m is not None
            and closest_distance_m <= STOP_DISTANCE_M
        ):
            self.stop_active = True
            self.publish_stop_command()

            if closest_distance_m is not None:
                distance_text = (
                    f", 추정 거리={closest_distance_m:.2f} m"
                )
            else:
                distance_text = ", 추정 거리=계산 불가"

            self.get_logger().warn(
                "조난자 감지! Go2를 정지합니다. "
                f"탐지 인원={person_count}, "
                f"최대 신뢰도={highest_confidence:.2f}"
                f"{distance_text}"
            )

        self.show_detection_result(
            result=result,
            detections=detections,
            person_count=person_count,
            highest_confidence=highest_confidence,
            closest_distance_m=closest_distance_m,
        )

    def stop_timer_callback(self):
        if self.stop_active:
            self.publish_stop_command()

    def publish_stop_command(self):
        stop_message = Twist()

        stop_message.linear.x = 0.0
        stop_message.linear.y = 0.0
        stop_message.linear.z = 0.0

        stop_message.angular.x = 0.0
        stop_message.angular.y = 0.0
        stop_message.angular.z = 0.0

        self.cmd_vel_publisher.publish(stop_message)

    def show_detection_result(
        self,
        result,
        detections,
        person_count: int,
        highest_confidence: float,
        closest_distance_m: Optional[float],
    ):
        if not self.window_available:
            return

        try:
            annotated_frame = result.plot()

            # 각 사람 바운딩 박스 위에 거리 표시
            for detection in detections:
                x1, y1, _, _ = detection["bbox"]
                distance_m = detection["distance_m"]

                if distance_m is None:
                    distance_text = "Depth: N/A"
                else:
                    distance_text = f"Distance: {distance_m:.2f} m"

                text_y = max(20, int(y1) - 10)

                cv2.putText(
                    annotated_frame,
                    distance_text,
                    (int(x1), text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            if self.stop_active:
                status_text = (
                    "SURVIVOR DETECTED - ROBOT STOPPED"
                )
            elif person_count > 0:
                status_text = (
                    f"Confirming detection "
                    f"{self.detect_streak}/"
                    f"{DETECT_CONFIRM_FRAMES}"
                )
            else:
                status_text = "SEARCHING"

            cv2.putText(
                annotated_frame,
                status_text,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (
                    (0, 0, 255)
                    if self.stop_active
                    else (255, 255, 255)
                ),
                2,
                cv2.LINE_AA,
            )

            if closest_distance_m is None:
                distance_summary = "Closest distance: N/A"
            else:
                distance_summary = (
                    f"Closest distance: "
                    f"{closest_distance_m:.2f} m"
                )

            cv2.putText(
                annotated_frame,
                (
                    f"Persons: {person_count}  "
                    f"Confidence: {highest_confidence:.2f}"
                ),
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                annotated_frame,
                distance_summary,
                (15, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(
                "Go2 Survivor Detection",
                annotated_frame,
            )

            pressed_key = cv2.waitKey(1) & 0xFF

            if pressed_key == ord("q"):
                self.get_logger().info(
                    "Q 키 입력으로 탐지 노드를 종료합니다."
                )
                rclpy.shutdown()

        except cv2.error as error:
            self.get_logger().warn(
                f"탐지 화면을 표시할 수 없습니다: {error}"
            )
            self.window_available = False

    def destroy_node(self):
        if self.stop_active:
            self.publish_stop_command()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    detector_node = None

    try:
        detector_node = YoloSurvivorDetector()
        rclpy.spin(detector_node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f"탐지 노드 실행 오류: {error}")

    finally:
        if detector_node is not None:
            detector_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
