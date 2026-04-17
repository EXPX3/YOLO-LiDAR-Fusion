from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
from calibration import LiDAR2CameraLive
from detector import YOLOv8Detector
from visualization import annotate_projected_object_distances, draw_projected_3D_points

import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener


def derive_camera_info_topic(image_topic: str) -> str:
    if image_topic.endswith('/image'):
        return image_topic.rsplit('/', 1)[0] + '/camera_info'
    return image_topic + '/camera_info'


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def transform_to_matrix(transform) -> np.ndarray:
    x = transform.transform.rotation.x
    y = transform.transform.rotation.y
    z = transform.transform.rotation.z
    w = transform.transform.rotation.w

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ],
        dtype=np.float64,
    )
    return matrix


def pointcloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    if msg.width == 0 or msg.height == 0:
        return np.empty((0, 3), dtype=np.float32)

    fields = {field.name: field for field in msg.fields}
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return np.empty((0, 3), dtype=np.float32)

    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype(
        {
            'names': ['x', 'y', 'z'],
            'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
            'offsets': [fields['x'].offset, fields['y'].offset, fields['z'].offset],
            'itemsize': msg.point_step,
        }
    )

    raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    points = np.empty((raw.shape[0], 3), dtype=np.float32)
    points[:, 0] = raw['x']
    points[:, 1] = raw['y']
    points[:, 2] = raw['z']

    finite_mask = np.isfinite(points).all(axis=1)
    return points[finite_mask]


def load_camera_matrix_from_yaml(camera_calibration_file: str) -> Optional[np.ndarray]:
    if not camera_calibration_file:
        return None

    calibration_path = Path(camera_calibration_file)
    if not calibration_path.is_file():
        return None

    with calibration_path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    if 'camera_matrix' in data and isinstance(data['camera_matrix'], dict):
        matrix_values = data['camera_matrix'].get('data')
    elif 'K' in data:
        matrix_values = data['K']
    elif 'k' in data:
        matrix_values = data['k']
    else:
        matrix_values = None

    if matrix_values is None or len(matrix_values) != 9:
        return None

    return np.asarray(matrix_values, dtype=np.float64).reshape(3, 3)


def load_defaults_from_fusion_config(fusion_config_path: str) -> dict:
    if not fusion_config_path:
        return {}

    config_path = Path(fusion_config_path)
    if not config_path.is_file():
        return {}

    with config_path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    return data.get('vlp16zed2ifusion', {}).get('ros__parameters', {})


def detect_display_geometry(default_width: int, default_height: int) -> tuple[int, int, int, int]:
    default_x = int(os.environ.get('YOLO_WINDOW_X', '0'))
    default_y = int(os.environ.get('YOLO_WINDOW_Y', '0'))

    try:
        result = subprocess.run(
            ['xrandr', '--current'],
            check=True,
            capture_output=True,
            text=True,
            env=os.environ.copy(),
        )
    except Exception:
        return default_x, default_y, default_width, default_height

    geometry_pattern = re.compile(r'(?P<w>\d+)x(?P<h>\d+)\+(?P<x>\d+)\+(?P<y>\d+)')
    primary_geometry = None
    connected_geometry = None

    for line in result.stdout.splitlines():
        if ' connected' not in line:
            continue

        match = geometry_pattern.search(line)
        if not match:
            continue

        geometry = (
            int(match.group('x')),
            int(match.group('y')),
            int(match.group('w')),
            int(match.group('h')),
        )

        if ' primary ' in line:
            primary_geometry = geometry
            break

        if connected_geometry is None:
            connected_geometry = geometry

    return primary_geometry or connected_geometry or (default_x, default_y, default_width, default_height)


class LiveMaskedProjectionNode(Node):
    def __init__(
        self,
        detector: YOLOv8Detector,
        erosion_factor: int,
        depth_factor: int,
        fusion_config_path: str,
        camera_calibration_file: str,
        image_topic: str,
        camera_info_topic: str,
        lidar_topic: str,
        output_topic: str,
        max_cloud_age_sec: Optional[float],
        display: bool,
    ) -> None:
        super().__init__('yolo_lidar_fusion_live')

        fusion_defaults = load_defaults_from_fusion_config(fusion_config_path)

        self.detector = detector
        self.erosion_factor = erosion_factor
        self.depth_factor = depth_factor
        self.display = display
        self.window_name = 'YOLO-LiDAR-Fusion Live'
        self.window_initialized = False
        self.window_fullscreen = os.environ.get('YOLO_WINDOW_FULLSCREEN', '1') != '0'
        self.window_width = int(os.environ.get('YOLO_WINDOW_WIDTH', '1920'))
        self.window_height = int(os.environ.get('YOLO_WINDOW_HEIGHT', '1080'))
        self.window_x, self.window_y, self.window_width, self.window_height = detect_display_geometry(
            self.window_width,
            self.window_height,
        )
        self.warned_about_window = False
        self.bridge = CvBridge()
        self.latest_cloud: Optional[PointCloud2] = None
        self.camera_matrix = load_camera_matrix_from_yaml(camera_calibration_file)
        self.camera_info_frame_id = ''
        self.image_topic = image_topic or fusion_defaults.get('image_topic', '/zed/zed_node/rgb/color/rect/image')
        self.camera_info_topic = camera_info_topic or fusion_defaults.get(
            'camera_info_topic',
            derive_camera_info_topic(self.image_topic),
        )
        self.lidar_topic = lidar_topic or fusion_defaults.get('lidar_topic', '/velodyne_points')
        self.output_topic = output_topic
        self.max_cloud_age_sec = (
            max_cloud_age_sec if max_cloud_age_sec is not None else float(fusion_defaults.get('max_cloud_age_sec', 0.20))
        )

        self.warned_about_cloud = False
        self.warned_about_camera_info = False
        self.warned_about_tf = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.image_pub = None
        if self.output_topic:
            self.image_pub = self.create_publisher(Image, self.output_topic, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        if self.camera_matrix is not None:
            self.get_logger().info(f'Loaded camera intrinsics from {camera_calibration_file}')
        if fusion_config_path:
            self.get_logger().info(f'Using live defaults from {fusion_config_path}')
        self.get_logger().info(
            f'Listening to image={self.image_topic}, camera_info={self.camera_info_topic}, lidar={self.lidar_topic}'
        )
        if self.output_topic:
            self.get_logger().info(f'Publishing masked projected image to {self.output_topic}')
        if self.display:
            mode = (
                f'fill-primary-display {self.window_width}x{self.window_height}+{self.window_x}+{self.window_y}'
                if self.window_fullscreen
                else f'{self.window_width}x{self.window_height}+{self.window_x}+{self.window_y}'
            )
            self.get_logger().info(f'Opening live display window in {mode} mode')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
            return

        self.camera_matrix = camera_matrix
        self.camera_info_frame_id = msg.header.frame_id

    def lidar_callback(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        output_image = frame.copy()

        if self.camera_matrix is None:
            if not self.warned_about_camera_info:
                self.get_logger().warn(f'Waiting for camera intrinsics on {self.camera_info_topic}')
                self.warned_about_camera_info = True
            self.publish_and_display(output_image, msg)
            return

        if self.latest_cloud is None:
            if not self.warned_about_cloud:
                self.get_logger().warn(f'Waiting for PointCloud2 on {self.lidar_topic}')
                self.warned_about_cloud = True
            self.publish_and_display(output_image, msg)
            return

        cloud = self.latest_cloud
        image_time = stamp_to_seconds(msg.header.stamp)
        cloud_time = stamp_to_seconds(cloud.header.stamp)
        if image_time > 0.0 and cloud_time > 0.0 and abs(image_time - cloud_time) > self.max_cloud_age_sec:
            self.publish_and_display(output_image, msg)
            return

        source_frame = cloud.header.frame_id
        target_frame = msg.header.frame_id or self.camera_info_frame_id
        if not source_frame or not target_frame:
            self.publish_and_display(output_image, msg)
            return

        try:
            lidar_to_camera_transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            if not self.warned_about_tf:
                self.get_logger().warn(f'Cannot look up TF {target_frame} <- {source_frame}: {exc}')
                self.warned_about_tf = True
            self.publish_and_display(output_image, msg)
            return

        self.warned_about_tf = False
        points_lidar = pointcloud2_to_xyz_array(cloud)
        if points_lidar.size == 0:
            self.publish_and_display(output_image, msg)
            return

        lidar2cam = LiDAR2CameraLive(self.camera_matrix, transform_to_matrix(lidar_to_camera_transform))
        _, _, pts_3D, pts_2D, all_filtered_points_of_object, all_object_IDs = self.detector.process_frame(
            frame,
            points_lidar,
            lidar2cam,
            erosion_factor=self.erosion_factor,
            depth_factor=self.depth_factor,
        )

        points_to_draw = np.vstack(all_filtered_points_of_object) if all_filtered_points_of_object else np.empty((0, 3))
        draw_projected_3D_points(lidar2cam, output_image, pts_3D, pts_2D, points_to_draw)
        annotate_projected_object_distances(
            lidar2cam,
            output_image,
            all_filtered_points_of_object,
            object_ids=all_object_IDs,
        )

        object_count = len(all_filtered_points_of_object)
        point_count = points_to_draw.shape[0]
        cv2.putText(
            output_image,
            f'objects={object_count} projected_points={point_count}',
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        self.publish_and_display(output_image, msg)

    def publish_and_display(self, image: np.ndarray, source_msg: Image) -> None:
        if self.image_pub is not None:
            output_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            output_msg.header = source_msg.header
            self.image_pub.publish(output_msg)

        if self.display:
            self.ensure_display_window()
            cv2.imshow(self.window_name, self.prepare_image_for_display(image))
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyWindow(self.window_name)
                rclpy.shutdown()

    def ensure_display_window(self) -> None:
        if self.window_initialized:
            return

        window_flags = cv2.WINDOW_NORMAL
        if hasattr(cv2, 'WINDOW_FREERATIO'):
            window_flags |= cv2.WINDOW_FREERATIO

        cv2.namedWindow(self.window_name, window_flags)
        try:
            if hasattr(cv2, 'WND_PROP_ASPECT_RATIO') and hasattr(cv2, 'WINDOW_FREERATIO'):
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_FREERATIO)
            cv2.moveWindow(self.window_name, self.window_x, self.window_y)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        except cv2.error as exc:
            if not self.warned_about_window:
                self.get_logger().warn(f'Could not apply requested window mode: {exc}')
                self.warned_about_window = True
        self.window_initialized = True

    def prepare_image_for_display(self, image: np.ndarray) -> np.ndarray:
        image_height, image_width = image.shape[:2]
        if image_width <= 0 or image_height <= 0:
            return image

        target_width = self.window_width
        target_height = self.window_height
        if target_width <= 0 or target_height <= 0:
            return image

        scale = max(target_width / image_width, target_height / image_height)
        resized_width = max(1, int(round(image_width * scale)))
        resized_height = max(1, int(round(image_height * scale)))
        resized_image = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

        x_offset = max(0, (resized_width - target_width) // 2)
        y_offset = max(0, (resized_height - target_height) // 2)
        return resized_image[y_offset:y_offset + target_height, x_offset:x_offset + target_width]


def run_live_ros_inference(
    detector: YOLOv8Detector,
    erosion_factor: int,
    depth_factor: int,
    fusion_config_path: str,
    camera_calibration_file: str,
    image_topic: str,
    camera_info_topic: str,
    lidar_topic: str,
    output_topic: str,
    max_cloud_age_sec: Optional[float],
    display: bool,
) -> None:
    rclpy.init(args=None)
    node = LiveMaskedProjectionNode(
        detector=detector,
        erosion_factor=erosion_factor,
        depth_factor=depth_factor,
        fusion_config_path=fusion_config_path,
        camera_calibration_file=camera_calibration_file,
        image_topic=image_topic,
        camera_info_topic=camera_info_topic,
        lidar_topic=lidar_topic,
        output_topic=output_topic,
        max_cloud_age_sec=max_cloud_age_sec,
        display=display,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if display:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
