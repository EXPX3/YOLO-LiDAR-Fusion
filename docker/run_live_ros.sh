#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
FUSION_CONFIG_PATH="${FUSION_CONFIG_PATH:-/ros2_ws/src/vlp16zed2ifusion/config/fusion_overlay_params.yaml}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /app/venv/bin/activate
set -u

cd /app/YOLO-LiDAR-Fusion/Code

cmd=(python main.py ros --fusion-config-path "${FUSION_CONFIG_PATH}")

if [[ -n "${YOLO_MODE:-}" ]]; then
  cmd+=(--mode "${YOLO_MODE}")
fi

if [[ -n "${YOLO_MODEL_SIZE:-}" ]]; then
  cmd+=(--model-size "${YOLO_MODEL_SIZE}")
fi

if [[ -n "${YOLO_EROSION:-}" ]]; then
  cmd+=(--erosion "${YOLO_EROSION}")
fi

if [[ -n "${YOLO_DEPTH:-}" ]]; then
  cmd+=(--depth "${YOLO_DEPTH}")
fi

if [[ -n "${YOLO_IMAGE_TOPIC:-}" ]]; then
  cmd+=(--image-topic "${YOLO_IMAGE_TOPIC}")
fi

if [[ -n "${YOLO_CAMERA_INFO_TOPIC:-}" ]]; then
  cmd+=(--camera-info-topic "${YOLO_CAMERA_INFO_TOPIC}")
fi

if [[ -n "${YOLO_LIDAR_TOPIC:-}" ]]; then
  cmd+=(--lidar-topic "${YOLO_LIDAR_TOPIC}")
fi

if [[ -n "${YOLO_OUTPUT_TOPIC:-}" ]]; then
  cmd+=(--output-topic "${YOLO_OUTPUT_TOPIC}")
fi

if [[ -n "${YOLO_MAX_CLOUD_AGE:-}" ]]; then
  cmd+=(--max-cloud-age "${YOLO_MAX_CLOUD_AGE}")
fi

if [[ "${YOLO_PCA:-0}" == "1" ]]; then
  cmd+=(--pca True)
fi

if [[ "${YOLO_DISPLAY:-1}" == "0" ]]; then
  cmd+=(--no-display)
fi

exec "${cmd[@]}" "$@"
