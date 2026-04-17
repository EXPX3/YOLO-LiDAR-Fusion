#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
FUSION_CONFIG_PATH="${FUSION_CONFIG_PATH:-/ros2_ws/src/vlp16zed2ifusion/config/fusion_overlay_params.yaml}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /app/venv/bin/activate
set -u

wait_for_first_message() {
  local topic="$1"
  local max_attempts="${2:-120}"
  local timeout_seconds="${3:-10}"
  local sleep_seconds="${4:-2}"
  local attempt=1

  while (( attempt <= max_attempts )); do
    if timeout "${timeout_seconds}" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      echo "Received first message on ${topic}"
      return 0
    fi

    echo "Waiting for first message on ${topic} (${attempt}/${max_attempts})"
    attempt=$((attempt + 1))
    sleep "${sleep_seconds}"
  done

  echo "Timed out waiting for first message on ${topic}" >&2
  return 1
}

wait_for_first_message "${YOLO_LIDAR_TOPIC:-/velodyne_points}"
wait_for_first_message "${YOLO_CAMERA_INFO_TOPIC:-/zed/zed_node/rgb/color/rect/camera_info}"
wait_for_first_message "${YOLO_IMAGE_TOPIC:-/zed/zed_node/rgb/color/rect/image}"

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
