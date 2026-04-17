#!/bin/bash

# Launch script for Docker container

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
KITTI_DATASET_DIR="${KITTI_DATASET_DIR:-${REPO_ROOT}/KITTI_dataset}"
KITTI_RAW_DATA_DIR="${KITTI_RAW_DATA_DIR:-${REPO_ROOT}/KITTI_raw_data}"

IMAGE_NAME="yolo-lidar-fusion"
IMAGE_TAG="latest"
CONTAINER_NAME="yolo-lidar-fusion-container"
XAUTH_FILE="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

echo "Launching Docker container from ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Using KITTI dataset directory: ${KITTI_DATASET_DIR}"
echo "Using KITTI raw data directory: ${KITTI_RAW_DATA_DIR}"

xhost +SI:localuser:root >/dev/null 2>&1

# Remove existing container if it exists
docker rm -f ${CONTAINER_NAME} 2>/dev/null

docker run -it \
    --name ${CONTAINER_NAME} \
    -v "${REPO_ROOT}:/app/YOLO-LiDAR-Fusion" \
    -v "${KITTI_DATASET_DIR}:/app/YOLO-LiDAR-Fusion/KITTI_dataset" \
    -v "${KITTI_RAW_DATA_DIR}:/app/YOLO-LiDAR-Fusion/KITTI_raw_data" \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTH_FILE \
    -v "$XAUTH_FILE:$XAUTH_FILE:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    ${IMAGE_NAME}:${IMAGE_TAG}

if [ $? -eq 0 ]; then
    echo "✓ Container launched successfully!"
else
    echo "✗ Container launch failed!"
    exit 1
fi
