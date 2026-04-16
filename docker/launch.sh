#!/bin/bash

# Launch script for Docker container

IMAGE_NAME="yolo-lidar-fusion"
IMAGE_TAG="latest"
CONTAINER_NAME="yolo-lidar-fusion-container"
XAUTH_FILE="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

echo "Launching Docker container from ${IMAGE_NAME}:${IMAGE_TAG}"

xhost +SI:localuser:root >/dev/null 2>&1

# Remove existing container if it exists
docker rm -f ${CONTAINER_NAME} 2>/dev/null

docker run -it \
    --name ${CONTAINER_NAME} \
    -v "/media/gvb/ssd24ubuntu/robotspace2/3DRecon/ws_YOLO-LiDARFusion/YOLO-LiDAR-Fusion:/app/YOLO-LiDAR-Fusion" \
    -v "/media/gvb/ssd24ubuntu/robotspace2/Datasets/kitti_object detection_vlplidar_camera/KITTI_dataset:/app/YOLO-LiDAR-Fusion/KITTI_dataset" \
    -v "/media/gvb/ssd24ubuntu/robotspace2/Datasets/kitti_object detection_vlplidar_camera/KITTI_raw_data:/app/YOLO-LiDAR-Fusion/KITTI_raw_data" \
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
