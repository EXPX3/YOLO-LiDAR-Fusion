#!/bin/bash

# Build script for Docker image

IMAGE_NAME="yolo-lidar-fusion"
IMAGE_TAG="latest"

echo "Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}"

# Must run build from docker directory, copy requirements first or build from parent
cd ..
docker build --network host -f docker/Dockerfile -t ${IMAGE_NAME}:${IMAGE_TAG} .
BUILD_RESULT=$?
cd docker

if [ $BUILD_RESULT -eq 0 ]; then
    echo "✓ Docker image built successfully!"
else
    echo "✗ Docker build failed!"
    exit 1
fi
