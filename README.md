# YOLO-LiDAR Fusion
## 1. Overview
This repository contains the code produced during my [Master's Thesis](https://github.com/TimKie/YOLO-LiDAR-Fusion/blob/fad44530dcfa172145f67ce0469d7b29bcd02bda/Master_Thesis_compressed.pdf) in collaboration with the UBIX research group of the University of Luxembourg’s Interdisciplinary Centre for Security, Reliability, and Trust (SnT).
This thesis aimed to develop a resource-efficient model for 3D object detection utilizing LiDAR and camera sensors, tailored for autonomous vehicles with limited computational resources. An overview of the model is shown in the figure below.

![Model_Overview](assets/model_overview.svg)

![Mode_Demo](assets/model_demo.gif)

## 2. Prerequisites
### Hardware
- Ideally: NVIDIA GPU such that the YOLOv8 model can be run with CUDA

_Note:_ The model can also be run on the CPU (slower). 

### Software
- Python version between 3.8 and 3.11 (due to open3d requirements)

_Note:_ The code was developed and tested on Python 3.10. 

## 3. Setup
Follow the steps below to set up the environment:

1. Go to the directory of your choice and clone the repository:

    ```shell
    git clone https://github.com/TimKie/YOLO-LiDAR-Fusion.git
    ```

2. Get into the working directory (root directory of the repository):

    ```shell
    cd YOLO-LiDAR-Fusion
    ```

3. Optionally create and start a virtual environment:

    ```python
    python -m venv venv
    source venv/bin/activate
    ```

4. Install required libraries:

    ```python
    pip install -r requirements.txt
    ```

5. For live ROS processing, run the code in an environment that already has ROS 2 Humble Python packages available (`rclpy`, `sensor_msgs`, `cv_bridge`) in addition to the Python requirements above.

   _Note:_ The lightweight Docker image in `docker/` is enough for KITTI-style offline inference, but live ROS topic processing needs a ROS-enabled runtime.

## 4. Usage
Follow the steps below to use the model:

_Note:_ Make sure that the file structure is as stated below in [File Structure](#7-file-structure). 

1. Go to the directory where the implementation is located:

   ```shell
   cd Code
   ```

2. The simplest command to process a single frame from the **KITTI_dataset** directory is as follows (where _image_index_ is a 6-digit number between '000000' and '007517'). This will display the processed frame as well as the ground truth (GT) bounding boxes (labels) of KITTI and print the best and average IoU scores between the predicted and GT bounding boxes for each object class in the frame:

   ```shell
   python main.py image_index
   ```

3. Some optional parameters that can be specified in the command are shown below:

   ```shell
   python main.py image_index --mode --model-size --erosion --depth --pca --dataset-path --output-path
   ```

   The parameters can take the following values:
   - **image_index**:
      - number of the image (between '000000' and '007517')
      - 'random' for multiple random images
      - 'evaluation' to process the complete dataset
      - 'video' to process raw data and create a processed video
      - 'ros' to process live ROS image and LiDAR topics
    
   - **--mode**:
      - detect (model only detects objects) (default)
      - track (model detects and tracks objects)
    
   - **--model-size**: specifies the YOLOv8 model size that is used (n, s, m, l, x) (default: m)
     
   - **--erosion**: specifies the amount of erosion used by the model (smaller value --> higher erosion) (default: 25)
   
   - **--depth**: specifies the depth filter factor used by the model (smaller value --> more aggressive filtering) (default: 20)
  
   - **--pca**: specifies whether PCA should be used to create the 3D bounding boxes for all detected objects (default: False)
  
   - **--dataset-path**: specifies the relative path to the KITTI dataset (default: '../KITTI_dataset/')
  
   - **--output-path**: specifies the relative path where the output should be saved

- If _image_index_ is set to 'ros', the following optional parameters can be specified:
   - **--fusion-config-path**: path to the `vlp16zed2ifusion` config file used to load the default live topics and LiDAR-to-camera extrinsic defaults
   - **--camera-calibration-file**: optional camera calibration YAML file; if omitted, the model uses the live `CameraInfo` topic for intrinsics
   - **--image-topic**: override the live camera image topic
   - **--camera-info-topic**: override the live camera info topic
   - **--lidar-topic**: override the live LiDAR topic
   - **--output-topic**: publish the masked projected image to a ROS topic (default: `/yolo_lidar_fusion/masked_projected_image`)
   - **--max-cloud-age**: maximum allowed timestamp difference between the image and LiDAR messages
   - **--no-display**: disable the OpenCV display window

- If _image_index_ is a 6-digit number between '000000' and '007517':
    - if **--output-path** is not specified: processed image will only be displayed
    - if **--output-path** is specified: processed image will be stored in the directory passed as a parameter

- If _image_index_ is set to 'random', the following parameter has to be specified:
   - **--image-amount**: specifies the desired amount of random images (default: 10)
  
- If _image_index_ is set to 'video', the following parameter has to be specified:
   - **--video-dir**: specifies the relative path of the directory that contains the ordered frames of the video that the model should process (default: '../KITTI_raw_data')

### Live ROS Mode
The repository now supports a live ROS path through `python main.py ros`. In this mode:

- YOLO runs on the camera image topic
- the LiDAR point cloud is read from a live `PointCloud2` topic
- the camera intrinsics come from the live `CameraInfo` topic by default
- the default image topic, camera info topic, LiDAR topic, and extrinsic defaults are loaded from the sibling
  `ws_vlp16zed2ifusion` workspace config:

  `../ws_vlp16zed2ifusion/src/vlp16zed2ifusion/config/fusion_overlay_params.yaml`

- only the LiDAR points that survive the segmentation-mask filtering are projected back onto the image

The default live topics are:

- image: `/zed/zed_node/rgb/color/rect/image`
- camera info: `/zed/zed_node/rgb/color/rect/camera_info`
- LiDAR: `/velodyne_points`
- output image: `/yolo_lidar_fusion/masked_projected_image`

## 5. Usage Examples:
- Display the detection results of image '000010' from the KITTI dataset with an erosion factor of 15 and a depth factor of 30:
  
      python main.py 000010 --erosion 15 --depth 30
      
- Use the biggest YOLOv8 model (x) and use PCA to create the bounding boxes for image '000032' from the KITTI dataset and save the result in the directory '../Model_Output':

      python main.py 000032 --model-size x --pca True --output-path ../Model_Output

- Process consecutive frames from the raw data section of the KITTI dataset website which are stored in the directory 'KITTI_raw_data' by using the small YOLOv8 model size, tracking and PCA. The processed video is stored at the default output directory './Model_Output':

      python main.py video --video-dir ./KITTI_raw_data --mode track --model-size s --pca True

- Process 5 random images from the 'KITTI_dataset' directory (default) by using an erosion factor of 15, a depth factor of 20 and the smallest YOLOv8 model size (n). The detection results are stored in the default output directory './Model_Output':

      python main.py random --image-amount 5 --depth 20 --erosion 15 --model-size n

- Run the live ROS pipeline with the default topics loaded from `vlp16zed2ifusion`:

      cd Code
      source /opt/ros/humble/setup.bash
      python main.py ros

- Run the live ROS pipeline while explicitly overriding the topics:

      python main.py ros --image-topic /zed/zed_node/rgb/color/rect/image --camera-info-topic /zed/zed_node/rgb/color/rect/camera_info --lidar-topic /velodyne_points --output-topic /yolo_lidar_fusion/masked_projected_image

## 6. Live ROS With ws_vlp16zed2ifusion
If the ZED and Velodyne topics are already being published from the sibling workspace in the
`segmented_mask_depth` parent repo, either through:

- `../ws_vlp16zed2ifusion/docker/run.sh`, or
- `../docker-compose.yml`

then this repository can subscribe to those topics from a second container or terminal session as long as:

- both containers use `--network host`
- both containers use the same `ROS_DOMAIN_ID`
- `ROS_LOCALHOST_ONLY` is not preventing DDS discovery across the containers
- the YOLO environment contains ROS 2 Humble Python packages and OpenCV

Recommended launch order:

1. Start the ZED + Velodyne publisher container first.
2. Wait until `/zed/zed_node/rgb/color/rect/image`, `/zed/zed_node/rgb/color/rect/camera_info`, and `/velodyne_points` are available.
3. Start the YOLO-LiDAR-Fusion container or shell and run `python main.py ros`.

Creating a Docker Compose file at the parent repo level does make sense if both workspaces are
meant to run together regularly.
It gives you:

- one shared place to define `--network host`, GPU access, X11 mounts, and environment variables
- repeatable startup for both containers
- a cleaner way to keep both workspaces mounted and versioned together

Compose is especially useful here because the `ws_vlp16zed2ifusion` container acts as the sensor publisher and this repository acts as the consumer.

In the provided Compose setup, the publisher service is built from:

`ws_vlp16zed2ifusion/docker/Dockerfile`

and the YOLO live ROS image is built from:

`external/YOLO-LiDAR-Fusion/docker/Dockerfile.live_ros`

The repository root-level Compose file is:

`./docker-compose.yml` from the parent repo root

Typical startup:

```shell
xhost +local:root
PARENT_REPO_ROOT="$(cd ../.. && pwd)"
cd "${PARENT_REPO_ROOT}"
./start_docker_compose_stack.sh
```

If you want to run the stack without the YOLO OpenCV display window:

```shell
PARENT_REPO_ROOT="$(cd ../.. && pwd)"
cd "${PARENT_REPO_ROOT}"
YOLO_DISPLAY=0 ./start_docker_compose_stack.sh
```

## 7. File Structure
The file structure is important to use the model without modifying the dataset paths in the main.py file. It should be as follows:

_Notes:_ 

The **KITTI_dataset** directory contains the training dataset of KITTI that can be downloaded [here](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d).

The **KITTI_raw_data** directory contains raw data of consecutive frames (for video inference) of the KITTI dataset that can be downloaded [here](https://www.cvlibs.net/datasets/kitti/raw_data.php).
   
    .
    ├── assets
    │   └── ...
    ├── Code
    │   ├── calibration.py
    │   ├── data_processing.py
    │   ├── detector.py
    │   ├── evaluation.py
    │   ├── fusion.py
    │   ├── main.py
    │   ├── utils.py
    │   └── visualization.py
    ├── KITTI_dataset
    │   ├── data_object_calib
    │   │   └── training
    │   │       └── calib
    │   │           └── ...
    │   ├── data_object_image_2
    │   │   └── training
    │   │       └── image_2
    │   │           └── ...
    │   ├── data_object_label_2
    │   │   └── training
    │   │       └── label_2
    │   │           └── ...
    │   └── data_object_velodyne
    │       └── training
    │           └── velodyne
    │               └── ...
    ├── KITTI_raw_data
    │   ├── calib
    │   │   ├── calib_cam_to_cam.txt
    │   │   ├── calib_imu_to_velo.txt
    │   │   └── calib_velo_to_cam.txt
    │   ├── image_02
    │   │   ├── data
    │   │   │   └── ...
    │   │   └── timestamps.txt
    │   └── velodyne_points
    │       ├── data
    │       │   └── ...
    │       └── timestamps.txt
    └── requirements.txt


Notes created during testing this repo:
Requirments:
   ubuntu/python:3.10-22.04_stable
   - The point cloud however is filtered by a function such that only the points which
lie inside the field of view of the image are kept, while all the others discarded
   - First, the image boundaries are defined, and then the
3D LiDAR points are converted into 2D image coordinates by a function that uses the
calibration matrices between the LiDAR and camera sensors
