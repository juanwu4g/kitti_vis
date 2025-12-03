# KITTI Dataset Visualization with ROS

A ROS-based visualization system for the KITTI dataset that publishes real-time sensor data (camera images, LiDAR point clouds, IMU, GPS) and 3D object tracking information for visualization in RViz.

## Features

- **Multi-Modal Sensor Visualization**: Camera images, LiDAR point clouds, IMU, and GPS data
- **3D Object Tracking**: Real-time tracking and visualization of cars, trucks, vans, trams, pedestrians, and cyclists
- **3D Bounding Boxes**: Accurate 3D bounding boxes rendered in camera and LiDAR coordinate frames
- **Object Trajectory Tracking**: Historical trajectory visualization for tracked objects
- **Distance Computation**: Real-time calculation of minimum distance between ego vehicle and detected objects
- **Ego Vehicle Visualization**: 3D model and field-of-view markers for the ego vehicle
- **Coordinate Frame Transformations**: Full KITTI calibration support for multi-frame conversions
- **Dockerized Environment**: Easy setup with all dependencies pre-configured

## Demo

![KITTI Visualization Demo](/demo.gif)

## Prerequisites

- **Docker** and **Docker Compose**
- **KITTI Dataset** (see [Dataset Setup](#dataset-setup) below)
- **X Server** (for GUI display on Linux)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/kitti_vis.git
cd kitti_vis
```

### 2. Dataset Setup

Download the KITTI dataset:

1. **Raw Data**: Download the raw data sequences from [KITTI Raw Data](http://www.cvlibs.net/datasets/kitti/raw_data.php)
   - Required: `2011_09_26_drive_0005_sync` (Camera, Velodyne, IMU/GPS data)
   - Place in: `data/kitti/RawData/2011_09_26/`

2. **Tracking Labels**: Download tracking labels from [KITTI Tracking](http://www.cvlibs.net/datasets/kitti/eval_tracking.php)
   - Required: Training labels
   - Place in: `data/kitti/tracking/training/label_02/`

Expected directory structure:
```
data/kitti/
├── RawData/
│   └── 2011_09_26/
│       └── 2011_09_26_drive_0005_sync/
│           ├── image_00/
│           ├── image_01/
│           ├── image_02/
│           ├── image_03/
│           ├── velodyne_points/
│           └── oxts/
└── tracking/
    └── training/
        └── label_02/
            ├── 0000.txt
            ├── 0001.txt
            └── ...
```

### 3. Build Docker Container

```bash
docker-compose up --build
```

This will:
- Build a ROS Noetic container with all dependencies
- Mount your project directory and data directory
- Set up the catkin workspace

## Usage

### 1. Start the Docker Container

```bash
docker-compose up
```

### 2. Enter the Container

In a new terminal:
```bash
docker exec -it kitti_vis bash
```

### 3. Build the ROS Package

```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Launch ROS Core

```bash
roscore
```

### 5. Run the KITTI Publisher Node

In a new terminal (inside the container):
```bash
source /catkin_ws/devel/setup.bash
rosrun camera_pub kitti.py
```

### 6. Visualize in RViz

In another terminal (inside the container):
```bash
rviz
```

**RViz Configuration:**
- Set **Fixed Frame** to `map`
- Add displays for the following topics:
  - `/kitti_cam` (Image)
  - `/kitti_point_cloud` (PointCloud2)
  - `/kitti_3d_boxes` (MarkerArray)
  - `/ego_car_marker` (MarkerArray)
  - `/kitti_loc` (MarkerArray) - Object trajectories
  - `/kitti_dist` (MarkerArray) - Distance markers
  - `/kitti_imu` (Imu)

## Project Structure

```
kitti_vis/
├── catkin_ws/
│   └── src/
│       ├── camera_pub/              # Main ROS package
│       │   ├── src/
│       │   │   ├── kitti.py         # Main ROS node
│       │   │   ├── publish_utils.py # ROS message publishers
│       │   │   ├── kitti_util.py    # Calibration & transformations
│       │   │   ├── data_utils.py    # Data loading utilities
│       │   │   └── misc.py          # Miscellaneous utilities
│       │   ├── models/
│       │   │   └── audi_r8.dae      # Ego vehicle 3D model
│       │   ├── package.xml
│       │   └── CMakeLists.txt
│       ├── plot2d.ipynb             # 2D visualization notebook
│       └── plot3d.ipynb             # 3D visualization notebook
├── data/                            # KITTI dataset (not in repo)
├── Dockerfile
├── docker-compose.yml
└── README.md
```

## ROS Topics Published

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/kitti_cam` | `sensor_msgs/Image` | Camera feed with 2D bounding boxes |
| `/kitti_point_cloud` | `sensor_msgs/PointCloud2` | LiDAR point cloud data |
| `/ego_car_marker` | `visualization_msgs/MarkerArray` | Ego vehicle model and FOV lines |
| `/kitti_3d_boxes` | `visualization_msgs/MarkerArray` | 3D bounding boxes for tracked objects |
| `/kitti_loc` | `visualization_msgs/MarkerArray` | Object trajectory paths |
| `/kitti_dist` | `visualization_msgs/MarkerArray` | Minimum distance labels |
| `/kitti_imu` | `sensor_msgs/Imu` | IMU orientation and acceleration |
| `/kitti_gps` | `sensor_msgs/NavSatFix` | GPS coordinates |

**Note**: GPS data cannot be directly visualized in RViz but can be logged for analysis.

## Technical Details

### Coordinate Frames

The system handles multiple KITTI coordinate frames:
- **Velodyne (LiDAR)**: Raw point cloud coordinate system
- **Reference Camera**: Unrectified camera coordinate system
- **Rectified Camera**: Rectified camera coordinate system (aligned with camera 0)
- **Image**: 2D image pixel coordinates

All transformations use KITTI calibration files for accurate multi-sensor fusion.

### Object Tracking

- Tracks 6 object types: **Car**, **Truck**, **Van**, **Tram**, **Pedestrian**, **Cyclist**
- Maintains trajectory history (up to 20 positions per object)
- Compensates for ego vehicle motion using IMU yaw data
- Computes minimum 2D distance between object and ego vehicle bounding boxes

### Publish Rate

Default: **10 Hz** (adjustable in `kitti.py`)

## Jupyter Notebooks

- **plot2d.ipynb**: Analyze and visualize 2D bounding boxes
- **plot3d.ipynb**: Analyze and visualize 3D bounding boxes and transformations

Run notebooks inside the Docker container:
```bash
jupyter notebook --ip=0.0.0.0 --allow-root
```

## Troubleshooting

### Docker Display Issues

If you encounter display errors when running RViz:
```bash
xhost +local:docker
```

### ROS Topic Not Showing

Verify the node is publishing:
```bash
rostopic list
rostopic echo /kitti_cam
```

### Build Errors

Ensure all dependencies are installed:
```bash
cd /catkin_ws
catkin_make clean
catkin_make
```
