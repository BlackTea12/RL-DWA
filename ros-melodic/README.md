# HOW TO START
_Summary: launch testfield dwa, vehicle(Tracer), lidar(rslidar:BPearl)_

## :one: Preinstallation & Dependencies

### 1. Hardware

- **ugv_sdk** :point_right: [link](https://github.com/agilexrobotics/ugv_sdk.git)

- **tracer_ros** :point_right: [link](https://github.com/agilexrobotics/tracer_ros.git)

- **rslidar** :point_right: [link](https://github.com/RoboSense-LiDAR/rslidar_sdk.git)

Please keep in mind that the following links are the main essentials github links. There are more essential dependencies that are refered in the following link. Here, **BPearl Lidar** and **Tracer** is used for the mobile robot.

### 2. System

- **point cloud helper library reference** :point_right: [link](https://github.com/mithi/point-cloud-clusters.git)


## :two: catkin workspace

rslidar_sdk :point_left: config.yaml, start.launch

point cloud helper library reference :point_left: rslidar_detection2d.py

to be updated...

## :three: Example

to be updated...

    $ roslaunch rslidar_sdk start.launch
    $ python3 rslidar_detection2d.py
    $ python dwa_algorithm.py
