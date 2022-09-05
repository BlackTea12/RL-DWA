# HOW TO START
_Summary: launch testfield dwa, vehicle(Tracer), lidar(rslidar:BPearl)_

## :one: Preinstallation & Dependencies

### 1. Hardware

- **ugv_sdk** :point_right: [link](https://github.com/agilexrobotics/ugv_sdk.git)

- **tracer_ros** :point_right: [link](https://github.com/agilexrobotics/tracer_ros.git)

- **rslidar** :point_right: [link](https://github.com/RoboSense-LiDAR/rslidar_sdk.git)

Please keep in mind that the following links are the main essentials github links. There are more essential dependencies that are referred in the following link. Here, **BPearl Lidar** and **Tracer** is used for the mobile robot.

### 2. System

- **point cloud helper library reference** :point_right: [link](https://github.com/mithi/point-cloud-clusters.git)

As **RSLidar(BPear)** is used as a sensor for obstacle detection, pointclouds data process with clustering is needed. Related algorithm and libraries are referred in the following link. For sensor application in custom environment, sensor tuning (ex., clustering min/max pointcloud numbers, detection range) is needed.


## :two: catkin workspace

### 1. Tracer PC (ROBOT SIDE)

_Related Files: [tracer pc folder](https://github.com/BlackTea12/RL-DWA/tree/main/ros-melodic/tracer%20pc)_

### :gem: **rslidar_sdk repository**

- config.yaml

    > Configuration file of RSLidar. We use BPearl Lidar, so the line 'lidar_type:' should be written as 'RSBP'. We have also changed the pointcloud range to desired range of 180 degree field of view only (start_angle ~ end_angle). Our example image is in [here](https://github.com/BlackTea12/RL-DWA/blob/main/img/range_limited_rspb.png) where limited view of Lidar in rviz is checked.

- start.launch
    
    > This launch file activates input of pointclouds from the lidar to ros pointcloud messages.
    
### :gem: **dwa custom python script**

- rslidar_detection2d.py

    > After launching lidar ros pointcloud messages, this script subscribes to the messages and process the data to 2D obstacle detection. Obstacle information is published by **geometry_msgs.msg/PoseArray** data type.

- dwa_algorithm.py

    > After executing rslidar_detection2d.py, this script **subsribes to obstacle information** and **publish /cmd_vel** (data type: geometry_msgs.msg/Twist) for Tracer.

### 2. RL PC (REINFORCEMENT LEARNING SIDE)

_Related Files: [rl pc folder](https://github.com/BlackTea12/RL-DWA/tree/main/ros-melodic/rl%20pc)_

### :gem: **/models**

Save your pretrained models here. Example models for adaptive predictive length dynamic window approach (APL-DWA) and adaptive window approach (ADWA) is included. You can check an example image result of system information in command terminal [here](https://github.com/BlackTea12/RL-DWA/blob/main/img/(example)%20apl-dwa%20sys%20info.png) when reinforcement learning based dynamic window approach algorithm is executed.

### :gem: **/scripts**

After training our model, an zip file is made in the folder models (follow training [here](https://github.com/BlackTea12/RL-DWA)). In **[dwa_predict.py](https://github.com/BlackTea12/RL-DWA/blob/main/ros-melodic/rl%20pc/scripts/dwa_predict.py)** script, pretrained model is loaded and action space values are sent as ros **std_msgs.msg/Float64MultiArray** data type. 

Tracer PC and RL PC are in the same wifi and the primary PC is set as RL PC. Therefore, Tracer PC will subscribe to the message (action space values) sent by RL PC. Subscribing script line is in [dwa_algorithm.py](https://github.com/BlackTea12/RL-DWA/blob/main/ros-melodic/tracer%20pc/dwa_algorithm.py)

For the observation space value, RL PC subscribes the Tracer odometry value which is published by **'roslaunch tracer_base tracer_base.launch'** in **rostopic _/odom_**.

## :three: Example

Here are the following execution order

:arrow_double_down: **RL PC**

    $ roscore    # start primary pc roscore
    $ python3 dwa_predict.py    # start reinforcement learning prediction
    
:arrow_double_down: **Tracer PC**

    $ roslaunch rslidar_sdk start.launch    # start lidar
    $ python3 rslidar_detection2d.py    # process lidar pointclouds into user data
    $ python dwa_algorithm.py   #activate
