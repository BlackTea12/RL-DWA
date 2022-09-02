# HOW TO START
_Summary: launch testfield dwa, vehicle(Tracer), lidar(rslidar:BPearl)_

### :one: Preinstallation & Dependencies

to be updated...

point cloud helper library reference [link](https://github.com/mithi/point-cloud-clusters.git)

### :two: catkin workspace

to be updated...

### :three: Example

to be updated...

    $ roslaunch rslidar_sdk start.launch
    $ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 world map 100
    $ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map rslidar 100
    $ python3 rslidar_detection2d.py
