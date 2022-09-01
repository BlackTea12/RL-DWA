roslaunch rslidar_sdk start.launch
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map world 100
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 rslidar map 100

python3 rslidar_detection2d.py
