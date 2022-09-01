#/usr/bin/python3 

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import time
from sensor_msgs import point_cloud2
import pcl
import numpy as np
from numpy import cos
from numpy import sin

#ROI
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# downsampling
# def do_voxel_grid_downssampling(pcl_data,leaf_size):
#     vox = pcl_data.make_voxel_grid_filter()
#     vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
#     return  vox.filter()

class lidar_test:
    def __init__(self):
        rospy.init_node("get_bpearl")
        # rate = rospy.Rate(5)
        # self.marker = Marker()
        # self.init_visual()
        rospy.Subscriber("rslidar_points", PointCloud2, self.pointcloudCallback)

        self.visual_pub = rospy.Publisher("visual_publish", Marker, queue_size=1)
        self.pub = rospy.Publisher("point_publish", PointCloud, queue_size=1)
        
        rospy.spin()

    def init_visual(self):
        self.marker.header.frame_id = "/rslidar"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # marker scale
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02

        # marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        # marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        # marker line points
        self.marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = -2.848
        first_line_point.y = -2.848
        first_line_point.z = 0.0
        self.marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = -5.696
        second_line_point.y = 0.0
        second_line_point.z = 0.0
        self.marker.points.append(second_line_point)
        # third point
        third_line_point = Point()
        third_line_point.x = 0.0
        third_line_point.y = 5.696
        third_line_point.z = 0.0
        self.marker.points.append(third_line_point)
        # fourth point
        fourth_line_point = Point()
        fourth_line_point.x = 2.848
        fourth_line_point.y = 2.848
        fourth_line_point.z = 0.0
        self.marker.points.append(fourth_line_point)
        # fifth point
        fifth_line_point = Point()
        fifth_line_point.x = -2.848
        fifth_line_point.y = -2.848
        fifth_line_point.z = 0.0
        self.marker.points.append(fifth_line_point)

    def pointcloudCallback(self, msg):
        start = time.time()
        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z", "intensity")))
        # print("Type : ", type(cloud_points), cloud_points[0])
        start = time.time()
        get_processing_list = list()

        for i in range(len(cloud_points)):
            temp_list = list()
            # temp_list.append(sin(np.deg2rad(-3)) * cloud_points[i][0] + cos(np.deg2rad(-3)) * cloud_points[i][2] - 0.73)
            # temp_list.append(-cloud_points[i][1] + 1.8)
            # temp_list.append(-sin(np.deg2rad(-3)) * cloud_points[i][2] + cos(np.deg2rad(-3)) * cloud_points[i][0] + 1.52)
            # temp_list.append(cloud_points[i][3])
            temp_list.append(cloud_points[i][0])
            temp_list.append(cloud_points[i][1])
            temp_list.append(0.0) #cloud_points[i][2])
            temp_list.append(cloud_points[i][3])

            get_processing_list.append(temp_list)
            
        #pre-processing
        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(cloud_points)
        # print("Input :", pcl_data)



        # LEAF_SIZE = 0.005
        # cloud = do_voxel_grid_downssampling(pcl_data, LEAF_SIZE)
        # print("Output :", cloud)

        cloud = pcl_data

        filter_axis = 'x'
        axis_min = -6.0
        axis_max = 3.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        filter_axis = 'y'
        axis_min = -3.0
        axis_max = 6.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        # filter_axis = 'z'
        # axis_min = 0.0
        # axis_max = 0.0
        # cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        # print("Output :", cloud)

        test = PointCloud()
        get_in = ChannelFloat32()
        get_in.name = 'Bpearl_intensity'
        test.points = []
        end = time.time()
        # print(end - start)
        for p in cloud:
            point_temp = Point32()
            point_temp.x = p[0]
            point_temp.y = p[1]
            point_temp.z = 0.0# p[2]
            get_in.values.append(p[3])
            test.points.append(point_temp)

        test.channels.append(get_in)
        test.header.frame_id = 'rslidar'
        test.header.stamp = rospy.Time.now()

        self.pub.publish(test)
        # self.visual_pub.publish(self.marker)

if __name__ == "__main__":
    lidar_test()
