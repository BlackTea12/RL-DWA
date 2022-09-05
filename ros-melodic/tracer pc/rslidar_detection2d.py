#/usr/bin/python3 

import sys
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32, Point
from geometry_msgs.msg import PoseArray, Pose, Quaternion
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# import time
from sensor_msgs import point_cloud2
import pcl
import numpy as np

# from numpy import cos
# from numpy import sin
import math

def pcl_to_ros(pcl_array):
    """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message
    
        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
            
        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "world"

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    ros_msg.data = "".join(buffer)

    return ros_msg

#ROI
def do_passthrough(point_cloud, name_axis, min_axis, max_axis):
    passthrough = point_cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(name_axis)
    passthrough.set_filter_limits(min_axis, max_axis)
    return passthrough.filter()

# downsampling
def do_voxel_grid_filter(point_cloud, LEAF_SIZE):
    vox = point_cloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) # The bigger the leaf size the less information retained
    return  vox.filter()

# This pipeline separates the objects in the table from the given scene
def split_cloud(cloud):

  # Downsample the cloud as high resolution which comes with a computation cost
  downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)

  # Get only information in our region of interest as we don't care about the other parts
  filtered_cloud = do_passthrough(point_cloud = downsampled_cloud, 
                                         name_axis = 'z', min_axis = -0.25, max_axis = 0.25)

  # Separate the table from everything else
  table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.01)

  return objects_cloud, table_cloud

def cluster_mask(cluster_indices, white_cloud):
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                            white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float( cluster_color[j] )
                                           ])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud

def get_clusters(cloud, tolerance, min_size, max_size):

  tree = cloud.make_kdtree()
  extraction_object = cloud.make_EuclideanClusterExtraction()

  extraction_object.set_ClusterTolerance(tolerance)
  extraction_object.set_MinClusterSize(min_size)
  extraction_object.set_MaxClusterSize(max_size)
  extraction_object.set_SearchMethod(tree)

  # Get clusters of indices for each cluster of points, each clusterbelongs to the same object
  # 'clusters' is effectively a list of lists, with each list containing indices of the cloud
  clusters = extraction_object.Extract()
  return clusters

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def XYZRGB_to_XYZ(XYZRGB_cloud):
    """ Converts a PCL XYZRGB point cloud to an XYZ point cloud (removes color info)
    
        Args:
            XYZRGB_cloud (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
            
        Returns:
            PointCloud_PointXYZ: A PCL XYZ point cloud
    """
    XYZ_cloud = pcl.PointCloud()
    points_list = []

    for data in XYZRGB_cloud:
        points_list.append([data[0], data[1], data[2]])

    XYZ_cloud.from_list(points_list)
    return XYZ_cloud

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_plane_segmentation(point_cloud, max_distance = 0.01):

    segmenter = point_cloud.make_segmenter()

    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(max_distance)

    #obtain inlier indices and model coefficients
    inlier_indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(inlier_indices, negative = False)
    outliers = point_cloud.extract(inlier_indices, negative = True)

    return inliers, outliers

class lidar_test:
    def __init__(self):
        rospy.init_node("get_bpearl")
        # rate = rospy.Rate(5)
        # self.marker = Marker()
        # self.init_visual()
        rospy.Subscriber("rslidar_points", PointCloud2, self.pointcloudCallback)

        # self.visual_pub = rospy.Publisher("visual_publish", Marker, queue_size=1)
        # self.pub = rospy.Publisher("point_publish", PointCloud, queue_size=1)
        self.detected_objects_publisher = rospy.Publisher("/detected_objects/point", PointCloud, queue_size=1)
        # self.object_markers_publisher = rospy.Publisher("/detected_objects/Marker", Marker, queue_size=1)
        self.pub_obs = rospy.Publisher("/realsense/obstacles", PoseArray, queue_size=1)
        self.poses = PoseArray()
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

    # def pointcloudCallback(self, msg):
    #     start = time.time()
    #     cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z", "intensity")))
    #     # print("Type : ", type(cloud_points), cloud_points[0])
    #     start = time.time()
    #     get_processing_list = list()

    #     for i in range(len(cloud_points)):
    #         temp_list = list()
    #         # temp_list.append(sin(np.deg2rad(-3)) * cloud_points[i][0] + cos(np.deg2rad(-3)) * cloud_points[i][2] - 0.73)
    #         # temp_list.append(-cloud_points[i][1] + 1.8)
    #         # temp_list.append(-sin(np.deg2rad(-3)) * cloud_points[i][2] + cos(np.deg2rad(-3)) * cloud_points[i][0] + 1.52)
    #         # temp_list.append(cloud_points[i][3])
    #         temp_list.append(cloud_points[i][0])
    #         temp_list.append(cloud_points[i][1])
    #         temp_list.append(cloud_points[i][2]) #cloud_points[i][2])
    #         temp_list.append(cloud_points[i][3])

    #         get_processing_list.append(temp_list)
            
    #     #pre-processing
    #     pcl_data = pcl.PointCloud_PointXYZRGB()
    #     pcl_data.from_list(cloud_points)
    #     # print("Input :", pcl_data)



    #     # LEAF_SIZE = 0.005
    #     # cloud = do_voxel_grid_downssampling(pcl_data, LEAF_SIZE)
    #     # print("Output :", cloud)

    #     cloud = pcl_data

    #     filter_axis = 'x'
    #     axis_min = -6.0
    #     axis_max = 3.0
    #     cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    #     filter_axis = 'y'
    #     axis_min = -3.0
    #     axis_max = 6.0
    #     cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    #     # filter_axis = 'z'
    #     # axis_min = 0.0
    #     # axis_max = 0.0
    #     # cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    #     # print("Output :", cloud)

    #     test = PointCloud()
    #     get_in = ChannelFloat32()
    #     get_in.name = 'Bpearl_intensity'
    #     test.points = []
    #     end = time.time()
    #     # print(end - start)
    #     cnt = 0
    #     for p in cloud:
    #         point_temp = Point32()
    #         point_temp.x = p[0]
    #         point_temp.y = p[1]
    #         point_temp.z = 0.0# p[2]
    #         get_in.values.append(p[3])
    #         test.points.append(point_temp)
    #         # cnt += 1

    #         dist = math.sqrt(p[0]**2 + p[1]**2)
    #         if dist < 3.0:
    #             cnt += 1
    #             # print("obj detected %f", round(dist,3))
    #     print(cnt)
    #     test.channels.append(get_in)
    #     test.header.frame_id = 'rslidar'
    #     test.header.stamp = rospy.Time.now()

    #     self.pub.publish(test)
    #     # self.visual_pub.publish(self.marker)

    def pointcloudCallback(self, msg):
        # ROS msg -> PCL data
        # cloud = ros_to_pcl(msg)
        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z", "intensity")))

        #pre-processing
        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(cloud_points)

        # Extract objects and floor from view
        objects_cloud, table_cloud = split_cloud(pcl_data)
        
         # Get a point cloud of only the position information without color information
        colorless_cloud = XYZRGB_to_XYZ(objects_cloud)

        # Get groups of indices for each cluster of points
        # Each group of points belongs to the same object
        # This is effectively a list of lists, with each list containing indices of the cloud
        clusters = get_clusters(colorless_cloud, tolerance = 0.05, min_size = 10, max_size = 500)

        # CLASSIFY THE CLUSTERS 
        detected_objects_labels = []
        detected_objects = []
        test = PointCloud()
        get_in = ChannelFloat32()
        get_in.name = 'Bpearl_intensity'
        test.points = []

        # distance and x, y
        self.poses = PoseArray()
        for i, indices in enumerate(clusters):
            Limit_Detection = 3.1 # meters
            check_qualification = False
            cluster = objects_cloud.extract(indices)

            for p in cluster:
                point_temp = Point32()
                point_temp.x = p[0]
                point_temp.y = p[1]
                point_temp.z = p[2]
                get_in.values.append(p[3])
                test.points.append(point_temp)

                tempD = math.sqrt(p[0]**2 + p[1] **2)
                if tempD < Limit_Detection: # saving for min dist
                    Limit_Detection = tempD
                    check_qualification = True
                    po = Pose()
                    po.position = Point(p[0], p[1],Limit_Detection)  # x, y, depth
                    po.orientation = Quaternion(0,0,0,1) # identity
            
            if check_qualification:
                self.poses.poses.append(po)

        sys.stdout.write('OBS Length %d\r' % (len(self.poses.poses)))
        sys.stdout.flush()

        test.channels.append(get_in)
        test.header.frame_id = 'rslidar'
        test.header.stamp = rospy.Time.now()
        self.detected_objects_publisher.publish(test)

        self.poses.header.frame_id = '/realsense/obstacles'
        self.poses.header.stamp = rospy.Time.now()
        self.pub_obs.publish(self.poses)

if __name__ == "__main__":
    lidar_test()
