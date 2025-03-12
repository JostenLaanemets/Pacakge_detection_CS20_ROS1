#!/usr/bin/env python
import rospy
import numpy as np
import open3d as o3d  # Open3D for point cloud manipulation
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker  # Marker for visualization
from geometry_msgs.msg import Point

# Define the bounding box coordinates (min and max in X, Y, Z)
min_bound = [-0.35, -0.23, 0.0]  # Define min bound of bounding box (X, Y, Z)
max_bound = [0.26, 0.2, 1.0]   # Define max bound of bounding box (X, Y, Z)

def point_cloud_callback(msg):
    # Convert ROS PointCloud2 message to NumPy array
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc_data))

    if len(points) == 0:
        rospy.logwarn("No points received from LiDAR!")
        return

    # Filter the points based on the bounding box
    mask = (points[:, 0] >= min_bound[0]) & (points[:, 0] <= max_bound[0]) & \
           (points[:, 1] >= min_bound[1]) & (points[:, 1] <= max_bound[1]) & \
           (points[:, 2] >= min_bound[2]) & (points[:, 2] <= max_bound[2])

    filtered_points = points[mask]

    # Create Open3D PointCloud for visualization
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(filtered_points)

    # Create PointCloud2 message from filtered points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "depth_camera_link"  # Change to match your camera frame

    filtered_cloud_ros = pc2.create_cloud_xyz32(header, filtered_points)

    # Publish the filtered point cloud
    pub_pc.publish(filtered_cloud_ros)
    rospy.loginfo("Published filtered point cloud with %d points", len(filtered_points))

    # Create a visualization marker for the bounding box
    marker = Marker()
    marker.header = header
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Set marker position (center of the bounding box)
    centroid = np.mean(filtered_points, axis=0)
    marker.pose.position.x = centroid[0]
    marker.pose.position.y = centroid[1]
    marker.pose.position.z = centroid[2]

    # Set marker size based on bounding box dimensions
    marker.scale.x = max_bound[0] - min_bound[0]
    marker.scale.y = max_bound[1] - min_bound[1]
    marker.scale.z = max_bound[2] - min_bound[2]

    # Set marker color (semi-transparent)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5  # Transparency

    pub_marker.publish(marker)
    rospy.loginfo("Published Bounding Box Marker")

def listener():
    rospy.init_node('bounding_box_detector', anonymous=True)

    # Subscribe to the point cloud topic
    rospy.Subscriber("/camera1/points2", PointCloud2, point_cloud_callback)

    # Create publishers
    global pub_pc, pub_marker
    pub_pc = rospy.Publisher("/filtered_point_cloud", PointCloud2, queue_size=1)
    pub_marker = rospy.Publisher("/bounding_box_marker", Marker, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
