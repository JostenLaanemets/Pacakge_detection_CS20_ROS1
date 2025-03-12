#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_trans

def plane_fit(points):
    """
    Fit a plane using least squares.
    Plane equation: aX + bY + cZ + d = 0, forcing c = -1 => Z = aX + bY + d.
    """
    X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
    A = np.c_[X, Y, np.ones_like(X)]
    B = Z
    coeff, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
    a, b, d = coeff
    c = -1.0
    return a, b, c, d

def normal_to_quaternion(normal):
    """
    Convert plane normal into a quaternion for correct orientation in RViz.
    """
    normal_norm = normal / np.linalg.norm(normal)
    x_axis = np.cross([0, 0, 1], normal_norm)
    # Handle the degenerate case when normal is parallel to [0, 0, 1]
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([1, 0, 0])
    else:
        x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal_norm, x_axis)
    
    rot_mat = np.eye(4)
    rot_mat[:3, 0] = x_axis
    rot_mat[:3, 1] = y_axis
    rot_mat[:3, 2] = normal_norm
    
    quat = tf_trans.quaternion_from_matrix(rot_mat)
    return quat

def create_plane_marker(a, b, c, points):
    """
    Creates a plane marker for visualization in RViz.
    The plane is centered at the LiDAR’s midpoint.
    """
    normal = np.array([a, b, c], dtype=np.float64)
    quat = normal_to_quaternion(normal)
    
    marker = Marker()
    marker.header.frame_id = "depth_camera_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "plane"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    
    # For demonstration, the x and y positions are fixed offsets;
    # the z position is computed from the mean of the points.
    marker.pose.orientation = Quaternion(*quat)
    marker.pose.position.x = -0.05
    marker.pose.position.y = -0.01
    marker.pose.position.z = np.mean(points[:, 2]) - 0.02
    
    marker.scale.x = 0.38    # Width of the plane marker along X
    marker.scale.y = 0.54    # Height of the plane marker along Y
    marker.scale.z = 0.05    # Thickness of the plane marker
    
    marker.color.a = 0.5     # Transparency
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def filter_points_in_plane_area(points, marker):
    """
    Filter points so that only those inside the plane area (as defined by the marker)
    are kept. Assumes points and the marker's pose are expressed in the same frame.
    """
    # Extract the marker's position and orientation
    pos = np.array([marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z])
    quat = [marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w]
    
    # Get the rotation matrix from the quaternion
    rot = tf_trans.quaternion_matrix(quat)[:3, :3]

    # Transform each point into the marker's coordinate frame.
    # Since our points are row vectors, the transformation is:
    # p_local = (p - pos) dot rot
    points_local = np.dot(points - pos, rot)

    # Define half-extents from marker scale (assuming marker.scale.z is the thickness)
    half_x = marker.scale.x / 2.0
    half_y = marker.scale.y / 2.0
    half_z = marker.scale.z / 2.0

    # Create a boolean mask for points that lie within the rectangular bounds
    mask = (
        (np.abs(points_local[:, 0]) <= half_x) &
        (np.abs(points_local[:, 1]) <= half_y) &
        (np.abs(points_local[:, 2]) <= half_z)
    )

    return points[mask]

# Global variables to hold the plane parameters, static marker, and scan count
scan_count = 0
plane_params = None
plane_marker_static = None

def callback(msg, args):
    global scan_count, plane_params, plane_marker_static
    pub_plane_marker, pub_filtered = args

    # Convert incoming PointCloud2 message into a numpy array of points
    points_camera = np.array(list(
        pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ))
    if len(points_camera) < 1:
        rospy.logwarn("No points available to process.")
        return

    # For the first 10 scans, compute and update the plane parameters.
    if scan_count < 10:
        a, b, c, d = plane_fit(points_camera)
        plane_params = (a, b, c, d)
        scan_count += 1
        rospy.loginfo("Scan %d: Plane eq: %.3f x + %.3f y + %.3f z + %.3f = 0", 
                      scan_count, a, b, c, d)
        # When 10 scans are reached, create and store the static plane marker.
        if scan_count == 10:
            plane_marker_static = create_plane_marker(a, b, c, points_camera)
            pub_plane_marker.publish(plane_marker_static)
            rospy.loginfo("Static plane marker set.")
        return

    if plane_marker_static is None:
        rospy.logwarn("Static plane marker not set yet!")
        return

    # Use the static plane marker for filtering.
    pub_plane_marker.publish(plane_marker_static)
    
    filtered_points = filter_points_in_plane_area(points_camera, plane_marker_static)
    rospy.loginfo("Filtered points count: %d", filtered_points.shape[0])
    
    # Publish the filtered points as a new PointCloud2 message if available.
    if filtered_points.shape[0] > 0:
        header = msg.header
        cloud_filtered = pc2.create_cloud_xyz32(header, filtered_points.tolist())
        pub_filtered.publish(cloud_filtered)

def main():
    rospy.init_node("plane_finder", anonymous=True)
    
    # Publisher for the plane marker (visualized in RViz)
    pub_plane_marker = rospy.Publisher("/plane_marker", Marker, queue_size=1)
    # Publisher for the filtered point cloud (only points within the plane area)
    pub_filtered = rospy.Publisher("/filtered_points", PointCloud2, queue_size=1)
    
    # Subscribe to the input point cloud topic; pass both publishers to the callback
    rospy.Subscriber("/camera1/points2", PointCloud2, callback,
                     callback_args=(pub_plane_marker, pub_filtered))
    
    rospy.spin()

if __name__ == "__main__":
    main()
