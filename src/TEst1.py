#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Quaternion, Point
import tf.transformations as tf_trans
from std_msgs.msg import Header, ColorRGBA

FRAME_ID = "depth_camera_link"

# ROI bounds in the plane's coordinate system
plane_xmin, plane_xmax = -0.26, 0.25
plane_ymin, plane_ymax = -0.20, 0.15
plane_zmin, plane_zmax = -0.04, 0.02

# Fixed position for ROI in camera frame
SHIFT_X = 0.0
SHIFT_Y = 0.2
SHIFT_Z = 1.0

# Safety margin for ROI filtering
MARGIN = 0.0

# Offset for the plane marker along its normal
plane_height_offset = 0.005

def plane_fit(points):
    """
    Fit a plane aX + bY + cZ + d = 0, forcing c = -1 => Z = aX + bY + d.
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
    Convert plane normal into a quaternion that orients +Z to that normal.
    """
    normal_norm = normal / np.linalg.norm(normal)
    x_axis = np.cross([0, 0, 1], normal_norm)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal_norm, x_axis)
    
    rot_mat = np.eye(4)
    rot_mat[:3, 0] = x_axis
    rot_mat[:3, 1] = y_axis
    rot_mat[:3, 2] = normal_norm
    
    quat = tf_trans.quaternion_from_matrix(rot_mat)
    return quat, rot_mat[:3, :3]

def create_plane_marker(a, b, c, midpoint):
    """
    Visualize the fitted plane
    """
    normal = np.array([a, b, c], dtype=np.float64)
    normal_norm = normal / np.linalg.norm(normal)
    quat, _ = normal_to_quaternion(normal)
    
    plane_center = midpoint + plane_height_offset * normal_norm
    
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.ns = "plane"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    
    marker.pose.orientation = Quaternion(*quat)
    marker.pose.position.x = plane_center[0]
    marker.pose.position.y = plane_center[1]
    marker.pose.position.z = plane_center[2]
    
    marker.scale.x = 0.4
    marker.scale.y = 0.5
    marker.scale.z = 0.002
    
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def create_roi_marker(a, b, c, d):
    """
    Create ROI box using plane equation
    """
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    quat, R = normal_to_quaternion(normal)
    
    # Calculate center Z using plane equation: z = ax + by + d
    center_x = SHIFT_X
    center_y = SHIFT_Y
    center_z = a*SHIFT_X + b*SHIFT_Y + d
    
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_box"
    marker.id = 1
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    
    marker.pose.orientation = Quaternion(*quat)
    marker.pose.position.x = center_x
    marker.pose.position.y = center_y
    marker.pose.position.z = center_z
    
    marker.scale.x = plane_xmax - plane_xmin
    marker.scale.y = plane_ymax - plane_ymin
    marker.scale.z = plane_zmax - plane_zmin
    
    marker.color.a = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    return marker

def create_roi_bounds_marker(a, b, c, d):
    """
    Create wireframe ROI bounds rotated 90 degrees right
    """
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    quat, R = normal_to_quaternion(normal)
    
    # Calculate center Z using plane equation: z = ax + by + d
    center_x = SHIFT_X
    center_y = SHIFT_Y
    center_z = a*SHIFT_X + b*SHIFT_Y + d
    
    # Define corners relative to center
    half_width = (plane_xmax - plane_xmin) / 2.0
    half_length = (plane_ymax - plane_ymin) / 2.0
    half_height = (plane_zmax - plane_zmin) / 2.0

    # Create corners in local frame, already rotated 90 degrees
    corners_local = np.array([
        [half_length, -half_width, -half_height],   # Front right
        [half_length, half_width, -half_height],    # Front left
        [-half_length, half_width, -half_height],   # Back left
        [-half_length, -half_width, -half_height],  # Back right
        [half_length, -half_width, half_height],    # Top front right
        [half_length, half_width, half_height],     # Top front left
        [-half_length, half_width, half_height],    # Top back left
        [-half_length, -half_width, half_height],   # Top back right
    ])

    # Create rotation matrix for 90 degrees around Z
    theta = np.pi/2  # 90 degrees
    rot_z = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    # Apply rotations and translation
    corners = corners_local.dot(rot_z).dot(R) + [center_x, center_y, center_z]

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_bounds"
    marker.id = 3
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    marker.scale.x = 0.005
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Define the lines for the box
    lines = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom
        (4, 5), (5, 6), (6, 7), (7, 4),  # Top
        (0, 4), (1, 5), (2, 6), (3, 7)   # Sides
    ]

    marker.points = []
    for start_idx, end_idx in lines:
        start_point = Point(
            x=corners[start_idx][0],
            y=corners[start_idx][1],
            z=corners[start_idx][2]
        )
        end_point = Point(
            x=corners[end_idx][0],
            y=corners[end_idx][1],
            z=corners[end_idx][2]
        )
        marker.points.extend([start_point, end_point])

    return marker

def create_points_marker(points_cam):
    """
    Create marker for ROI-filtered points
    """
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.ns = "roi_points"
    marker.id = 2
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    
    marker.points = [Point(x=pt[0], y=pt[1], z=pt[2]) for pt in points_cam]
    return marker

def create_cloud_msg(points_array):
    header = Header(
        stamp=rospy.Time.now(),
        frame_id=FRAME_ID
    )
    return pc2.create_cloud_xyz32(header, points_array.tolist())

def filter_points_in_roi(points, a, b, d, normal, R):
    """
    Filter points within ROI using plane equation
    """
    # Transform points to ROI coordinate system
    points_centered = points - [SHIFT_X, SHIFT_Y, a*SHIFT_X + b*SHIFT_Y + d]
    points_roi = points_centered.dot(R.T)
    
    # Apply ROI bounds
    roi_mask = (
        (points_roi[:, 0] >= plane_xmin) & (points_roi[:, 0] <= plane_xmax) &
        (points_roi[:, 1] >= plane_ymin) & (points_roi[:, 1] <= plane_ymax) &
        (points_roi[:, 2] >= plane_zmin) & (points_roi[:, 2] <= plane_zmax)
    )
    
    return points[roi_mask]

def callback(msg):
    points_camera = np.array(list(
        pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
    ))
    if len(points_camera) < 1:
        rospy.logwarn("No points available to fit a plane.")
        return
    
    # 1) Plane fit
    a, b, c_, d = plane_fit(points_camera)
    rospy.loginfo("Plane eq: %.3f x + %.3f y + %.3f z + %.3f = 0", a, b, c_, d)
    
    # 2) Get plane normal and rotation
    normal = np.array([a, b, c_])
    normal = normal / np.linalg.norm(normal)
    _, R = normal_to_quaternion(normal)
    
    # 3) Filter points in ROI
    filtered_points = filter_points_in_roi(points_camera, a, b, d, normal, R)
    
    if len(filtered_points) == 0:
        rospy.logwarn("No points found in the ROI.")
        return
    
    # 4) Create and publish markers
    plane_marker = create_plane_marker(a, b, c_, np.mean(points_camera, axis=0))
    pub_plane_marker.publish(plane_marker)
    
    roi_marker = create_roi_marker(a, b, c_, d)
    pub_roi_marker.publish(roi_marker)
    
    bounds_marker = create_roi_bounds_marker(a, b, c_, d)
    pub_bounds_marker.publish(bounds_marker)
    
    pc2_msg = create_cloud_msg(filtered_points)
    pub_points.publish(pc2_msg)
    
    pts_marker = create_points_marker(filtered_points)
    pub_points_marker.publish(pts_marker)

def main():
    rospy.init_node("plane_roi_node", anonymous=True)
    rospy.Subscriber("/camera1/points2", PointCloud2, callback)
    rospy.spin()

if __name__ == "__main__":
    pub_plane_marker   = rospy.Publisher("/plane_marker", Marker, queue_size=1)
    pub_roi_marker     = rospy.Publisher("/roi_box_marker", Marker, queue_size=1)
    pub_bounds_marker  = rospy.Publisher("/roi_bounds_marker", Marker, queue_size=1)
    pub_points         = rospy.Publisher("/detected_points", PointCloud2, queue_size=1)
    pub_points_marker  = rospy.Publisher("/detected_points_marker", Marker, queue_size=1)
    main()