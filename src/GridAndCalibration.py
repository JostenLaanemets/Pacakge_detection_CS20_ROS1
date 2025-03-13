#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point
from std_msgs.msg import Float32MultiArray
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tf_trans

# Global variables to store the frozen calibration plane, filtered distances, and previous live distances.
calibration_plane = None
filtered_grid_distances = None
prev_live_distances = None

def plane_fit(points):
    """
    Fit a plane using least squares.
    Plane equation: a*X + b*Y - Z + d = 0, i.e. Z = a*X + b*Y + d.
    """
    X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
    A = np.c_[X, Y, np.ones_like(X)]
    coeff, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
    a, b, d = coeff
    c = -1.0  # fixed coefficient for Z
    return a, b, c, d

def normal_to_quaternion(normal):
    """
    Convert a plane normal into a quaternion for proper RViz orientation.
    """
    normal_norm = normal / np.linalg.norm(normal)
    x_axis = np.cross([0, 0, 1], normal_norm)
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([1, 0, 0])
    else:
        x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(normal_norm, x_axis)
    
    rot_mat = np.eye(4)
    rot_mat[:3, 0] = x_axis
    rot_mat[:3, 1] = y_axis
    rot_mat[:3, 2] = normal_norm
    quat = tf_trans.quaternion_from_matrix(rot_mat)
    return quat

def create_plane_marker(a, b, c, points):
    """
    Create a cube marker representing the calibration plane.
    The marker's pose is based on the point cloud's mean (with a small offset).
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

    marker.pose.orientation = Quaternion(*quat)
    marker.pose.position.x = -0.051
    marker.pose.position.y = -0.01
    marker.pose.position.z = np.mean(points[:, 2]) - 0.02

    marker.scale.x = 0.39    # width of the plane
    marker.scale.y = 0.59    # height of the plane
    marker.scale.z = 0.05    # thickness of the plane

    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def generate_grid_points_on_plane(marker, grid_rows=5, grid_cols=5):
    """
    Generate grid cell center points on the plane defined by the marker.
    The grid divides the plane's area (from marker scale) into grid_rows x grid_cols cells.
    """
    cell_width = marker.scale.x / grid_cols
    cell_height = marker.scale.y / grid_rows

    pos = np.array([marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z])
    quat = [marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w]
    rot = tf_trans.quaternion_matrix(quat)[:3, :3]

    grid_points = []
    for i in range(grid_rows):
        for j in range(grid_cols):
            # Local coordinate of the cell center (marker center is at 0,0).
            x_local = -marker.scale.x / 2 + (j + 0.5) * cell_width
            y_local = -marker.scale.y / 2 + (i + 0.5) * cell_height
            local_point = np.array([x_local, y_local, 0.0])
            global_point = pos + np.dot(rot, local_point)
            grid_points.append(global_point)
    return grid_points

def create_grid_marker(grid_points):
    """
    Create a SPHERE_LIST marker to visualize grid cell center points in RViz.
    """
    marker = Marker()
    marker.header.frame_id = "depth_camera_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "grid_points"
    marker.id = 1
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD

    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02

    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    for pt in grid_points:
        p = Point()
        p.x, p.y, p.z = pt
        marker.points.append(p)
    return marker

def create_text_markers(grid_points, grid_distances):
    """
    Create a MarkerArray of TEXT_VIEW_FACING markers to display the distance for each grid cell.
    Each marker is positioned near its grid point with a small Z offset.
    """
    marker_array = MarkerArray()
    for idx, (pt, dist) in enumerate(zip(grid_points, grid_distances)):
        text_marker = Marker()
        text_marker.header.frame_id = "depth_camera_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "grid_distance"
        text_marker.id = idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = pt[0]
        text_marker.pose.position.y = pt[1]
        text_marker.pose.position.z = pt[2] + 0.02  # slight offset above the grid point
        text_marker.pose.orientation.w = 1.0

        text_marker.scale.z = 0.03  # text height

        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0

        text_marker.text = "{:.3f}".format(dist)
        marker_array.markers.append(text_marker)
    return marker_array

def compute_live_grid_distances(marker, points, grid_rows, grid_cols):
    """
    Compute live grid distances using the fixed calibration plane.
    For each grid cell, the median distance (instead of minimum) among points is computed.
    """
    marker_pos = np.array([marker.pose.position.x,
                           marker.pose.position.y,
                           marker.pose.position.z])
    quat = [marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w]
    T = tf_trans.quaternion_matrix(quat)
    T[:3, 3] = marker_pos
    T_inv = np.linalg.inv(T)
    
    homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))
    local_points = (T_inv @ homogeneous_points.T).T
    
    cell_width = marker.scale.x / grid_cols
    cell_height = marker.scale.y / grid_rows
    live_distances = np.zeros((grid_rows, grid_cols))
    
    for i in range(grid_rows):
        for j in range(grid_cols):
            x_min = -marker.scale.x / 2 + j * cell_width
            x_max = x_min + cell_width
            y_min = -marker.scale.y / 2 + i * cell_height
            y_max = y_min + cell_height
            mask = (local_points[:, 0] >= x_min) & (local_points[:, 0] < x_max) & \
                   (local_points[:, 1] >= y_min) & (local_points[:, 1] < y_max)
            cell_points = points[mask]  # Use global coordinates for distance computation.
            if cell_points.shape[0] > 0:
                distances = np.linalg.norm(cell_points, axis=1)
                d = np.median(distances)
            else:
                d = 0.0
            live_distances[i, j] = d
    return live_distances.flatten()

def update_temporal_filter(new_distances, alpha=0.1):
    """
    Apply an exponential moving average to smooth the grid distances over time.
    """
    global filtered_grid_distances
    if filtered_grid_distances is None:
        filtered_grid_distances = new_distances
    else:
        filtered_grid_distances = alpha * new_distances + (1 - alpha) * filtered_grid_distances
    return filtered_grid_distances

def callback(msg, args):
    global calibration_plane, prev_live_distances
    pub_plane_marker, pub_grid_marker, pub_text_markers, pub_distance_array = args

    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
    if points.shape[0] < 3:
        rospy.logwarn("Not enough points to fit a plane.")
        return

    # Calibrate the plane once (freeze it).
    if calibration_plane is None:
        a, b, c, d = plane_fit(points)
        calibration_plane = create_plane_marker(a, b, c, points)
        rospy.loginfo("Calibration completed: a=%.3f, b=%.3f, c=%.3f, d=%.3f", a, b, c, d)
        pub_plane_marker.publish(calibration_plane)
    else:
        pub_plane_marker.publish(calibration_plane)
        grid_rows = rospy.get_param("~grid_rows", 10)
        grid_cols = rospy.get_param("~grid_cols", 8)

        # Generate and publish grid points.
        grid_points = generate_grid_points_on_plane(calibration_plane, grid_rows, grid_cols)
        grid_marker = create_grid_marker(grid_points)
        pub_grid_marker.publish(grid_marker)

        # Compute live grid distances.
        live_distances = compute_live_grid_distances(calibration_plane, points, grid_rows, grid_cols)

        # Compare current live distances with previous distances.
        global prev_live_distances
        if prev_live_distances is not None:
            for idx, (prev_d, curr_d) in enumerate(zip(prev_live_distances, live_distances)):
                #if idx == 0:
                    #print("Grid cell %d detected: previous distance %.3f, current distance %.3f" % (idx, prev_d, curr_d))
                if prev_d - curr_d >= 0.003:
                    
                    rospy.loginfo("Grid cell %d detected: previous distance %.3f, current distance %.3f", idx, prev_d, curr_d)
                    #pass
        else:
            rospy.sleep(1)
            prev_live_distances = live_distances.copy()

        # Apply temporal filtering to smooth the distances.
        smoothed_distances = update_temporal_filter(live_distances)

        # Publish text markers displaying the (smoothed) distances.
        text_marker_array = create_text_markers(grid_points, smoothed_distances)
        pub_text_markers.publish(text_marker_array)

        # Also publish the smoothed grid distances as a Float32MultiArray.
        distance_msg = Float32MultiArray()
        distance_msg.data = smoothed_distances.tolist()
        pub_distance_array.publish(distance_msg)

        

def main():
    rospy.init_node("plane_grid_finder", anonymous=True)
    
    pub_plane_marker = rospy.Publisher("/plane_marker", Marker, queue_size=1)
    pub_grid_marker = rospy.Publisher("/grid_marker", Marker, queue_size=1)
    pub_text_markers = rospy.Publisher("/grid_distance_markers", MarkerArray, queue_size=1)
    pub_distance_array = rospy.Publisher("/grid_distance_array", Float32MultiArray, queue_size=1)
    
    rospy.Subscriber("/camera1/points2", PointCloud2, callback,
                     callback_args=(pub_plane_marker, pub_grid_marker, pub_text_markers, pub_distance_array))
    
    rospy.spin()

if __name__ == "__main__":
    main()
