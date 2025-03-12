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
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal_norm, x_axis)
    
    rot_mat = np.eye(4)
    rot_mat[:3, 0] = x_axis
    rot_mat[:3, 1] = y_axis
    rot_mat[:3, 2] = normal_norm
    
    quat = tf_trans.quaternion_from_matrix(rot_mat)
    return quat

def create_plane_marker(a, b, c, midpoint):
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
    
    marker.pose.orientation = Quaternion(*quat)
    marker.pose.position.x = -0.05
    marker.pose.position.y = -0.01
    marker.pose.position.z = np.mean(midpoint[:, 2]) - 0.002
    
    marker.scale.x = 0.4
    marker.scale.y = 0.56
    marker.scale.z = 0.002
    
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

scan_count = 0
plane_params = None

def callback(msg, pub_plane_marker):
    global scan_count, plane_params
    
    points_camera = np.array(list(
        pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ))
    if len(points_camera) < 1:
        rospy.logwarn("No points available to fit a plane.")
        return
    
    if scan_count < 10:
        a, b, c, d = plane_fit(points_camera)
        plane_params = (a, b, c, d)
        rospy.loginfo("Plane eq: %.3f x + %.3f y + %.3f z + %.3f = 0", a, b, c, d)
        scan_count += 1
        return
    
    if plane_params is None:
        rospy.logwarn("No plane detected yet!")
        return
    
    a, b, c, d = plane_params
    plane_marker = create_plane_marker(a, b, c, points_camera)
    pub_plane_marker.publish(plane_marker)

def main():
    rospy.init_node("plane_finder", anonymous=True)
    pub_plane_marker = rospy.Publisher("/plane_marker", Marker, queue_size=1)
    rospy.Subscriber("/camera1/points2", PointCloud2, callback, callback_args=pub_plane_marker)
    rospy.spin()

if __name__ == "__main__":
    main()
