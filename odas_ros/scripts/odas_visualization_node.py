#!/usr/bin/env python3
import math

import rclpy
import rclpy.node

import numpy as np
import ros2_numpy as rnp

import io
import libconf

import std_msgs.msg

# import sensor_msgs.point_cloud2 as pcl2  # type: ignore
from odas_ros_msgs.msg import OdasSstArrayStamped, OdasSslArrayStamped
from  geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2, PointField


# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class OdasVisualizationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('odas_visualization_node')

        # Stamped Pose Message containing the converted Sound Source Tracking (SST) position from ODAS.
        self._sst_input_PoseArray = PoseArray()
        # Subscribe to the Sound Source Tracking from ODAS Server
        self._sst_sub = self.create_subscription(OdasSstArrayStamped, 'sst', self._sst_cb, 10)
        # ODAS SST Publisher for PoseStamped
        self._sst_pose_pub = self.create_publisher(PoseArray, 'sst_poses', 10)

        # Subscribe to the Sound Source Localization and Sound Source Tracking from ODAS Server
        self._ssl_sub = self.create_subscription(OdasSslArrayStamped, 'ssl', self._ssl_cb, 10)
        # ODAS SSL Publisher for PointCloud2
        self._ssl_pcl_pub = self.create_publisher(PointCloud2, 'ssl_pcl2', 500)


    def _ssl_cb(self, ssl_msg):
        points = np.array([[src.x, src.y, src.z, src.e] for src in ssl_msg.sources])
        points_array = np.array(points, dtype=np.float32)
        
        # Define the fields for PointCloud2 (x, y, z, intensity)
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
        num_sources = len(ssl_msg.sources)
        # Create a structured NumPy array
        structured_array = np.zeros(num_sources, dtype=dtype)
        structured_array['x'] = points_array[:, 0]
        structured_array['y'] = points_array[:, 1]
        structured_array['z'] = points_array[:, 2]
        structured_array['intensity'] = points_array[:, 3]
        
        # Convert the NumPy structured array to a PointCloud2 message
        pointcloud_msg = rnp.msgify(PointCloud2, structured_array)
        
        # Set the header of the PointCloud2 message
        pointcloud_msg.header = ssl_msg.header

        self._ssl_pcl_pub.publish(pointcloud_msg)

    def _sst_cb(self, sst):
        # Sound Source Tracking Callback (ODAS)
        self._sst_input_PoseArray.header.stamp = self.get_clock().now().to_msg()
        self._sst_input_PoseArray.header.frame_id = sst.header.frame_id
        self._sst_input_PoseArray.poses = []

        for src in sst.sources:
            if src.id == 0:
                continue
            q = self._unit_vector_to_quaternion(src.x, src.y, src.z)

            # Update the SST PoseStamped
            _sst_input_Pose = Pose()
            _sst_input_Pose.position.x = 0.0
            _sst_input_Pose.position.y = 0.0
            _sst_input_Pose.position.z = 0.0
            _sst_input_Pose.orientation.x = q[0]
            _sst_input_Pose.orientation.y = q[1]
            _sst_input_Pose.orientation.z = q[2]
            _sst_input_Pose.orientation.w = q[3]

            self._sst_input_PoseArray.poses.append(_sst_input_Pose)

        self._sst_pose_pub.publish(self._sst_input_PoseArray)


    def _unit_vector_to_quaternion(self, x, y, z):
        # Convert a xyz unit vector (point on a unit sphere) to quaternion
        yaw = np.arctan2(y,x)
        pitch = -np.arctan2(z,np.sqrt(x*x+y*y))
        roll = 0
        q = quaternion_from_euler(roll, pitch, yaw)
        return q


    def run(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odas_visualization_node = OdasVisualizationNode()

    try:
        odas_visualization_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        odas_visualization_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
