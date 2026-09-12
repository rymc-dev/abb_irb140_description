#!/usr/bin/env python3
"""Relabel the wrist RGBD camera's point cloud frame_id for Gazebo sim.

gz-sensors' rgbd_camera sensor always stamps /rgbd_camera/points with
optical_frame_id ("depth_camera_optical" here), but never actually rotates
the raw PointCloudPacked XYZ data into optical convention the way it does
for the image/depth_image/camera_info topics - the data stays in the
sensor's native body-frame axes (+X out of the lens), i.e. the same
convention as the un-rotated "depth_camera" link. Confirmed empirically
against a live sim: sampled points had x in [0.358, 2.633] (always positive,
matching near/far clip - the depth axis), y symmetric about 0 matching
depth*tan(hfov/2) (left/right), z asymmetric (up/down). That is
depth_camera's convention, not depth_camera_optical's. See
gazebosim/gz-sensors#29 and #545 (open upstream as of Gazebo Harmonic
8.2.2); neither <gz_frame_id> nor <optical_frame_id> in the sensor SDF
changes this behaviour for /points.

This node subscribes to the raw bridged cloud and republishes it unchanged
except for header.frame_id, so RViz/octomap_server transform it using the
TF frame that actually matches the data. Sim-only: the real RealSense
driver's cloud (launch/real_robot.launch.py) is genuinely optical-convention
already and is not touched by this.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class FixDepthPointsFrame(Node):
    def __init__(self):
        super().__init__('fix_depth_points_frame')
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/points', 10)
        self._sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points_raw', self._on_cloud, 10)

    def _on_cloud(self, msg: PointCloud2):
        msg.header.frame_id = 'depth_camera'
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FixDepthPointsFrame()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
