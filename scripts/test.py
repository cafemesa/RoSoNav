#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from tf import TransformListener
from sensor_msgs import point_cloud2
from math import atan2, sqrt
import numpy as np

class PointCloudToLaserScan:
    def __init__(self):
        rospy.init_node('pointcloud_to_laserscan_node', anonymous=True)
        self.tf_listener = TransformListener()

        self.pc_subscriber = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pointcloud_callback)
        self.laser_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)

        

    def pointcloud_callback(self, pointcloud_msg):

        ranges = []
        width = pointcloud_msg.width
        height = pointcloud_msg.height

        self.row_index = height//2

        row_offset = self.row_index * width

        for i, point in enumerate(point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)):
            if i >= row_offset and i < row_offset + width:
                x, y, z = point
                range_val = sqrt(x**2 + y**2)
                angle = atan2(y, x)
                ranges.append((angle, range_val))

        ranges.sort(key=lambda x: x[0])

        scan_msg = LaserScan()
        scan_msg.header = pointcloud_msg.header
        scan_msg.angle_min = ranges[0][0]
        scan_msg.angle_max = ranges[-1][0]
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(ranges)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.0
        scan_msg.range_max = np.nanmax([r[1] for r in ranges])

        scan_msg.ranges = [r[1] for r in ranges]

        self.laser_publisher.publish(scan_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pcl2ls = PointCloudToLaserScan()
        pcl2ls.run()
    except rospy.ROSInterruptException:
        pass
