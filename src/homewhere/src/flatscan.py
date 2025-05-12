#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(cloud):
    points = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
    if points:
        z_values = [p[2] for p in points]
        print("Z min:", min(z_values), "Z max:", max(z_values))

rospy.init_node('cloud_z_debugger')
rospy.Subscriber('/scan_cloud', PointCloud2, callback)
rospy.spin()