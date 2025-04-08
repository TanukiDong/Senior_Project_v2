#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
import sensor_msgs.point_cloud2 as pc2

class ScanToCloud:
    def __init__(self):
        rospy.init_node('scan_to_cloud', anonymous=True)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Laser projection
        self.lp = LaserProjection()

        # Subscribers and publishers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.cloud_pub = rospy.Publisher('/scan_cloud', PointCloud2, queue_size=10)

        rospy.loginfo("✅ scan_to_cloud node initialized.")

    def scan_callback(self, scan_msg):
        try:
            # Step 1: Project LaserScan to PointCloud2 (sensor frame)
            cloud_out = self.lp.projectLaser(scan_msg)

            # Step 2: Lookup transform from sensor frame to base_link
            transform = self.tf_buffer.lookup_transform(
                target_frame="base_link",  # Adjust if your robot frame is different
                source_frame=scan_msg.header.frame_id,
                time=scan_msg.header.stamp,
                timeout=rospy.Duration(1.0)
            )

            # Step 3: Transform cloud to base_link
            cloud_transformed = tf2_sensor_msgs.do_transform_cloud(cloud_out, transform)

            # Optional Debug: Print Z range of point cloud
            points = list(pc2.read_points(cloud_transformed, field_names=("x", "y", "z"), skip_nans=True))
            if points:
                z_values = [p[2] for p in points]
                rospy.loginfo(f"🌐 PointCloud Z range: min {min(z_values):.3f}, max {max(z_values):.3f}")

            # Step 4: Publish the transformed cloud
            cloud_transformed.header.stamp = rospy.Time.now()
            cloud_transformed.header.frame_id = "base_link"
            self.cloud_pub.publish(cloud_transformed)

            rospy.loginfo("✅ Published transformed cloud!")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"⚠️ TF exception: {e}")

if __name__ == '__main__':
    try:
        node = ScanToCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
