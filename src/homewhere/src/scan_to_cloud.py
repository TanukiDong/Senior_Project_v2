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

        # Laser projection tool
        self.lp = LaserProjection()

        # Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        # Publishers
        self.cloud_pub_full = rospy.Publisher('/scan_cloud_full', PointCloud2, queue_size=10)   # For height map
        self.cloud_pub_clean = rospy.Publisher('/scan_cloud_clean', PointCloud2, queue_size=10) # For SLAM

        #rospy.loginfo("✅ scan_to_cloud node initialized and running.")

    def scan_callback(self, scan_msg):
        try:
            # Step 1: Project LaserScan to PointCloud2 (sensor frame)
            cloud_out = self.lp.projectLaser(scan_msg)

            # Step 2: Transform to base_link frame
            transform = self.tf_buffer.lookup_transform(
                target_frame="base_link",
                source_frame=scan_msg.header.frame_id,
                time=scan_msg.header.stamp,
                timeout=rospy.Duration(1.0)
            )

            cloud_transformed = tf2_sensor_msgs.do_transform_cloud(cloud_out, transform)

            # Convert to list of points for filtering
            points = list(pc2.read_points(cloud_transformed, field_names=("x", "y", "z"), skip_nans=True))

            if not points:
                rospy.logwarn("⚠️ No points in cloud after transformation.")
                return

            # Step 3: Publish full cloud (no filter) for height mapping
            cloud_transformed.header.stamp = rospy.Time.now()
            cloud_transformed.header.frame_id = "base_link"
            self.cloud_pub_full.publish(cloud_transformed)

            # Step 4: Filter for SLAM (reduce noise)
            filtered_points = [p for p in points if abs(p[2]) < 0.3]

            # Optional: Debug info
            z_values = [p[2] for p in points]
            if z_values:
                pass
                #rospy.loginfo(f"🌐 Full PointCloud Z range: min {min(z_values):.3f}, max {max(z_values):.3f}")
            if filtered_points:
                z_filtered = [p[2] for p in filtered_points]
                #rospy.loginfo(f"🧹 Filtered PointCloud Z range: min {min(z_filtered):.3f}, max {max(z_filtered):.3f}")

            # Step 5: Create filtered PointCloud2
            if filtered_points:
                cloud_filtered = pc2.create_cloud_xyz32(cloud_transformed.header, [(p[0], p[1], p[2]) for p in filtered_points])
                self.cloud_pub_clean.publish(cloud_filtered)

            #rospy.loginfo("✅ Published both full and filtered clouds.")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"⚠️ TF exception: {e}")

if __name__ == '__main__':
    try:
        ScanToCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
