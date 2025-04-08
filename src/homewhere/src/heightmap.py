#!/usr/bin/env python
import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray

import tf
import tf2_ros

class HeightMapNode:
    def __init__(self):
        # --- Map Parameters ---
        self.map_size = 20.0  # meters (grid covers 20m x 20m)
        self.resolution = 0.1  # meters per cell
        self.origin_x = -self.map_size / 2.0
        self.origin_y = -self.map_size / 2.0

        self.grid_width = int(self.map_size / self.resolution)
        self.grid_height = int(self.map_size / self.resolution)

        # Height map and timestamps for each cell
        self.height_map = np.full((self.grid_width, self.grid_height), np.nan)
        self.cell_timestamp = np.full((self.grid_width, self.grid_height), rospy.get_time())

        
        # Robot pose from odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # IMU orientation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_imu = 0.0

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publishers
        self.map_pub = rospy.Publisher('/height_map', OccupancyGrid, queue_size=1)
        self.marker_pub = rospy.Publisher('/height_map_markers', MarkerArray, queue_size=1)
        self.gradient_marker_pub = rospy.Publisher('/gradient_map_markers', MarkerArray, queue_size=1)

        # TF (optional, not used currently)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Timer to publish map every second
        rospy.Timer(rospy.Duration(0.1), self.publish_map)

        rospy.loginfo("✅ HeightMapNode initialized and running.")

    # -----------------------------
    # Callbacks
    # -----------------------------

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = euler[2]

    def imu_callback(self, msg):
        q = msg.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll, self.pitch, self.yaw_imu = euler

    def scan_callback(self, scan_msg):
        angle = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        lidar_tilt = -0.1  # radians (match your URDF tilt!)

        for r in scan_msg.ranges:
            if r < scan_msg.range_min or r > scan_msg.range_max:
                angle += angle_increment
                continue

            # Polar to Cartesian conversion with tilt
            x_lidar = r * math.cos(angle) * math.cos(lidar_tilt)
            y_lidar = r * math.sin(angle)
            z_lidar = -r * math.cos(angle) * math.sin(lidar_tilt)

            angle += angle_increment

            # Apply IMU correction
            x_corr, y_corr, z_corr = self.correct_for_imu(x_lidar, y_lidar, z_lidar)

            # Transform to global map frame
            X, Y, Z = self.transform_to_global(x_corr, y_corr, z_corr)

            # Update height map
            self.update_height_map(X, Y, Z)

    # -----------------------------
    # Core Processing Functions
    # -----------------------------

    def correct_for_imu(self, x_local, y_local, z_local):
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)

        # Pitch correction
        x_pitch = x_local * cp + z_local * sp
        y_pitch = y_local
        z_pitch = -x_local * sp + z_local * cp

        # Roll correction
        x_roll = x_pitch
        y_roll = y_pitch * cr - z_pitch * sr
        z_roll = y_pitch * sr + z_pitch * cr

        return x_roll, y_roll, z_roll

    def transform_to_global(self, x_corr, y_corr, z_corr):
        cy = math.cos(self.robot_yaw)
        sy = math.sin(self.robot_yaw)

        X = self.robot_x + (x_corr * cy - y_corr * sy)
        Y = self.robot_y + (x_corr * sy + y_corr * cy)
        Z = z_corr  # Add LiDAR mount height if necessary

        return X, Y, Z

    def update_height_map(self, X, Y, Z):
        i = int((X - self.origin_x) / self.resolution)
        j = int((Y - self.origin_y) / self.resolution)

        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            current_time = rospy.get_time()

            if np.isnan(self.height_map[i, j]):
                self.height_map[i, j] = Z
                self.cell_timestamp[i, j] = current_time
                print(f"🆕 New cell ({i}, {j}): Height = {Z:.3f} m")
            else:
                new_height = min(self.height_map[i, j], Z)
                if new_height != self.height_map[i, j]:
                    print(f"🔄 Updated cell ({i}, {j}): {self.height_map[i, j]:.3f} -> {new_height:.3f} m")
                self.height_map[i, j] = new_height
                self.cell_timestamp[i, j] = current_time

    def compute_gradient_map(self):
        grad_map = np.full((self.grid_width, self.grid_height), np.nan)

        for i in range(self.grid_width - 1):
            for j in range(self.grid_height - 1):
                if not np.isnan(self.height_map[i, j]) and not np.isnan(self.height_map[i + 1, j]) and not np.isnan(self.height_map[i, j + 1]):
                    dh_dx = self.height_map[i + 1, j] - self.height_map[i, j]
                    dh_dy = self.height_map[i, j + 1] - self.height_map[i, j]
                    grad_map[i, j] = math.sqrt(dh_dx**2 + dh_dy**2)

        return grad_map

    def classify_map(self, grad_map, ramp_threshold_low=0.05, ramp_threshold_high=0.15):
        class_map = np.zeros_like(grad_map, dtype=int)

        for i in range(grad_map.shape[0]):
            for j in range(grad_map.shape[1]):
                if np.isnan(grad_map[i, j]):
                    class_map[i, j] = -1  # Unknown
                elif grad_map[i, j] < ramp_threshold_low:
                    class_map[i, j] = 0  # Flat
                elif ramp_threshold_low <= grad_map[i, j] <= ramp_threshold_high:
                    class_map[i, j] = 1  # Ramp
                else:
                    class_map[i, j] = 2  # Wall

        return class_map

    def is_continuous_ramp(self, class_map, min_length=3):
        for i in range(self.grid_width):
            ramp_length = 0
            for j in range(self.grid_height):
                if class_map[i, j] == 1:  # Ramp
                    ramp_length += 1
                    if ramp_length >= min_length:
                        return True
                else:
                    ramp_length = 0
        return False

    # -----------------------------
    # Visualization and Publishing
    # -----------------------------
    def publish_gradient_markers(self, grad_map):
        marker_array = MarkerArray()
        marker_id = 0

        for i in range(self.grid_width):
            for j in range(self.grid_height):
                if np.isnan(grad_map[i, j]):
                    continue  # Skip unknown cells

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "gradient_map"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Marker position
                marker.pose.position.x = self.origin_x + (i + 0.5) * self.resolution
                marker.pose.position.y = self.origin_y + (j + 0.5) * self.resolution
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0

                # Marker size
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = 0.05

                # Color based on gradient value
                gradient = grad_map[i, j]

                # Normalize for color scaling (adjust max_grad for your environment)
                max_grad = 0.3  # Set expected maximum gradient
                norm_grad = min(gradient / max_grad, 1.0)

                # Color scale: green (low) -> yellow (medium) -> red (high)
                if norm_grad < 0.33:
                    # Low gradient: Green
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif norm_grad < 0.66:
                    # Medium gradient: Yellow
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    # High gradient: Red
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0

                marker.color.a = 0.8  # Visibility

                marker_array.markers.append(marker)
                marker_id += 1

        self.gradient_marker_pub.publish(marker_array)

    def publish_classification_markers(self, class_map):
        marker_array = MarkerArray()
        marker_id = 0

        for i in range(self.grid_width):
            for j in range(self.grid_height):
                if class_map[i, j] == -1:
                    continue  # Skip unknown cells

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "height_map"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Marker position
                marker.pose.position.x = self.origin_x + (i + 0.5) * self.resolution
                marker.pose.position.y = self.origin_y + (j + 0.5) * self.resolution
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0

                # Marker size
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = 0.05

                # Marker color
                if class_map[i, j] == 0:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif class_map[i, j] == 1:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif class_map[i, j] == 2:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def publish_map(self, event):
        # Clean old cells
        current_time = rospy.get_time()
        cell_lifetime = 1.0  # seconds
        time_since_update = current_time - self.cell_timestamp
        expired_cells = time_since_update > cell_lifetime
        self.height_map[expired_cells] = np.nan

        num_cleared = np.sum(expired_cells)
        if num_cleared > 0:
            print(f"🧹 Cleared {num_cleared} old cells.")

        # Compute maps
        grad_map = self.compute_gradient_map()
        print(f"📊 Max gradient in map: {np.nanmax(grad_map):.3f}")

        class_map = self.classify_map(grad_map)
        # Publish new gradient markers
        self.publish_gradient_markers(grad_map)

        # Detection messages
        if np.any(class_map == 1):
            rospy.loginfo("🚧 Ramp detected!")
        if np.any(class_map == 2):
            rospy.loginfo("🧱 Wall detected!")
        if self.is_continuous_ramp(class_map):
            print("🚧🚧🚧 Continuous ramp detected! 🚧🚧🚧")

        # Publish occupancy grid (debugging)
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.orientation.w = 1.0

        data = []
        flat_map = self.height_map.flatten()
        for h in flat_map:
            if np.isnan(h):
                data.append(-1)
            else:
                scaled = int(max(0, min(100, (h + 2.0) * 10)))
                data.append(scaled)

        grid_msg.data = data
        self.map_pub.publish(grid_msg)

        

        # Publish markers
        self.publish_classification_markers(class_map)

# -----------------------------
# Main Entry Point
# -----------------------------

def main():
    rospy.init_node('height_map_node', anonymous=True)
    HeightMapNode()
    rospy.spin()

if __name__ == '__main__':
    main()
