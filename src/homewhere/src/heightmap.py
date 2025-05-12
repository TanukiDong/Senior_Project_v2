#!/usr/bin/env python
import rospy
import math
import numpy as np

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2

import tf
import tf2_ros

class HeightMapNode:
    def __init__(self):
        # --- Map Parameters ---
        self.map_size = 30.0
        self.resolution = 0.1
        self.origin_x = -self.map_size / 2.0
        self.origin_y = -self.map_size / 2.0
        self.flat_th = 0.02
        self.ramp_th = 0.15
        self.wall_th = 0.30  # only gradients this steep are walls

        self.grid_width = int(self.map_size / self.resolution)
        self.grid_height = int(self.map_size / self.resolution)

        self.height_map = np.full((self.grid_width, self.grid_height), np.nan)
        self.cell_timestamp = np.full((self.grid_width, self.grid_height), rospy.get_time())

        # Robot pose from odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Subscribers
        rospy.sleep(2.0)
        rospy.Subscriber('/scan_cloud_full', PointCloud2, self.cloud_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publishers
        self.map_pub = rospy.Publisher('/height_map', OccupancyGrid, queue_size=1)
        self.marker_pub = rospy.Publisher('/height_map_markers', MarkerArray, queue_size=1)
        self.gradient_marker_pub = rospy.Publisher('/gradient_map_markers', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(0.2), self.publish_map)

        #rospy.loginfo("✅ HeightMapNode initialized and running.")

    # -----------------------------
    # Callbacks
    # -----------------------------

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = euler[2]

    def cloud_callback(self, cloud_msg):
        # Convert PointCloud2 to list of points
        points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

        if not points:
            rospy.logwarn("⚠️ Received empty point cloud!")
            return  # Skip processing if no points

        for p in points:
            x_lidar, y_lidar, z_lidar = p

            # Apply IMU correction
            x_corr, y_corr, z_corr =x_lidar, y_lidar, z_lidar

            # Transform to global map frame
            X, Y, Z = self.transform_to_global(x_corr, y_corr, z_corr)

            # Update height map
            self.update_height_map(X, Y, Z)

    # -----------------------------
    # Processing functions
    # -----------------------------

    def transform_to_global(self, x_corr, y_corr, z_corr):
        cy = math.cos(self.robot_yaw)
        sy = math.sin(self.robot_yaw)

        X = self.robot_x + (x_corr * cy - y_corr * sy)
        Y = self.robot_y + (x_corr * sy + y_corr * cy)
        Z = z_corr  # NOTE: no +0.15 here, already transformed

        return X, Y, Z

    # -----------------------------
    # Visualization
    # -----------------------------

    def publish_map(self, event):
        # Clean old cells
        current_time = rospy.get_time()
        cell_lifetime = 30.0  # seconds
        time_since_update = current_time - self.cell_timestamp
        expired_cells = time_since_update > cell_lifetime
        self.height_map[expired_cells] = np.nan

        # Compute maps
        grad_map = self.compute_gradient_map()
        class_map = self.classify_map(grad_map)

        # Count classification statistics
        flat_count = np.sum(class_map == 0)
        ramp_count = np.sum(class_map == 1)
        wall_count = np.sum(class_map == 2)
        total_known = flat_count + ramp_count + wall_count

        # Logging
        rospy.loginfo(f"📊 Max gradient: {np.nanmax(grad_map):.3f}")
        rospy.loginfo(f"🧠 Cells: flat={flat_count}, ramp={ramp_count}, wall={wall_count}, total={total_known}")
        if total_known > 0:
            rospy.loginfo(f"📈 Percentages: flat={flat_count/total_known*100:.1f}%, ramp={ramp_count/total_known*100:.1f}%, wall={wall_count/total_known*100:.1f}%")
        else:
            rospy.loginfo("⚠️ No valid cells yet.")

        # Publish OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = -self.origin_y - self.map_size
        grid_msg.info.origin.position.y = self.origin_x

        grid_msg.info.origin.orientation.w = 1.0

        rotated_map = np.rot90(class_map, k=1)  # 90 degrees counter-clockwise

        data = []
        for row in rotated_map:
            for label in row:
                if label == -1:
                    data.append(-1)
                elif label == 0 or label == 1:
                    data.append(0)
                elif label == 2:
                    data.append(100)




        grid_msg.data = data
        self.map_pub.publish(grid_msg)

        # Publish markers
        self.publish_classification_markers(class_map)
        self.publish_gradient_markers(grad_map)

    def update_height_map(self, X, Y, Z):
        i = int((X - self.origin_x) / self.resolution)
        j = int((Y - self.origin_y) / self.resolution)

        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            existing = self.height_map[i, j]
            if np.isnan(existing):
                self.height_map[i, j] = Z
            elif abs(Z - existing) > 0.02:  # was 0.05; allow finer updates
                self.height_map[i, j] = Z

            self.cell_timestamp[i, j] = rospy.get_time()

 # -----------------------------
    # Gradient and Classification
    # -----------------------------
    def compute_gradient_map(self):
        raw_grad = np.full((self.grid_width, self.grid_height), np.nan)

        # Step 1: Compute raw central-difference gradients
        for i in range(1, self.grid_width - 1):
            for j in range(1, self.grid_height - 1):
                if all(not np.isnan(self.height_map[x, y]) for x, y in [
                    (i-1, j), (i+1, j), (i, j-1), (i, j+1)]):
                    dh_dx = (self.height_map[i+1, j] - self.height_map[i-1, j]) / (2 * self.resolution)
                    dh_dy = (self.height_map[i, j+1] - self.height_map[i, j-1]) / (2 * self.resolution)

                    # Emphasize vertical edges (common for walls) if desired:
                    grad_mag = math.sqrt(dh_dx**2 + (1.5 * dh_dy)**2)
                    raw_grad[i, j] = grad_mag


        # Step 2: Weighted 3x3 blur (center has double weight)
        smoothed_grad = np.full_like(raw_grad, np.nan)
        for i in range(1, self.grid_width - 1):
            for j in range(1, self.grid_height - 1):
                weighted_vals = []
                weights = []
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        val = raw_grad[i + dx, j + dy]
                        if not np.isnan(val):
                            weight = 2.0 if dx == 0 and dy == 0 else 1.0
                            weighted_vals.append(val * weight)
                            weights.append(weight)
                if weights:
                    smoothed_grad[i, j] = sum(weighted_vals) / sum(weights)

        return smoothed_grad



    def classify_map(self, grad_map):
        class_map = np.zeros_like(grad_map, dtype=int)
        flat, ramp, wall = 0, 0, 0

        for i in range(1, grad_map.shape[0] - 1):
            for j in range(1, grad_map.shape[1] - 1):
                g = grad_map[i, j]
                if np.isnan(g):
                    class_map[i, j] = -1
                    continue

                # Count neighbors with strong gradient to reinforce wall certainty
                neighbors = [grad_map[i + dx, j + dy] for dx in [-1, 0, 1] for dy in [-1, 0, 1]
                            if not (dx == 0 and dy == 0) and not np.isnan(grad_map[i + dx, j + dy])]
                wall_like = sum(1 for n in neighbors if n >= self.wall_th)

                if g < self.flat_th:
                    class_map[i, j] = 0
                    flat += 1
                elif g < self.ramp_th:
                    rospy.loginfo(f"[RAMP] grad={g:.3f} at ({i},{j})")
                    class_map[i, j] = 1
                    ramp += 1
                elif g < self.wall_th:
                    # ambiguous zone → if many neighbors are steep, classify as wall
                    if wall_like >= 4:
                        class_map[i, j] = 2
                        wall += 1
                    else:
                        class_map[i, j] = 1
                        ramp += 1
                else:
                    class_map[i, j] = 2
                    wall += 1
                    

        total = flat + ramp + wall
        if total > 0:
            rospy.loginfo(f"🧠 Cells: flat={flat}, ramp={ramp}, wall={wall}, total={total}")
            rospy.loginfo(f"📈 Percentages: flat={flat/total:.1%}, ramp={ramp/total:.1%}, wall={wall/total:.1%}")
        rospy.loginfo(f"📈 Thresholds: flat<{self.flat_th}, ramp<{self.ramp_th}, wall≥{self.wall_th}")
        return class_map


    # -----------------------------
    # Visualization and Publishing
    # -----------------------------
    def publish_gradient_markers(self, grad_map):
        marker_array = MarkerArray()
        marker_id = 0

        for i in range(self.grid_width):
            for j in range(self.grid_height):
                gradient = grad_map[i, j]
                if np.isnan(gradient):
                    continue

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "gradient"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = self.origin_x + (i + 0.5) * self.resolution
                marker.pose.position.y = self.origin_y + (j + 0.5) * self.resolution
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = 0.05
                marker.color.a = 0.85

                # Use shared thresholds
                if gradient < self.flat_th:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0  # green
                elif gradient < self.ramp_th:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0  # yellow
                elif gradient < self.wall_th:
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0  # orange
                else:
                    marker.color.r = 0.9
                    marker.color.g = 0.0
                    marker.color.b = 0.0  # red

                marker_array.markers.append(marker)
                marker_id += 1

        self.gradient_marker_pub.publish(marker_array)



    def publish_classification_markers(self, class_map, ramp_positions=[]):
        marker_array = MarkerArray()
        marker_id = 0
        ramp_pos_set = set(ramp_positions)

        for i in range(self.grid_width):
            for j in range(self.grid_height):
                label = class_map[i, j]
                if label == -1:
                    continue  # skip unknown

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "height_map"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = self.origin_x + (i + 0.5) * self.resolution
                marker.pose.position.y = self.origin_y + (j + 0.5) * self.resolution
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = 0.05
                marker.color.a = 0.9

                if (i, j) in ramp_pos_set:
                    # Highlight manually selected ramp
                    marker.color.r = 1.0
                    marker.color.g = 0.7
                    marker.color.b = 0.0
                elif label == 0:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0     # green = flat
                elif label == 1:
                    marker.color.r = 1.0
                    marker.color.g = 0.6
                    marker.color.b = 0.0     # orange = ramp
                elif label == 2:
                    marker.color.r = 0.9
                    marker.color.g = 0.0
                    marker.color.b = 0.0     # strong red = wall

                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)


# -----------------------------
# Main Entry Point
# -----------------------------

def main():
    rospy.init_node('height_map_node', anonymous=True)
    HeightMapNode()
    rospy.spin()

if __name__ == '__main__':
    main()