#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
import math

# Wheel base distance (distance between left and right wheels)
WHEEL_DISTANCE = 0.5
WHEEL_RADIUS = 0.07

# Initialize global variables for wheel velocities
velFrontLeft_Linear = 0.0
velFrontLeft_Angular = 0.0
velFrontRight_Linear = 0.0
velFrontRight_Angular = 0.0
velBackLeft_Linear = 0.0
velBackLeft_Angular = 0.0
velBackRight_Linear = 0.0
velBackRight_Angular = 0.0

# Callback functions to update wheel velocities
def front_left_callback(msg):
    global velFrontLeft_Linear, velFrontLeft_Angular
    velFrontLeft_Linear = msg.linear.x
    velFrontLeft_Angular = msg.angular.z

def front_right_callback(msg):
    global velFrontRight_Linear, velFrontRight_Angular
    velFrontRight_Linear = msg.linear.x
    velFrontRight_Angular = msg.angular.z

def back_left_callback(msg):
    global velBackLeft_Linear, velBackLeft_Angular
    velBackLeft_Linear = msg.linear.x
    velBackLeft_Angular = msg.angular.z

def back_right_callback(msg):
    global velBackRight_Linear, velBackRight_Angular
    velBackRight_Linear = msg.linear.x
    velBackRight_Angular = msg.angular.z

def odometry_publisher():
    rospy.init_node('odometry')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # Subscribe to each wheel's velocity topic
    rospy.Subscriber("/cmd_vel_front_left", Twist, front_left_callback)
    rospy.Subscriber("/cmd_vel_front_right", Twist, front_right_callback)
    rospy.Subscriber("/cmd_vel_back_left", Twist, back_left_callback)
    rospy.Subscriber("/cmd_vel_back_right", Twist, back_right_callback)

    # Initial position and orientation
    x = y = theta = 0.0
    rate = rospy.Rate(10)  # 10 Hz
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        # Calculate average velocities for left and right wheel sets
        left_avg = (velFrontLeft_Linear + velFrontRight_Linear) / 2 * WHEEL_RADIUS
        right_avg = (velBackLeft_Linear + velBackRight_Angular) / 2 * WHEEL_RADIUS

        # Calculate linear and angular velocities
        dv = velFrontLeft_Linear * WHEEL_RADIUS  # Linear velocity
        dth = velFrontLeft_Angular  # Angular velocity

        # Update position
        dx = dv * math.cos(theta) * dt
        dy = dv * math.sin(theta) * dt
        dth = dth * dt
        x += dx
        y += dy
        theta += dth

        # Skip publishing if state hasn't changed significantly
        # if last_x == x and last_y == y and last_theta == theta:
        #     rate.sleep()
        #     continue

        # last_x, last_y, last_theta = x, y, theta

        # Create a quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        # Publish the transform over TF
        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = dv
        odom.twist.twist.angular.z = dth

        # Publish the odometry message
        odom_pub.publish(odom)

        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass
