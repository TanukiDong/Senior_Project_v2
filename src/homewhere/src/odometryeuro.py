#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
import math
from sensor_msgs.msg import JointState

# Wheel base distance
WHEEL_RADIUS = 0.07
# WHEEL_BASE = 0.5 / 2

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
    
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Initial position and orientation
    x = y = theta = 0.0
    rate = rospy.Rate(10)
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        # Calculation
        v = velFrontLeft_Linear * WHEEL_RADIUS
        vx = v * math.cos(theta)
        vy = v * math.sin(theta)
        omega = velFrontLeft_Angular

        dx = vx * dt
        dy = vy * dt
        dtheta = omega * dt
        
        x += dx
        y += dy
        theta += dtheta

        # Create a quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, theta),
            current_time,
            "base_footprint",
            "odom"
        )
        
        odom_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, -theta),
            current_time,
            "base_link",
            "base_footprint"
        )
        # odom_broadcaster.sendTransform(
        #     (0.0, 0.0, 0.0),
        #     tf.transformations.quaternion_from_euler(0, 0, -theta),
        #     current_time,
        #     "base_scan",
        #     "base_link"
        # )

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
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        # Publish the odometry message
        odom_pub.publish(odom)
        
        
        # Create a JointState message
        joint_state = JointState()
        joint_state.header.stamp = current_time

        # Define joint names
        joint_state.name = [
            "wheel_front_left_joint", "wheel_front_right_joint",
            "wheel_rear_left_joint", "wheel_rear_right_joint",
            "steer_front_left_joint", "steer_front_right_joint",
            "steer_rear_left_joint", "steer_rear_right_joint"
        ]

        # Define joint positions
        joint_state.position = [
            0.0, 0.0,
            0.0, 0.0,
            theta, theta, theta, theta
            # 0.0, 0.0,
            # 0.0, 0.0,
        ]

        # Publish joint states
        joint_pub.publish(joint_state)

        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass