#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
import math
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.07

vx_cmd = 0.0
vy_cmd = 0.0

def cmd_vel_callback(msg):
    global vx_cmd, vy_cmd
    vx_cmd = msg.linear.x
    vy_cmd = msg.linear.y

def odometry_publisher():
    rospy.init_node('odometry')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # Initial position and orientation
    x = y = theta = 0.0
    rate = rospy.Rate(10)
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        vx = vx_cmd
        vy = vy_cmd

        x += vx * dt
        y += vy * dt

        # Create a quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
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
        
        odom_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            current_time,
            "laser",
            "base_link"
        )

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy

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
