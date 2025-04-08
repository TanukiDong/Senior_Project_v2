#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
import math
import traceback
from sensor_msgs.msg import JointState

# Wheel base distance
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

def clamp_steering(angle, min_angle=-math.pi/2, max_angle=math.pi/2):
    """Clamp steering angles to be within [-π/2, π/2]"""
    return max(min(angle, max_angle), min_angle)

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
        try:
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

            # Check if steering is at its limit
            steering_angle = clamp_steering(theta)
            at_limit = abs(steering_angle) >= (math.pi / 2)

            if at_limit and ((theta > 0 and dtheta > 0) or (theta < 0 and dtheta < 0)):
                # Stop rotation if trying to go further in the same direction
                dtheta = 0
            else:
                # Allow instant switching if direction changes
                theta += dtheta  

            # Create a quaternion from theta
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

            try:
                odom_broadcaster.sendTransform(
                    (x, y, 0.0),
                    tf.transformations.quaternion_from_euler(0, 0, theta % (2 * math.pi)),  # Limit theta
                    current_time,
                    "base_footprint",
                    "odom"
                )
            except Exception as e:
                rospy.logerr("Error broadcasting transform (base_footprint -> odom): {}".format(e))
                rospy.logerr(traceback.format_exc())

            try:
                odom_broadcaster.sendTransform(
                    (0.0, 0.0, 0.0),
                    tf.transformations.quaternion_from_euler(0, 0, -theta),
                    current_time,
                    "base_link",
                    "base_footprint"
                )
            except Exception as e:
                rospy.logerr("Error broadcasting transform (base_link -> base_footprint): {}".format(e))
                rospy.logerr(traceback.format_exc())

            # Publish the odometry message
            try:
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
                odom.twist.twist.angular.z = omega if not at_limit else 0.0  # Stop rotation if at limit

                # Publish the odometry message
                odom_pub.publish(odom)
            except Exception as e:
                rospy.logerr("Error publishing odometry message: {}".format(e))
                rospy.logerr(traceback.format_exc())

            # Create a JointState message
            try:
                joint_state = JointState()
                joint_state.header.stamp = current_time

                # Define joint names
                joint_state.name = [
                    "wheel_front_left_joint", "wheel_front_right_joint",
                    "wheel_rear_left_joint", "wheel_rear_right_joint",
                    "steer_front_left_joint", "steer_front_right_joint",
                    "steer_rear_left_joint", "steer_rear_right_joint"
                ]

                wheel_rotation = (x / WHEEL_RADIUS) % (2 * math.pi)  # Limit wheel rotation

                joint_state.position = [
                    wheel_rotation,  # Front left wheel rotation
                    wheel_rotation,  # Front right wheel rotation
                    wheel_rotation,  # Rear left wheel rotation
                    wheel_rotation,  # Rear right wheel rotation
                    clamp_steering(theta),  # Steering front left
                    clamp_steering(theta),  # Steering front right
                    clamp_steering(theta),  # Steering rear left
                    clamp_steering(theta)   # Steering rear right
                ]

                # Publish joint states
                joint_pub.publish(joint_state)
            except Exception as e:
                rospy.logerr("Error publishing joint states: {}".format(e))
                rospy.logerr(traceback.format_exc())

            rate.sleep()

        except Exception as e:
            pass

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Node interrupted")
    except Exception as e:
        rospy.logerr("Unexpected error in main function: {}".format(e))
        rospy.logerr(traceback.format_exc())
