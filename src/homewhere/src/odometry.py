#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
import math
from sensor_msgs.msg import JointState

# Constants
WHEEL_RADIUS = 0.07
MAX_STEER_ANGLE = math.pi / 2  # ±90 degrees

# Initialize global variables for wheel velocities
velFrontLeft_Linear = velFrontRight_Linear = velBackLeft_Linear = velBackRight_Linear = 0.0
velFrontLeft_Angular = velFrontRight_Angular = velBackLeft_Angular = velBackRight_Angular = 0.0

# Steering angles
steer_front_left = steer_front_right = steer_rear_left = steer_rear_right = 0.0

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
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # ✅ **Re-adding missing Subscribers**
    rospy.Subscriber("/cmd_vel_front_left", Twist, front_left_callback)
    rospy.Subscriber("/cmd_vel_front_right", Twist, front_right_callback)
    rospy.Subscriber("/cmd_vel_back_left", Twist, back_left_callback)
    rospy.Subscriber("/cmd_vel_back_right", Twist, back_right_callback)

    # Initial position and orientation
    x = y = theta = 0.0
    rate = rospy.Rate(10)
    last_time = rospy.Time.now()

    global steer_front_left, steer_front_right, steer_rear_left, steer_rear_right

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        # Compute velocity from all four wheels
        dv_x = (velFrontLeft_Linear + velFrontRight_Linear + velBackLeft_Linear + velBackRight_Linear) / 4.0 * WHEEL_RADIUS
        omega = (velFrontLeft_Angular + velFrontRight_Angular + velBackLeft_Angular + velBackRight_Angular) / 4.0

        # Update position
        dx = dv_x * math.cos(theta) * dt
        dy = dv_x * math.sin(theta) * dt
        dtheta = omega * dt

        x += dx
        y += dy
        theta += dtheta

        # Update steering joint angles (clamping at ±90 degrees)
        steer_front_left += velFrontLeft_Angular * dt
        steer_front_right += velFrontRight_Angular * dt
        steer_rear_left += velBackLeft_Angular * dt
        steer_rear_right += velBackRight_Angular * dt

        steer_front_left = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_front_left))
        steer_front_right = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_front_right))
        steer_rear_left = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_rear_left))
        steer_rear_right = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_rear_right))

        # Create a quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        # Publish transforms
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

        # ✅ **Fixed: Ensure `/odom` updates with velocity values**
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # ✅ **Ensure the odometry message contains the correct velocity values**
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = dv_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega  # **Ensure the correct angular velocity is set**

        odom_pub.publish(odom)

        # ✅ **Fixed: Ensure `/joint_states` updates correctly**
        joint_state = JointState()
        joint_state.header.stamp = current_time

        # Define joint names
        joint_state.name = [
            "wheel_front_left_joint", "wheel_front_right_joint",
            "wheel_rear_left_joint", "wheel_rear_right_joint",
            "steer_front_left_joint", "steer_front_right_joint",
            "steer_rear_left_joint", "steer_rear_right_joint"
        ]

        # ✅ **Update joint positions with clamped steering angles**
        joint_state.position = [
            0.0, 0.0,  # Wheel rotation
            0.0, 0.0,
            steer_front_left, steer_front_right,  # Clamped steering angles
            steer_rear_left, steer_rear_right
        ]

        # ✅ **Ensure `/joint_states` contains velocity values for better visualization**
        joint_state.velocity = [
            dv_x, dv_x,  # Wheel linear velocity
            dv_x, dv_x,
            velFrontLeft_Angular, velFrontRight_Angular,  # Angular velocity for steering
            velBackLeft_Angular, velBackRight_Angular
        ]

        # ✅ **Publish joint states so RViz updates correctly**
        joint_pub.publish(joint_state)

        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass
