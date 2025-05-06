#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# ─── Constants ────────────────────────────────────────────────
WHEEL_RADIUS = 0.07  # (m)

class OdometryPublisher(object):
    def __init__(self):
        rospy.init_node('odometry')

        # ── Wheel-velocity variables ────────────
        self.velFrontLeft_Linear   = 0.0
        self.velFrontRight_Linear  = 0.0
        self.velBackLeft_Linear    = 0.0
        self.velBackRight_Linear   = 0.0
        self.steer_angle           = 0.0

        # ── Pose state ────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.last_time = rospy.Time.now()

        # ── Publishers / TF ───────────────────────────────────
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # ── Subscribers ───────────────────────────────────────
        rospy.Subscriber('/cmd_vel_front_left',  Twist,  self.front_left_callback)
        rospy.Subscriber('/cmd_vel_front_right', Twist,  self.front_right_callback)
        rospy.Subscriber('/cmd_vel_back_left',   Twist,  self.back_left_callback)
        rospy.Subscriber('/cmd_vel_back_right',  Twist,  self.back_right_callback)
        rospy.Subscriber('/cmd_steer',           Float64, self.steer_callback)

        # ── Main update loop (10 Hz) ──────────────────────────
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.on_shutdown(self.shutdown)

    # ── Callbacks to update wheel velocities ──────────────────
    def front_left_callback(self, msg):
        self.velFrontLeft_Linear  = msg.linear.x

    def front_right_callback(self, msg):
        self.velFrontRight_Linear  = msg.linear.x

    def back_left_callback(self, msg):
        self.velBackLeft_Linear  = msg.linear.x

    def back_right_callback(self, msg):
        self.velBackRight_Linear  = msg.linear.x

    def steer_callback(self, msg):
        self.steer_angle = msg.data

    # ── Periodic update (publishes TF, Odometry, JointState) ──
    def update(self, _event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Basic planar kinematics (uses front-left wheel only, as in original)
        v  = self.velFrontLeft_Linear * WHEEL_RADIUS
        vx = v * math.cos(self.steer_angle)
        vy = v * math.sin(self.steer_angle)

        self.x += vx * dt
        self.y += vy * dt

        # ─ TF: odom → base_footprint and base_footprint → base_link
        # odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.steer_angle)
        # odom_quat_reverse = tf.transformations.quaternion_from_euler(0, 0, -self.theta)
        
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            # odom_quat,
            current_time,
            'base_footprint',
            'odom'
        )

        # ─ Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        # odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        # ─ JointState ──
        joint_state = JointState()
        joint_state.header.stamp = current_time
        joint_state.name = [
            'wheel_front_left_joint', 'wheel_front_right_joint',
            'wheel_rear_left_joint',  'wheel_rear_right_joint',
            'steer_front_left_joint', 'steer_front_right_joint',
            'steer_rear_left_joint',  'steer_rear_right_joint'
        ]
        joint_state.position = [
            0.0, 0.0, 0.0, 0.0,
            self.steer_angle, self.steer_angle, self.steer_angle, self.steer_angle
        ]
        self.joint_pub.publish(joint_state)

    # ── Shutdown ───────────────────────────────────
    def shutdown(self):
        rospy.loginfo('Odometry node shutting down.')

# ──── Main ────────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
