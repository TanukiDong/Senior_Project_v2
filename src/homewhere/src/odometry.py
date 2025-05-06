#!/usr/bin/env python
# odometry_node.py

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int16

WHEEL_RADIUS = 0.0695          # [m]
UPDATE_HZ    = 10.0            # loop rate
DISP_FACTOR  = 1.83            # divide dl to tame drift

class OdometryPublisher:
    """Publishes /odom and /joint_states for a swerve‑drive robot."""

    def __init__(self):
        rospy.init_node("odometry")

        # publishers / broadcasters
        self.odom_pub   = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.joint_pub  = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.tf_br      = tf.TransformBroadcaster()

        # state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0                 # not used yet, keep for future yaw integration

        self.vx_cmd = 0.0                # from /cmd_vel (not used but kept)
        self.vy_cmd = 0.0

        self.dl  = 0.0                   # wheel travel [m]   (avg of wheels from HW node)
        self.rpm = 0.0                   # wheel speed  [rpm]
        self.servo_theta = 0.0           # steering angle [rad]

        self.last_time = rospy.Time.now()

        # subscribers
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb)
        rospy.Subscriber("/cmd_hardware_reading",
                         Float32MultiArray, self._hardware_cb)
        rospy.Subscriber("/cmd_angle", Int16, self._angle_cb)

        # run periodic update
        self.timer = rospy.Timer(rospy.Duration(1.0 / UPDATE_HZ), self._update)

        rospy.loginfo("OdometryPublisher initialised.")

    # ──────────────────────────────────── callbacks ────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self.vx_cmd = msg.linear.x
        self.vy_cmd = msg.linear.y

    def _hardware_cb(self, msg: Float32MultiArray):
        """Message layout: [average_dl, average_rpm]  (see hardware_controller)."""
        self.dl, self.rpm = msg.data

    def _angle_cb(self, msg: Int16):
        self.servo_theta = math.radians(msg.data)   # degrees → rad

    # ─────────────────────────────────── update loop ───────────────────────────────────
    def _update(self, event):
        """Integrate position, publish TF, Odometry, JointState."""
        # linear displacement expressed in map frame
        dx = self.dl * math.cos(self.servo_theta)
        dy = self.dl * math.sin(self.servo_theta)

        self.x += dx / DISP_FACTOR
        self.y += dy / DISP_FACTOR

        # print("x,y = ", self.x, self.y)

        # instantaneous velocities
        v  = self.rpm * 2 * math.pi / 60 * WHEEL_RADIUS
        vx = v * math.cos(self.servo_theta)
        vy = v * math.sin(self.servo_theta)

        now = rospy.Time.now()

        # ── TF transforms ────────────────────────────────────────────────────────────
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        self.tf_br.sendTransform((self.x, self.y, 0.0), odom_quat,
                                 now, "base_footprint", "odom")

        # base_link coincides with base_footprint in this model (no roll/pitch)
        self.tf_br.sendTransform((0.0, 0.0, 0.0),
                                 tf.transformations.quaternion_from_euler(0, 0, -self.theta),
                                 now, "base_link", "base_footprint")

        # laser frame
        self.tf_br.sendTransform((0.0, 0.0, 0.0),
                                 tf.transformations.quaternion_from_euler(0, 0, 0),
                                 now, "laser", "base_link")

        # ── Odometry message ─────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        self.odom_pub.publish(odom)

        # ── JointState (steering only — wheel rotations not tracked here) ────────────
        js = JointState()
        js.header.stamp = now
        js.name = [
            "wheel_front_left_joint", "wheel_front_right_joint",
            "wheel_rear_left_joint",  "wheel_rear_right_joint",
            "steer_front_left_joint", "steer_front_right_joint",
            "steer_rear_left_joint",  "steer_rear_right_joint"
        ]
        js.position = [
            0.0, 0.0, 0.0, 0.0,          # wheels
            self.theta, self.theta, self.theta, self.theta  # steering joints
        ]
        self.joint_pub.publish(js)

    # ────────────────────────────────────── main ─────────────────────────────────────
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        OdometryPublisher().run()
    except rospy.ROSInterruptException:
        pass
