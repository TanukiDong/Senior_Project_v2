#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import tf
import math

class SlopeDetector:
    def __init__(self):
        rospy.init_node("slope_detector")

        self.pitch_threshold_deg = rospy.get_param("~pitch_threshold_deg", 5.0)
        self.pub = rospy.Publisher("/on_slope", Bool, queue_size=10)

        self.latest_pitch = 0.0
        self.on_slope = False

        rospy.Subscriber("/imu", Imu, self.imu_callback)

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.pub.publish(Bool(data=self.on_slope))
            # rospy.loginfo_throttle(1.0, "Pitch: %.2f°, On Slope: %s", self.latest_pitch, str(self.on_slope))
            rate.sleep()

    def imu_callback(self, msg):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        try:
            (_, pitch, _) = tf.transformations.euler_from_quaternion(quaternion)
            pitch_deg = math.degrees(pitch)
            self.latest_pitch = pitch_deg
            self.on_slope = abs(pitch_deg) > self.pitch_threshold_deg
        except Exception as e:
            rospy.logwarn("Error converting quaternion: %s", str(e))
            self.on_slope = False


if __name__ == "__main__":
    try:
        SlopeDetector()
    except rospy.ROSInterruptException:
        pass
