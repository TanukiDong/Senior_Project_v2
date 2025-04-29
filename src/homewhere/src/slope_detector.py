#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import tf
import math

class SlopeDetector:
    def __init__(self):
        rospy.init_node("slope_detector")

        self.slope_th = rospy.get_param("~slope_th", 5.0)
        self.pub = rospy.Publisher("/on_slope", Bool, queue_size=10)

        self.latest_pitch = 0.0
        self.on_slope = False

        rospy.Subscriber("/imu", Imu, self.imu_callback)

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.pub.publish(Bool(data=self.on_slope))
            rate.sleep()

    def imu_callback(self, msg):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        try:
            (roll, pitch, _) = tf.transformations.euler_from_quaternion(quaternion)
            roll_deg  = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            
            self.latest_roll  = roll_deg
            self.latest_pitch = pitch_deg
            
            self.on_slope = abs(pitch_deg) > self.slope_th or abs(roll_deg) > self.slope_th
        except Exception as e:
            rospy.logwarn("Error converting quaternion: %s", str(e))
            self.on_slope = False


if __name__ == "__main__":
    try:
        SlopeDetector()
    except rospy.ROSInterruptException:
        pass
