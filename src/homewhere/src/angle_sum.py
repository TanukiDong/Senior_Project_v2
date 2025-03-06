#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class CmdVelToDelta:
    def __init__(self):
        rospy.init_node('angle_sum')
        self.last_time = rospy.Time.now()
        self.angle = 0.0  # Initial angle

        # Publisher to send the accumulated angle
        self.delta_pub = rospy.Publisher('/cmd_angle', Float32, queue_size=10)

        # Subscriber to listen to cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        """Integrate angular velocity over time to compute the accumulated angle."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:  # Avoid division errors
            return
        
        self.last_time = current_time

        # Compute angle change
        dtheta = msg.angular.z * dt
        self.angle += dtheta

        # Normalize the angle to stay within -pi to pi
        self.angle = math.atan2(math.sin(self.angle), math.cos(self.angle))

        # Publish the accumulated angle wrapped inside Float32
        angle_msg = Float32()
        angle_msg.data = self.angle
        self.delta_pub.publish(angle_msg)

if __name__ == '__main__':
    CmdVelToDelta()
    rospy.spin()
