#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ServoSim:
    def __init__(self):
        rospy.init_node('servo_sim')

        # Store the current & target angles (radians) for each steer joint
        self.current = 0.0
        self.target = 0.0

        # Subscribers to target angles
        self.sub_front_left  = rospy.Subscriber('/cmd_steer',  Float64, self.cb)

        # Publishers for servo velocities
        self.pub_front_left_vel  = rospy.Publisher('/cmd_servo_vel_front_left',  Twist, queue_size=10)
        self.pub_front_right_vel = rospy.Publisher('/cmd_servo_vel_front_right', Twist, queue_size=10)
        self.pub_back_left_vel   = rospy.Publisher('/cmd_servo_vel_back_left',   Twist, queue_size=10)
        self.pub_back_right_vel  = rospy.Publisher('/cmd_servo_vel_back_right',  Twist, queue_size=10)

        # Desired servo speed in rad/s (positive or negative)
        # 0.2 sec per 60 deg -> pi/3 rad per 0.2 sec => 5.23 rad/s
        self.servo_speed = math.pi/3/0.2  # Example max

        # Loop rate = 10 Hz
        self.rate = rospy.Rate(100)

        rospy.loginfo("ServoSim node started. Converting target angles to servo velocities.")

    # Callback functions: set the target angle
    def cb(self, msg):
        self.target = msg.data

    def run(self):
        # Each iteration, check if current angle is within some threshold of target.
        # If not, command ±servo_speed; else command 0.
        while not rospy.is_shutdown():
            dt = 0.01  # Because we run at 10Hz

            # For each wheel, compute the needed velocity
            vel = self.compute_steer_velocity(dt)

            # Publish a Twist with angular.z = velocity
            self.publish_vel('front_left',  vel)
            self.publish_vel('front_right', vel)
            self.publish_vel('back_left',   vel)
            self.publish_vel('back_right',  vel)

            self.rate.sleep()

    def compute_steer_velocity(self, dt):
        """Calculate the velocity needed to get from current angle to target angle."""
        cur = self.current
        tgt = self.target

        # print("Cur, Tar",cur, tgt)

        # If close enough, no movement
        tolerance = 0.04
        diff = tgt - cur

        if abs(diff) < tolerance:
            # Already near target
            vel = 0.0
        else:
            # Move in the sign of the difference
            vel = self.servo_speed if diff > 0 else -self.servo_speed

        # Update the "potentiometer" reading
        self.current += vel * dt
        return vel

    def publish_vel(self, name, vel):
        """Publish a Twist msg with angular.z = vel for the given joint's servo topic."""
        twist = Twist()
        twist.angular.z = vel
        if name == 'front_left':
            self.pub_front_left_vel.publish(twist)
        elif name == 'front_right':
            self.pub_front_right_vel.publish(twist)
        elif name == 'back_left':
            self.pub_back_left_vel.publish(twist)
        elif name == 'back_right':
            self.pub_back_right_vel.publish(twist)
        # print(vel)

if __name__ == '__main__':
    try:
        node = ServoSim()
        node.run()
    except rospy.ROSInterruptException:
        pass