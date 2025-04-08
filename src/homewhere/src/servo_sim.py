#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ServoSim:
    def __init__(self):
        rospy.init_node('servo_sim')

        # Store the current & target angles (radians) for each steer joint
        self.current = {
            'front_left':  0.0,
            'front_right': 0.0,
            'back_left':   0.0,
            'back_right':  0.0
        }
        self.target = {
            'front_left':  0.0,
            'front_right': 0.0,
            'back_left':   0.0,
            'back_right':  0.0
        }

        # Subscribers to target angles
        self.sub_front_left  = rospy.Subscriber('/cmd_steer_front_left',  Float64, self.cb_front_left)
        self.sub_front_right = rospy.Subscriber('/cmd_steer_front_right', Float64, self.cb_front_right)
        self.sub_back_left   = rospy.Subscriber('/cmd_steer_back_left',   Float64, self.cb_back_left)
        self.sub_back_right  = rospy.Subscriber('/cmd_steer_back_right',  Float64, self.cb_back_right)

        # Publishers for servo velocities
        self.pub_front_left_vel  = rospy.Publisher('/cmd_servo_vel_front_left',  Twist, queue_size=10)
        self.pub_front_right_vel = rospy.Publisher('/cmd_servo_vel_front_right', Twist, queue_size=10)
        self.pub_back_left_vel   = rospy.Publisher('/cmd_servo_vel_back_left',   Twist, queue_size=10)
        self.pub_back_right_vel  = rospy.Publisher('/cmd_servo_vel_back_right',  Twist, queue_size=10)

        # Desired servo speed in rad/s (positive or negative)
        # 0.2 sec per 60 deg -> pi/3 rad per 0.2 sec => 5.23 rad/s
        self.servo_speed = 5.23  # Example max

        # Loop rate = 10 Hz
        self.rate = rospy.Rate(10)

        rospy.loginfo("ServoSim node started. Converting target angles to servo velocities.")

    # Callback functions: set the target angle
    def cb_front_left(self, msg):
        self.target['front_left'] = msg.data

    def cb_front_right(self, msg):
        self.target['front_right'] = msg.data

    def cb_back_left(self, msg):
        self.target['back_left'] = msg.data

    def cb_back_right(self, msg):
        self.target['back_right'] = msg.data

    def run(self):
        # Each iteration, check if current angle is within some threshold of target.
        # If not, command ±servo_speed; else command 0.
        while not rospy.is_shutdown():
            dt = 0.1  # Because we run at 10Hz

            # For each wheel, compute the needed velocity
            fl_vel = self.compute_steer_velocity('front_left',  dt)
            fr_vel = self.compute_steer_velocity('front_right', dt)
            bl_vel = self.compute_steer_velocity('back_left',   dt)
            br_vel = self.compute_steer_velocity('back_right',  dt)

            # Publish a Twist with angular.z = velocity
            self.publish_vel('front_left',  fl_vel)
            self.publish_vel('front_right', fr_vel)
            self.publish_vel('back_left',   bl_vel)
            self.publish_vel('back_right',  br_vel)

            self.rate.sleep()

    def compute_steer_velocity(self, name, dt):
        """Calculate the velocity needed to get from current angle to target angle."""
        cur = self.current[name]
        tgt = self.target[name]

        # If close enough, no movement
        tolerance = 0.01
        diff = tgt - cur

        if abs(diff) < tolerance:
            # Already near target
            vel = 0.0
        else:
            # Move in the sign of the difference
            vel = self.servo_speed if diff > 0 else -self.servo_speed

        # Update the "potentiometer" reading
        self.current[name] += vel * dt
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

if __name__ == '__main__':
    try:
        node = ServoSim()
        node.run()
    except rospy.ROSInterruptException:
        pass
