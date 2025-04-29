#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from std_msgs.msg import Int16, Float32

import signal
import sys

class Control:
    def __init__(self):
        rospy.init_node('control')

        # Publishers for left and right wheel velocity commands
        self.front_left_velocity_publisher = rospy.Publisher('/cmd_vel_front_left', Twist, queue_size=10)
        self.front_right_velocity_publisher = rospy.Publisher('/cmd_vel_front_right', Twist, queue_size=10)
        self.back_left_velocity_publisher = rospy.Publisher('/cmd_vel_back_left', Twist, queue_size=10)
        self.back_right_velocity_publisher = rospy.Publisher('/cmd_vel_back_right', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.theta_publisher = rospy.Publisher('/cmd_angle', Int16, queue_size=10)
        
        self.active_keys = set()
        self.manual_control_active = False

        # Initialize velocity messages for each wheel
        self.front_left_velocity = Twist()
        self.front_right_velocity = Twist()
        self.back_left_velocity = Twist()
        self.back_right_velocity = Twist()
        self.angle = 90
        self.manual_velocity = 0.2

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

        # Ctrl+C interceptor
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def shutdown_handler(self, signum, frame):
        """Handles Ctrl+C and stops the robot safely."""
        rospy.loginfo("Shutdown initiated! Stopping the robot.")
        self.stop_robot()
        sys.exit(0)

    def stop_robot(self):
        """Stops the robot by publishing zero velocities."""
        self.set_velocity([[0.0, 0.0, 0.0, 0.0],0])
        self.publish_velocity()

    def rad2deg(self, rad):
        return int(rad/math.pi*180)

    def get_v_and_theta(self, vx, vy):
        v = (vx**2 + vy**2)**0.5
        theta = math.atan2(vy, vx)
        return v, theta

    def real_velocity(self, v, theta):
        if -math.pi/2 <= theta and theta < math.pi/2:
            return v
        return -v  
    
    def real_theta(self, theta):
        if theta >= math.pi/2:
            return theta - math.pi
        elif theta < -math.pi/2:
            return theta + math.pi
        return theta

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel messages from move_base."""

        # Extract linear and angular velocities from the cmd_vel message
        vx = msg.linear.x
        vy = msg.linear.y

        # rospy.loginfo(f"vx = {vx}, vy = {vy}")
        
        v, theta = self.get_v_and_theta(vx, vy)

        # rospy.loginfo(f"v = {v}, theta = {theta}")

        real_v = self.real_velocity(v,theta)
        real_t = self.real_theta(theta)
        real_t = self.rad2deg(real_t)

        # rospy.loginfo(f"v = {real_v}, theta = {real_t}")

        # Apply the computed velocities
        self.set_velocity([[real_v]*4, real_t])

        # Publish the updated velocities to each wheel
        self.publish_velocity()

    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.front_left_velocity_publisher.publish(self.front_left_velocity)
        self.front_right_velocity_publisher.publish(self.front_right_velocity)
        self.back_left_velocity_publisher.publish(self.back_left_velocity)
        self.back_right_velocity_publisher.publish(self.back_right_velocity)

        self.theta_publisher.publish(int(self.angle))

    def set_velocity(self, vel):
        """Set velocity for each individual wheel."""

        self.front_left_velocity.linear.x = vel[0][0]
        self.front_right_velocity.linear.x = vel[0][1]
        self.back_left_velocity.linear.x = vel[0][2]
        self.back_right_velocity.linear.x = vel[0][3]

        self.angle = vel[1]
            
    def on_press(self, key):
        """Handle key press events."""
        self.manual_control_active = True

        try:
            still = [0]*4
            forward = [self.manual_velocity]*4
            backward = [-self.manual_velocity]*4

            # Add the pressed key to the active keys set
            self.active_keys.add(key.char)

            # Compute velocity and angular velocity based on active keys
            velocity = still
            angle = 0

            if 'w' in self.active_keys:  # Forward
                velocity = forward
            if 's' in self.active_keys:  # Backward
                velocity = backward
            if 'a' in self.active_keys:  # Left turn
                velocity = forward
                angle = 90
            if 'd' in self.active_keys:  # Right turn
                velocity = forward
                angle = -90
            if 'z' in self.active_keys:  # Left turn
                velocity = backward
                angle = -45
            if 'x' in self.active_keys:  # Right turn
                velocity = backward
                angle = 45

            self.set_velocity([velocity, angle])
            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
        self.active_keys.discard(key.char)
        self.set_velocity([[0.0, 0.0, 0.0, 0.0],0])
        self.publish_velocity()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = Control()
        controller.run()
    except rospy.ROSInterruptException:
        pass
