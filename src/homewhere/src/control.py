#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from std_msgs.msg import Int16, Float32

import signal
import sys

# Constants
H = 0.2
W = 0.2

class Control:
    def __init__(self):
        rospy.init_node('control')

        # Publishers for left and right wheel velocity commands
        self.front_left_velocity_publisher = rospy.Publisher('/cmd_vel_front_left', Twist, queue_size=10)
        self.front_right_velocity_publisher = rospy.Publisher('/cmd_vel_front_right', Twist, queue_size=10)
        self.back_left_velocity_publisher = rospy.Publisher('/cmd_vel_back_left', Twist, queue_size=10)
        self.back_right_velocity_publisher = rospy.Publisher('/cmd_vel_back_right', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.theta_fl_publisher = rospy.Publisher('/cmd_angle', Int16, queue_size=10)
        self.theta_fl_publisher = rospy.Publisher('/cmd_angle_front_left', Int16, queue_size=10)
        self.theta_fr_publisher = rospy.Publisher('/cmd_angle_front_right', Int16, queue_size=10)
        self.theta_bl_publisher = rospy.Publisher('/cmd_angle_back_left', Int16, queue_size=10)
        self.theta_br_publisher = rospy.Publisher('/cmd_angle_back_right', Int16, queue_size=10)
        
        self.active_keys = set()
        self.manual_control_active = False

        # Initialize velocity messages for each wheel
        self.front_left_velocity = Twist()
        self.front_right_velocity = Twist()
        self.back_left_velocity = Twist()
        self.back_right_velocity = Twist()
        self.velocity = 0
        self.angle_fl = 90
        self.angle_fr = 90
        self.angle_bl = 90
        self.angle_br = 90
        self.angle = 90
        self.manual_velocity = 0.1

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
        self.set_velocity([[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0], 0])
        self.publish_velocity()

    def rad2deg(self, rad):
        return int(rad/math.pi*180)

    def get_v_and_theta(self, vx, vy):
        """Calculate the swerve speed and angle
                vx: the speed of the CG in the x-direction in m/s
                vy: the speed of the CG in the y-direction in m/s
            return: tuple of (v,theta)"""
        v = (vx**2 + vy**2)**0.5
        theta = math.atan2(vy, vx)
        return v, theta
    
    def get_delta_v_and_theta(self, v, w):
        """Calculate the Ackermann angle and speed difference
                v: the speed of the CG in m/s
                w: the angular velocity in the +z direction in rad/s
            return: tuple of four delta velocities and thetas ((4),(4))"""
        r = 0
        if v == 0 or w == 0:
            return ((0,0,0,0),(0,0,0,0))
        else:
            r = v/w

        hor_fl = r-W if r > 0 else W-r
        hor_fr = r+W if r > 0 else W+r

        dtheta_fl = math.atan(H, hor_fl)
        dtheta_fr = math.atan(H, hor_fr)
        dtheta_bl = -dtheta_fl
        dtheta_br = -dtheta_fr

        dv_l = w*(H**2 + hor_fl)**0.5
        dv_r = w*(H**2 + hor_fr)**0.5

        return ((dv_l,dv_r,dv_l,dv_r),(dtheta_fl,dtheta_fr,dtheta_bl,dtheta_br))

    def real_velocity(self, v, theta):
        if -math.pi/2 <= theta and theta <= math.pi/2:
            return v
        return -v  
    
    def real_theta(self, theta):
        if theta > math.pi/2:
            return theta - math.pi
        elif theta < -math.pi/2:
            return theta + math.pi
        return theta

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel messages from move_base."""

        # Extract linear and angular velocities from the cmd_vel message
        v = msg.linear.x
        w = msg.angular.z
    
        v_list, theta_list = self.get_delta_v_and_theta(v,w)

        real_v = [self.real_velocity(v_i,theta_i) for v_i,theta_i in zip(v_list,theta_list)]
        real_t = [self.real_theta(theta_i) for theta_i in theta_list]
        real_t = [self.rad2deg(theta_i) for theta_i in real_t]

        print("Real",real_v,real_t)

        # Apply the computed velocities
        self.set_velocity([real_v, real_t, v])

        # Publish the updated velocities to each wheel
        self.publish_velocity()

    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.front_left_velocity_publisher.publish(self.front_left_velocity)
        self.front_right_velocity_publisher.publish(self.front_right_velocity)
        self.back_left_velocity_publisher.publish(self.back_left_velocity)
        self.back_right_velocity_publisher.publish(self.back_right_velocity)

        self.theta_fl_publisher.publish(int(self.angle_fl))
        self.theta_fr_publisher.publish(int(self.angle_fr))
        self.theta_bl_publisher.publish(int(self.angle_bl))
        self.theta_br_publisher.publish(int(self.angle_br))

    def set_velocity(self, vel):
        """Set velocity for each individual wheel."""

        self.front_left_velocity.linear.x = vel[0][0]
        self.front_right_velocity.linear.x = vel[0][1]
        self.back_left_velocity.linear.x = vel[0][2]
        self.back_right_velocity.linear.x = vel[0][3]

        self.angle_fl = vel[1][0]
        self.angle_fr = vel[1][1]
        self.angle_bl = vel[1][2]
        self.angle_br = vel[1][3]

        self.velocity = vel[1]
            
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
            angle = [0]*4

            if 'w' in self.active_keys:  # Forward
                velocity = forward
            if 's' in self.active_keys:  # Backward
                velocity = backward
            # TODO: add correct angle
            if 'a' in self.active_keys:  # Left turn
                angle = [90]*4
            if 'd' in self.active_keys:  # Right turn
                angle = [90]*4

            self.set_velocity([velocity, angle, velocity[0]])
            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
        self.active_keys.discard(key.char)
        self.set_velocity([[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0], 0])
        self.publish_velocity()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = Control()
        controller.run()
    except rospy.ROSInterruptException:
        pass
