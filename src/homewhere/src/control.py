#!/usr/bin/env python

import math
from math import sqrt, atan2
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class Control:
    def __init__(self):
        rospy.init_node('control')

        # Publishers for left and right wheel velocity commands
        self.front_left_velocity_publisher = rospy.Publisher('/cmd_vel_front_left', Twist, queue_size=10)
        self.front_right_velocity_publisher = rospy.Publisher('/cmd_vel_front_right', Twist, queue_size=10)
        self.back_left_velocity_publisher = rospy.Publisher('/cmd_vel_back_left', Twist, queue_size=10)
        self.back_right_velocity_publisher = rospy.Publisher('/cmd_vel_back_right', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.active_keys = set()
        self.manual_control_active = False

        # Robot geometry
        self.wheelbase = 0.25   # front-to-back distance
        self.trackwidth = 0.25  # left-to-right distance
        self.L = self.wheelbase / 2.0
        self.W = self.trackwidth / 2.0
        self.R = math.sqrt(self.L**2 + self.W**2)

        # Initialize velocity messages for each wheel
        self.front_left_velocity = Twist()
        self.front_right_velocity = Twist()
        self.back_left_velocity = Twist()
        self.back_right_velocity = Twist()


        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel messages from move_base."""

        # Extract forward (x) and strafe (y)
        x = msg.linear.x
        y  = msg.linear.y

        # We ignore rotation => rot = 0
        # rot = 0.0

        # # Swerve math: we only combine forward & strafe
        # # y,y: side offsets, x,x: front offsets
        # y = strafe #- rot * self.L / self.R  # but rot=0 => y=strafe
        # y = strafe #+ rot * self.L / self.R  # => y=strafe
        # x = forward # rot * self.W / self.R # => x=forward
        # x = forward #+ rot * self.W / self.R # => x=forward

        # Wheel speeds
        wheel_speed = math.sqrt(y*y + x*x)
        wheel_angle = math.atan2(y, x)

        front_right_speed = wheel_speed
        front_left_speed  = wheel_speed
        back_left_speed   = wheel_speed
        back_right_speed  = wheel_speed

        # Wheel angles
        front_right_angle = wheel_angle
        front_left_angle  = wheel_angle
        back_left_angle   = wheel_angle
        back_right_angle  = wheel_angle


        # Threshold to prevent unnecessary rotation at goal
        # ANGULAR_THRESHOLD = 0.0
        # LINEAR_THRESHOLD = 0.0
        # MAX_ANGLE = math.pi / 2
        
        # # Determine if the desired rotation angle exceeds the limit
        # if abs(angular_velocity) > MAX_ANGLE:
        #     if angular_velocity > MAX_ANGLE:
        #         angular_velocity -= math.pi
        #     else: # angular_velocity < MAX_ANGLE
        #         angular_velocity += math.pi
                
        #     linear_velocity = -linear_velocity

        # Decision logic for movement
        # if abs(linear_velocity) < LINEAR_THRESHOLD and abs(angular_velocity) < ANGULAR_THRESHOLD:
        #     # **Stop the robot completely**
        #     velocity = [0.0] * 4
        #     angular_velocities = [0.0] * 4
        # elif abs(linear_velocity) < LINEAR_THRESHOLD:
        #     # **Only rotate in place**
        #     velocity = [0.0] * 4
        #     angular_velocities = [angular_velocity] * 4
        # elif abs(angular_velocity) < ANGULAR_THRESHOLD:
        #     # **Only move forward/backward without turning**
        #     velocity = [linear_velocity] * 4
        #     angular_velocities = [0.0] * 4
        # else:
        #     # **Move and rotate at the same time**
        velocity = [front_left_speed,front_right_speed,back_left_speed,back_right_speed]
        angular_velocities = [front_left_angle,front_right_angle,back_left_angle,back_right_angle]

        # Apply the computed velocities
        self.set_velocity([velocity, angular_velocities])

        # Publish the updated velocities to each wheel
        self.publish_velocity()

    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.front_left_velocity_publisher.publish(self.front_left_velocity)
        self.front_right_velocity_publisher.publish(self.front_right_velocity)
        self.back_left_velocity_publisher.publish(self.back_left_velocity)
        self.back_right_velocity_publisher.publish(self.back_right_velocity)

    def set_velocity(self, vel):
        """Set velocity for each individual wheel."""

        self.front_left_velocity.linear.x = vel[0][0]
        self.front_right_velocity.linear.x = vel[0][1]
        self.back_left_velocity.linear.x = vel[0][2]
        self.back_right_velocity.linear.x = vel[0][3]

        self.front_left_velocity.angular.z = vel[1][0]
        self.front_right_velocity.angular.z = vel[1][1]
        self.back_left_velocity.angular.z = vel[1][2]
        self.back_right_velocity.angular.z = vel[1][3]
            
    def on_press(self, key):
        """Handle key press events."""
        self.manual_control_active = True

        try:
            still = [0, 0, 0, 0]
            forward = [5, 5, 5, 5]
            backward = [-5, -5, -5, -5]
            left_turn = [1, 1, 1, 1]
            right_turn = [-1, -1, -1, -1]

            # Add the pressed key to the active keys set
            self.active_keys.add(key.char)

            # Compute velocity and angular velocity based on active keys
            velocity = still
            angular_velocity = still

            if 'w' in self.active_keys:  # Forward
                velocity = forward
            if 's' in self.active_keys:  # Backward
                velocity = backward
            if 'a' in self.active_keys:  # Left turn
                angular_velocity = left_turn
            if 'd' in self.active_keys:  # Right turn
                angular_velocity = right_turn

            self.set_velocity([velocity, angular_velocity])
            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
        self.active_keys.discard(key.char)
        self.set_velocity([[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]])
        self.publish_velocity()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = Control()
        controller.run()
    except rospy.ROSInterruptException:
        pass
