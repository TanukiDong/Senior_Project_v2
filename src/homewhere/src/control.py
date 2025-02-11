#!/usr/bin/env python

import math
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

        # Extract linear and angular velocities from the cmd_vel message
        linear_velocity = msg.linear.x * 10
        angular_velocity = msg.angular.z

        # Threshold to prevent unnecessary rotation at goal
        ANGULAR_THRESHOLD = 0.1
        LINEAR_THRESHOLD = 0.1
        MAX_ANGLE = math.radians(90)
        
        # Determine if the desired rotation angle exceeds the limit
        if abs(angular_velocity) > MAX_ANGLE:
            # Reverse direction and move backward instead
            angular_velocity = -angular_velocity  # Flip the rotation direction
            linear_velocity = -linear_velocity    # Move in reverse

        # Decision logic for movement
        if abs(linear_velocity) < LINEAR_THRESHOLD and abs(angular_velocity) < ANGULAR_THRESHOLD:
            # **Stop the robot completely**
            velocity = [0.0] * 4
            angular_velocities = [0.0] * 4
        elif abs(linear_velocity) < LINEAR_THRESHOLD:
            # **Only rotate in place**
            velocity = [0.0] * 4
            angular_velocities = [angular_velocity] * 4
        elif abs(angular_velocity) < ANGULAR_THRESHOLD:
            # **Only move forward/backward without turning**
            velocity = [linear_velocity] * 4
            angular_velocities = [0.0] * 4
        else:
            # **Move and rotate at the same time**
            velocity = [linear_velocity] * 4
            angular_velocities = [angular_velocity] * 4

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
            forward = [10, 10, 10, 10]
            backward = [-10, -10, -10, -10]
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
