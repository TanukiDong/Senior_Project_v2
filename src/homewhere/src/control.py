#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from std_msgs.msg import Float64

class Control:
    def __init__(self):
        rospy.init_node('control')

        # Publishers for left and right wheel velocity commands
        self.front_left_velocity_publisher = rospy.Publisher('/cmd_vel_front_left', Twist, queue_size=10)
        self.front_right_velocity_publisher = rospy.Publisher('/cmd_vel_front_right', Twist, queue_size=10)
        self.back_left_velocity_publisher = rospy.Publisher('/cmd_vel_back_left', Twist, queue_size=10)
        self.back_right_velocity_publisher = rospy.Publisher('/cmd_vel_back_right', Twist, queue_size=10)
        self.steer_angle_publisher = rospy.Publisher('/cmd_steer', Float64, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.active_keys = set()
        self.manual_control_active = False

        # Initialize velocity messages for each wheel
        self.front_left_velocity = Twist()
        self.front_right_velocity = Twist()
        self.back_left_velocity = Twist()
        self.back_right_velocity = Twist()
        self.angle = 0.0
        self.hemisphere = "front" # front or back for the servo

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel messages from move_base."""

        # Extract linear and angular velocities from the cmd_vel message
        vx = msg.linear.x * 5
        vy = msg.linear.y * 5

        # Get actual v, and theta
        v = (vx**2 + vy**2)**0.5
        theta = math.atan2(vy,vx)

        # Tolerances
        if self.hemisphere == "front":
            # Beyond the boundaries
            if theta < -math.pi/2 - 0.2:
                theta += math.pi
                v *= -1
                self.hemisphere = "back"
            elif theta > math.pi/2 + 0.2:
                theta -= math.pi
                v *= -1
                self.hemisphere = "back"
            else:
                pass

        else:
            # Beyond the boundaries
            if theta < -math.pi/2 + 0.2:
                theta += math.pi
                v *= -1
            elif theta > math.pi/2 - 0.2:
                theta -= math.pi
                v *= -1
            else:
                self.hemisphere = "front"

        print(f"Hemisphere: {self.hemisphere}")

        # Apply the computed velocities
        self.set_velocity(v, theta)

        # Publish the updated velocities to each wheel
        self.publish_velocity()

    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.front_left_velocity_publisher.publish(self.front_left_velocity)
        self.front_right_velocity_publisher.publish(self.front_right_velocity)
        self.back_left_velocity_publisher.publish(self.back_left_velocity)
        self.back_right_velocity_publisher.publish(self.back_right_velocity)

        self.steer_angle_publisher.publish(self.angle)

    def set_velocity(self, vel, angle):
        """Set velocity for each individual wheel."""

        self.front_left_velocity.linear.x = vel
        self.front_right_velocity.linear.x = vel
        self.back_left_velocity.linear.x = vel
        self.back_right_velocity.linear.x = vel

        self.angle = angle
            
    def on_press(self, key):
        """Handle key press events."""
        self.manual_control_active = True

        try:
            still = 0
            forward = 5
            backward = -5
            left_turn = math.pi/2
            right_turn = -math.pi/2

            # Add the pressed key to the active keys set
            self.active_keys.add(key.char)

            # Compute velocity and angular velocity based on active keys
            velocity = still
            angle = still

            if 'w' in self.active_keys:  # Forward
                velocity = forward
            if 's' in self.active_keys:  # Backward
                velocity = backward
            if 'a' in self.active_keys:  # Left turn
                angle = left_turn
            if 'd' in self.active_keys:  # Right turn
                angle = right_turn

            self.set_velocity(velocity, angle)
            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
        self.active_keys.discard(key.char)
        self.set_velocity(0.0,0.0)
        self.publish_velocity()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = Control()
        controller.run()
    except rospy.ROSInterruptException:
        pass
