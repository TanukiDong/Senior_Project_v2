#!/usr/bin/env python

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

        # Manual control flag
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
            still = [0,0,0,0]
            forward = [-10, -10, 10, -10]
            backward = [10, 10, 10, 10]
            left_turn = [0.785, 0.785, 0.785, 0.785]
            right_turn = [-0.785, -0.785, -0.785, -0.785]

            if key.char == 'w':  # Forward
                self.set_velocity([forward, still])
            elif key.char == 's':  # Backward
                self.set_velocity([backward, still])
            elif key.char == 'a':  # Left turn
                self.set_velocity([still,left_turn])
            elif key.char == 'd':  # Right turn
                self.set_velocity([still, right_turn])
            elif key.char == 'q':  # Quit
                rospy.signal_shutdown("User requested shutdown.")

            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
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
