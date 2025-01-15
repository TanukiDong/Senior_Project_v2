#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class SkidSteeringControl:
    def __init__(self):
        rospy.init_node('skid_steering_control')

        # Publishers for left and right wheel velocity commands
        self.vel_left_front_pub = rospy.Publisher('/cmd_vel_left_front', Twist, queue_size=10)
        self.vel_left_middle_pub = rospy.Publisher('/cmd_vel_left_middle', Twist, queue_size=10)
        self.vel_left_back_pub = rospy.Publisher('/cmd_vel_left_back', Twist, queue_size=10)
        self.vel_right_front_pub = rospy.Publisher('/cmd_vel_right_front', Twist, queue_size=10)
        self.vel_right_middle_pub = rospy.Publisher('/cmd_vel_right_middle', Twist, queue_size=10)
        self.vel_right_back_pub = rospy.Publisher('/cmd_vel_right_back', Twist, queue_size=10)

        # self.avg_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Subscriber for /cmd_vel (from move_base or other sources)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        # Manual control flag
        self.manual_control_active = False

        # Initialize velocity messages for each wheel
        self.left_front_velocity = Twist()
        self.left_middle_velocity = Twist()
        self.left_back_velocity = Twist()
        self.right_front_velocity = Twist()
        self.right_middle_velocity = Twist()
        self.right_back_velocity = Twist()

        # self.avg_velocity = Twist()

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel messages from move_base."""
        if not self.manual_control_active:  # Only process /cmd_vel when manual control is inactive
            self.map_cmd_vel_to_wheels(msg.linear.x, msg.angular.z)
            self.publish_velocity()

    def map_cmd_vel_to_wheels(self, linear_x, angular_z):
        """Map linear and angular velocities to wheel velocities."""
        THRESHOLD1 = 0.1
        THRESHOLD2 = 0.2
        forward = [50, 50, 50, 50, 50, 50]
        backward = [-50, -50, -50, -50, -50, -50]
        left_turn = [-50, -50, -50, 50, 50, 50]
        right_turn = [50, 50, 50, -50, -50, -50]

        scalar = 0.1
        scalar2 = 1
        forward_scaled = [v * scalar for v in forward]
        backward_scaled = [v * scalar for v in backward]
        left_turn_scaled = [v * scalar2 for v in left_turn]
        right_turn_scaled = [v * scalar2 for v in right_turn]
        
        if abs(angular_z) > THRESHOLD2: 
            if angular_z > 0:
                self.set_velocity(left_turn_scaled)
            if angular_z < 0:
                self.set_velocity(right_turn_scaled)
            return
        elif abs(linear_x) > THRESHOLD1:
            if linear_x > 0:
                self.set_velocity(forward_scaled)
            if linear_x < 0:
                self.set_velocity(backward_scaled)
            return
        else:
            self.set_velocity([0,0,0,0,0,0])


    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.vel_left_front_pub.publish(self.left_front_velocity)
        self.vel_left_middle_pub.publish(self.left_middle_velocity)
        self.vel_left_back_pub.publish(self.left_back_velocity)
        self.vel_right_front_pub.publish(self.right_front_velocity)
        self.vel_right_middle_pub.publish(self.right_middle_velocity)
        self.vel_right_back_pub.publish(self.right_back_velocity)

        # self.avg_vel_pub.publish(self.avg_velocity)

    def set_velocity(self, vel):
        """Set velocity for each individual wheel."""
        self.left_front_velocity.angular.z = vel[0]
        self.left_middle_velocity.angular.z = vel[1]
        self.left_back_velocity.angular.z = vel[2]
        self.right_front_velocity.angular.z = vel[3]
        self.right_middle_velocity.angular.z = vel[4]
        self.right_back_velocity.angular.z = vel[5]
    
    # def scalar_multiply(self, scalar, list_values):
    #     return [x * scalar for x in list_values]

    def avg(self, value):
        return sum(value)/len(value)

    def on_press(self, key):
        """Handle key press events."""
        self.manual_control_active = True

        try:
            forward = [50, 50, 50, 50, 50, 50]
            backward = [-50, -50, -50, -50, -50, -50]
            left_turn = [-50, -50, -50, 50, 50, 50]
            right_turn = [50, 50, 50, -50, -50, -50]
            # left_turn = [-40, -35, -30, 30, 35, 40]
            # right_turn = [30, 35, 40, -40, -35, -30]

            scalar = 0.5
            forward_scaled = [v * scalar for v in forward]
            backward_scaled = [v * scalar for v in backward]
            left_turn_scaled = [v * scalar for v in left_turn]
            right_turn_scaled = [v * scalar for v in right_turn]

            if key.char == 'w':  # Forward
                self.set_velocity(forward_scaled)
            elif key.char == 's':  # Backward
                self.set_velocity(backward_scaled)
            elif key.char == 'a':  # Left turn
                self.set_velocity(left_turn_scaled)
            elif key.char == 'd':  # Right turn
                self.set_velocity(right_turn_scaled)
            elif key.char == 'q':  # Quit
                rospy.signal_shutdown("User requested shutdown.")

            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
        self.set_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.publish_velocity()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = SkidSteeringControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
