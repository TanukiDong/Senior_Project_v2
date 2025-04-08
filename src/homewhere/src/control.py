#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from pynput import keyboard

class Control:
    def __init__(self):
        rospy.init_node('control')

        # Publishers for wheel velocity commands
        self.front_left_velocity_publisher = rospy.Publisher('/cmd_vel_front_left', Twist, queue_size=10)
        self.front_right_velocity_publisher = rospy.Publisher('/cmd_vel_front_right', Twist, queue_size=10)
        self.back_left_velocity_publisher = rospy.Publisher('/cmd_vel_back_left', Twist, queue_size=10)
        self.back_right_velocity_publisher = rospy.Publisher('/cmd_vel_back_right', Twist, queue_size=10)
        self.centroid_velocity_publisher = rospy.Publisher('/cmd_vel_centroid', Twist, queue_size=10)

        # Publishers for steering angle commands
        self.front_left_steer_publisher = rospy.Publisher('/cmd_steer_front_left', Float64, queue_size=10)
        self.front_right_steer_publisher = rospy.Publisher('/cmd_steer_front_right', Float64, queue_size=10)
        self.back_left_steer_publisher = rospy.Publisher('/cmd_steer_back_left', Float64, queue_size=10)
        self.back_right_steer_publisher = rospy.Publisher('/cmd_steer_back_right', Float64, queue_size=10)

        # Subscribe to the global /cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.active_keys = set()
        self.manual_control_active = False

        # Initialize velocity messages for each wheel
        self.front_left_velocity = Twist()
        self.front_right_velocity = Twist()
        self.back_left_velocity = Twist()
        self.back_right_velocity = Twist()
        self.centroid_velovity = Twist()

        # Initialize steering angle messages for each wheel
        self.front_left_steering_angle = Float64()
        self.front_right_steering_angle = Float64()
        self.back_left_steering_angle = Float64()
        self.back_right_steering_angle = Float64()

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def calculate(v, w, S=0.5):

        # In case parallel parking
        if v == 0:
            if w == 0:
                return [0]*6,[0]*4
            elif w > 0:
                return [0,0,0,0,0,w], [0.111,0.907,-0.111,-0.907]
            else:
                return [0,0,0,0,0,w], [-0.111,-0.907,0.111,0.907]

        # Calculate turn radius
        r = math.inf if w == 0 else v / w

        # Calculate left and right angles for steering joints
        theta_l = math.atan2(S / 2, (r - S / 2)) if r > 0 else -math.atan2(S / 2, (-r + S / 2))
        theta_r = math.atan2(S / 2, (r + S / 2)) if r > 0 else -math.atan2(S / 2, (-r - S / 2))

        # Calculate turn radii and wheel velocities
        r_l = math.sqrt((S / 2) ** 2 + (r - S / 2) ** 2)
        r_r = math.sqrt((S / 2) ** 2 + (r + S / 2) ** 2)
        v_l = abs(w) * r_l if w != 0 else v
        v_r = abs(w) * r_r if w != 0 else v

        # Calculate hardware velocities for each wheel
        v_l_hardware = v_l if abs(theta_l) <= math.pi / 2 else -v_l
        v_r_hardware = v_r if abs(theta_r) <= math.pi / 2 else -v_r
        if w != 0:
            v_l_hardware = v_l_hardware if v > 0 else -v_l_hardware
            v_r_hardware = v_r_hardware if v > 0 else -v_r_hardware

        def constrain_theta(theta):
            if theta < -math.pi / 2:
                return -math.pi / 2
            elif theta > math.pi / 2:
                return math.pi / 2
            else:
                return theta

        # Constrain the steering angles for each wheel
        theta_fl = constrain_theta(theta_l)
        theta_fr = constrain_theta(theta_r)
        theta_bl = -theta_fl
        theta_br = -theta_fr

        # Create velocity array for wheel speeds (linear velocities)
        velocity = [v_l_hardware, v_r_hardware, v_l_hardware, v_r_hardware, v, w]

        # Create angular velocities array (steering angles)
        steering_angles = [theta_fl, theta_fr, theta_bl, theta_br]

        return velocity, steering_angles

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel messages from move_base."""

        # Extract linear and angular velocities from the cmd_vel message
        v = msg.linear.x
        w = msg.angular.z

        velocity, steering_angles = Control.calculate(v,w)

        # Publish the velocities for each wheel
        self.set_velocity([velocity, steering_angles])
        self.publish_velocity()

    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.front_left_velocity_publisher.publish(self.front_left_velocity)
        self.front_right_velocity_publisher.publish(self.front_right_velocity)
        self.back_left_velocity_publisher.publish(self.back_left_velocity)
        self.back_right_velocity_publisher.publish(self.back_right_velocity)
        self.centroid_velocity_publisher.publish(self.centroid_velovity)

        # Publish the steering angles for each wheel
        self.front_left_steer_publisher.publish(self.front_left_steering_angle)
        self.front_right_steer_publisher.publish(self.front_right_steering_angle)
        self.back_left_steer_publisher.publish(self.back_left_steering_angle)
        self.back_right_steer_publisher.publish(self.back_right_steering_angle)

    def set_velocity(self, vel):
        """Set velocity for each individual wheel and the steering angles."""
        self.front_left_velocity.linear.x = vel[0][0]
        self.front_right_velocity.linear.x = vel[0][1]
        self.back_left_velocity.linear.x = vel[0][2]
        self.back_right_velocity.linear.x = vel[0][3]
        self.centroid_velovity.linear.x = vel[0][4]
        self.centroid_velovity.angular.z = vel[0][5]

        # Set steering angles for each wheel
        self.front_left_steering_angle.data = vel[1][0]
        self.front_right_steering_angle.data = vel[1][1]
        self.back_left_steering_angle.data = vel[1][2]
        self.back_right_steering_angle.data = vel[1][3]

    def on_press(self, key):
        """Handle key press events."""
        self.manual_control_active = True

        try:
            still = 0
            forward = 5
            backward = -5
            left_turn = 10
            right_turn = -10

            # Add the pressed key to the active keys set
            self.active_keys.add(key.char)

            # Compute velocity and angular velocity based on active keys
            velocity = still
            angular_velocity = still

            if 'w' in self.active_keys:  # Forward
                velocity = forward
            elif 's' in self.active_keys:  # Backward
                velocity = backward
            if 'a' in self.active_keys:  # Left turn
                angular_velocity = left_turn
            elif 'd' in self.active_keys:  # Right turn
                angular_velocity = right_turn

            x = Control.calculate(velocity,angular_velocity)
            print(x)

            self.set_velocity(x)
            self.publish_velocity()

        except AttributeError:
            pass

    def on_release(self, key):
        """Handle key release events."""
        self.manual_control_active = False
        self.active_keys.discard(key.char)
        self.set_velocity([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        self.publish_velocity()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = Control()
        controller.run()
    except rospy.ROSInterruptException:
        pass