#!/usr/bin/env python

import rospy
import os
from hardware import arduino_control, motors_control
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

class Hardware_Controller:

    # Config Here
    ARDUINO_ADDR = "ttyUSB0"
    FRONT_ADDR = "ttyUSB1"
    REAR_ADDR = "ttyUSB2"
    REFRESH_RATE = 0.1 # in second

    def __init__(self):
        """Initialize ROS Node and Hardware Connections"""
        rospy.init_node("hardware_controller")

        # Define class variables for velocity and angle
        self.velFrontLeft_Linear = 0.0
        self.velFrontRight_Linear = 0.0
        self.velBackLeft_Linear = 0.0
        self.velBackRight_Linear = 0.0
        self.steerFrontLeft_Linear = 0.0
        self.steerFrontRight_Linear = 0.0
        self.steerBackLeft_Linear = 0.0
        self.steerBackRight_Linear = 0.0

        # ROS Publisher
        self.reading_publisher = rospy.Publisher('/cmd_hardware_reading', Float64MultiArray, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber("/cmd_vel_front_left", Twist, self.front_left_callback)
        rospy.Subscriber("/cmd_vel_front_right", Twist, self.front_right_callback)
        rospy.Subscriber("/cmd_vel_back_left", Twist, self.back_left_callback)
        rospy.Subscriber("/cmd_vel_back_right", Twist, self.back_right_callback)
        rospy.Subscriber("/cmd_steer_front_left", Float64, self.steer_front_left_callback)
        rospy.Subscriber("/cmd_steer_front_right", Float64, self.steer_front_right_callback)
        rospy.Subscriber("/cmd_steer_back_left", Float64, self.steer_back_left_callback)
        rospy.Subscriber("/cmd_steer_back_right", Float64, self.steer_back_right_callback)

        # Detect and connect hardware devices
        self.setup_hardware()

        # Start periodic update loop
        rospy.Timer(rospy.Duration(Hardware_Controller.REFRESH_RATE), self.update)  # Runs update every 0.1 sec

    def setup_hardware(self):
        """Initialize Arduino and Motor Drivers"""
        ports = os.listdir("/dev/")
        usb = [port for port in ports if port[:6] == "ttyUSB"]

        # Arduino Connection
        if Hardware_Controller.ARDUINO_ADDR in usb:
            self.arduino = arduino_control.Arduino("/dev/" + Hardware_Controller.ARDUINO_ADDR)
            rospy.loginfo(f"Arduino connected via serial @{Hardware_Controller.ARDUINO_ADDR}")
        else:
            rospy.logerr(f"No Arduino at port: {Hardware_Controller.ARDUINO_ADDR}")
            raise Exception(f"No Arduino at port: {Hardware_Controller.ARDUINO_ADDR}")

        # Motor Connection
        if Hardware_Controller.FRONT_ADDR in usb and Hardware_Controller.REAR_ADDR in usb:
            self.motors = motors_control.Motors(
                front_port="/dev/" + Hardware_Controller.FRONT_ADDR,
                rear_port="/dev/" + Hardware_Controller.REAR_ADDR
            )
            rospy.loginfo(f"Motors connected via serial @({Hardware_Controller.FRONT_ADDR}, {Hardware_Controller.REAR_ADDR})")
        else:
            rospy.logerr(f"No motor at ports {Hardware_Controller.FRONT_ADDR} or {Hardware_Controller.REAR_ADDR}")
            raise Exception(f"No motor at ports {Hardware_Controller.FRONT_ADDR} or {Hardware_Controller.REAR_ADDR}")

    # ---- ROS Subscriber Callbacks ----
    def front_left_callback(self, msg):
        self.velFrontLeft_Linear = msg.linear.x

    def front_right_callback(self, msg):
        self.velFrontRight_Linear = msg.linear.x

    def back_left_callback(self, msg):
        self.velBackLeft_Linear = msg.linear.x

    def back_right_callback(self, msg):
        self.velBackRight_Linear = msg.linear.x

    def steer_front_left_callback(self, msg):
        self.steerFrontLeft_Linear = msg.data  # Float32 contains data in `.data`

    def steer_front_right_callback(self, msg):
        self.steerFrontRight_Linear = msg.data  # Float32 contains data in `.data`

    def steer_back_left_callback(self, msg):
        self.steerBackLeft_Linear = msg.data  # Float32 contains data in `.data`

    def steer_back_right_callback(self, msg):
        self.steerBackRight_Linear = msg.data  # Float32 contains data in `.data`

    # ---- Sensor Reading ----
    def read_sensor(self):
        """Get IMU and Encoder Data"""
        try:
            encoder = self.motors.get_tick()
            speed = self.motors.get_speeds()
            imu = self.arduino.get_tilt()
            return [*encoder, *speed, *imu]  # Return as a list
        except Exception as e:
            rospy.logerr(f"Sensor reading error: {e}")
            return [0.0, 0.0]  # Fail-safe default

    # ---- Actuator Commands ----
    def cmd_actuators(self):
        """Send Velocity and Steering Commands"""
        try:
            # Set wheel velocities
            vel_list = [
                self.velFrontLeft_Linear,
                self.velFrontRight_Linear,
                self.velBackLeft_Linear,
                self.velBackRight_Linear,
            ]
            self.motors.set_vel(vel_list)

            # Set steering angle
            self.arduino.control_servo_6(self.steerFrontLeft_Linear)
            self.arduino.control_servo_7(self.steerFrontRight_Linear)
            self.arduino.control_servo_8(self.steerBackLeft_Linear)
            self.arduino.control_servo_9(self.steerBackRight_Linear)

        except Exception as e:
            rospy.logerr(f"Actuator command error: {e}")

    # ---- Periodic Update ----
    def update(self, event):
        """Runs periodically to get sensor readings and command actuators"""
        try:
            # Read sensor data (returns 4 encoder values + 2 IMU values)
            reading_list = self.read_sensor()

            # Ensure the list has exactly 6 elements
            if len(reading_list) != 6:
                rospy.logerr("Sensor data length mismatch. Expected 6 values.")
                return

            # Publish sensor readings
            msg = Float64MultiArray()
            msg.data = reading_list  # [enc1, enc2, enc3, enc4, imu1, imu2]
            self.reading_publisher.publish(msg)

            # Command the actuators
            self.cmd_actuators()

        except Exception as e:
            rospy.logerr(f"Update error: {e}")


    # ---- Fail-Safe Shutdown ----
    def fail_safe(self):
        """Shutdown motors and reset servo if an error occurs"""
        rospy.logwarn("Emergency Stop Activated!")
        try:
            self.motors.terminate()
            self.arduino.control_servo_8(0)
        except Exception as e:
            rospy.logerr(f"Fail-safe error: {e}")

# ---- Run the Node ----
if __name__ == "__main__":
    try:
        controller = Hardware_Controller()
        rospy.spin()  # Keeps the node running
    except Exception as e:
        rospy.logerr(f"Fatal Error: {e}")
