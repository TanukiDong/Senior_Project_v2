#!/usr/bin/env python

import rospy
import os
from hardware import arduino_control, motors_control
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int16

class Hardware_Controller:
    """A Class to Control Hardwares"""

    # Config Here
    ARDUINO_ADDR = "ttyUSB3"
    FRONT_ADDR = "ttyUSB1"
    REAR_ADDR = "ttyUSB2"
    REFRESH_RATE = 0.1 # in second

    def __init__(self):
        """Initialize ROS Node and Hardware Connections"""
        rospy.init_node("hardware_controller")

        # Define class variables for velocity and angle
        self.velFrontLeft_Linear_x = 0.0
        self.velFrontLeft_Linear_y = 0.0
        self.velFrontRight_Linear_x = 0.0
        self.velFrontLeft_Linear_y = 0.0
        self.velBackLeft_Linear_x = 0.0
        self.velBackLeft_Linear_y = 0.0
        self.velBackRight_Linear_x = 0.0
        self.velBackRight_Linear_y = 0.0
        self.theta_fl = 0.0
        self.theta_fr = 0.0
        self.theta_bl = 0.0
        self.theta_br = 0.0
        self.vel_limit = 0.1

        # ROS Publisher
        self.reading_publisher = rospy.Publisher('/cmd_hardware_reading', Float32MultiArray, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber("/cmd_vel_front_left", Twist, self.front_left_callback)
        rospy.Subscriber("/cmd_vel_front_right", Twist, self.front_right_callback)
        rospy.Subscriber("/cmd_vel_back_left", Twist, self.back_left_callback)
        rospy.Subscriber("/cmd_vel_back_right", Twist, self.back_right_callback)
        rospy.Subscriber("/cmd_angle_front_left", Int16, self.angle_fl_callback)
        rospy.Subscriber("/cmd_angle_front_right", Int16, self.angle_fr_callback)
        rospy.Subscriber("/cmd_angle_back_left", Int16, self.angle_bl_callback)
        rospy.Subscriber("/cmd_angle_back_right", Int16, self.angle_br_callback)

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
        self.velFrontLeft_Linear_x = msg.linear.x
        # self.velFrontLeft_Linear_y = msg.linear.y

    def front_right_callback(self, msg):
        self.velFrontRight_Linear_x = msg.linear.x
        # self.velFrontRight_Linear_y = msg.linear.y

    def back_left_callback(self, msg):
        self.velBackLeft_Linear_x = msg.linear.x
        # self.velBackLeft_Linear_y = msg.linear.y

    def back_right_callback(self, msg):
        self.velBackRight_Linear_x = msg.linear.x
        # self.velBackRight_Linear_y = msg.linear.y

    def angle_fl_callback(self, msg):
        self.theta_fl = msg.data  # Float32 contains data in `.data`

    def angle_fr_callback(self, msg):
        self.theta_fr = msg.data  # Float32 contains data in `.data`

    def angle_bl_callback(self, msg):
        self.theta_bl = msg.data  # Float32 contains data in `.data`

    def angle_br_callback(self, msg):
        self.theta_br = msg.data  # Float32 contains data in `.data`

    # ---- Sensor Reading ----
    def read_sensor(self):
        """Get IMU and Encoder Data"""
        try:
            dl_list = self.motors.get_delta_travelled()
            rpm_list = self.motors.get_rpms()

            print("DL, RPM:", dl_list, rpm_list)
            # imu = self.arduino.get_tilt()
            # return [*encoder, *imu]  # Return as a list
            return [*dl_list, *rpm_list]
        except Exception as e:
            rospy.logerr(f"Sensor reading error: {e}")
            return [[0.0]*8]  # Fail-safe default

    # ---- Actuator Commands ----
    def cmd_actuators(self):
        """Send Velocity and Steering Commands"""
        try:
            vel_list = [self.velFrontLeft_Linear_x,
                   self.velFrontRight_Linear_x,
                   self.velBackLeft_Linear_x,
                   self.velBackRight_Linear_x]
            vel_list = [vel_i if vel_i < self.vel_limit else self.vel_limit for vel_i in vel_list] 

            theta_list = [self.theta_fl,
                          self.theta_fr,
                          self.theta_bl,
                          self.theta_br]   
            theta_list = [int(theta_i) for theta_i in theta_list]
            
            # set wheel velocities
            self.motors.set_vel(vel_list)
            # Set steering angles
            self.arduino.control_servos(theta_list)

        except Exception as e:
            rospy.logerr(f"Actuator command error: {e}")

    # ---- Periodic Update ----
    def update(self, event):
        """Runs periodically to get sensor readings and command actuators"""
        try:
            # Read sensor data (returns 4 encoder values + 2 IMU values)
            reading_list = self.read_sensor()

            # Ensure the list has exactly 6 elements
            if len(reading_list) != 2:
                rospy.logerr("Sensor data length mismatch. Expected 6 values.")
                return

            # Publish sensor readings
            msg = Float32MultiArray()
            msg.data = reading_list
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