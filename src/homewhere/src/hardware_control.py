#!/usr/bin/env python

import rospy
import os
from hardware import arduino_control, motors_control
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int16
from kalman_filter import KalmanFilter
import numpy as np

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
        self.theta = 0.0
        self.vel_limit = 0.4

        # ROS Publisher
        self.encoder_publisher = rospy.Publisher('/cmd_hardware_reading', Float32MultiArray, queue_size=10)
        self.imu_publisher = rospy.Publisher('/cmd_mpu_reading', Float32MultiArray, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber("/cmd_vel_front_left", Twist, self.front_left_callback)
        rospy.Subscriber("/cmd_vel_front_right", Twist, self.front_right_callback)
        rospy.Subscriber("/cmd_vel_back_left", Twist, self.back_left_callback)
        rospy.Subscriber("/cmd_vel_back_right", Twist, self.back_right_callback)
        rospy.Subscriber("/cmd_angle", Int16, self.angle_callback)

        # Detect and connect hardware devices
        self.setup_hardware()

        self.kf = KalmanFilter(
            R=np.diag(
                [5e-4]*4+
                [2e-2]*4
            ),
            Q=np.diag([
                0.1, 0.1
            ]),
            dt=Hardware_Controller.REFRESH_RATE
        )

        self.current_theoretical_velocity = 0.0

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
        self.velFrontLeft_Linear_y = msg.linear.y
        self.current_theoretical_velocity = msg.linear.x*60/2/np.pi/0.0695 # for KF

    def front_right_callback(self, msg):
        self.velFrontRight_Linear_x = msg.linear.x
        self.velFrontRight_Linear_y = msg.linear.y

    def back_left_callback(self, msg):
        self.velBackLeft_Linear_x = msg.linear.x
        self.velBackLeft_Linear_y = msg.linear.y

    def back_right_callback(self, msg):
        self.velBackRight_Linear_x = msg.linear.x
        self.velBackRight_Linear_y = msg.linear.y

    def angle_callback(self, msg):
        self.theta = msg.data  # Float32 contains data in `.data`

    # ---- Sensor Reading ----
    def read_sensor(self):
        try:
            dl_list = self.motors.get_delta_travelled()  # [dl1, dl2, dl3, dl4]
            rpm_list = self.motors.get_rpms()            # [rpm1, rpm2, rpm3, rpm4]

            # Stack all into one big measurement vector
            z = np.array(dl_list + rpm_list).reshape(8, 1)
            u = np.array([[self.current_theoretical_velocity]])

            self.kf.predict(u)
            filtered_dl, filtered_rpm = self.kf.update(z)

            rospy.loginfo(f"dl: {dl_list}, v: {self.current_theoretical_velocity}, rpm: {rpm_list}")
            rospy.loginfo(f"Filtered: dl={filtered_dl:.4f}, rpm={filtered_rpm:.2f}")
            return [filtered_dl, filtered_rpm]

        except Exception as e:
            rospy.logerr(f"Encoder reading error: {e}")
            return [0.0, 0.0]
        
    def read_mpu(self):
        """Get IMU Data"""
        try:
            angles = self.arduino.read_mpu6050().get_angle()
            print("tilts: ", angles)
            return angles
        except Exception as e:
            rospy.logerr(f"MPU reading error: {e}")
            return [[0.0], [0.0]]  # Fail-safe default

    # ---- Actuator Commands ----
    def cmd_actuators(self):
        """Send Velocity and Steering Commands"""
        try:
            vel = self.velFrontLeft_Linear_x
            if abs(vel) > self.vel_limit:
                vel = self.vel_limit*(1 if vel > 0 else -1)

            vel_list = [vel]*4
            print("Vel:", vel)
            
            self.motors.set_vel(vel_list)
            # Set steering angle
            self.arduino.control_servos(int(self.theta))

            print(vel_list, self.theta)
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
                rospy.logerr("Sensor data length mismatch. Expected 2 values.")
                return

            # Publish sensor readings
            msg = Float32MultiArray()
            msg.data = reading_list
            print("Reading list: ", reading_list)
            self.encoder_publisher.publish(msg)

            # Publish IMU
            tilts = Float32MultiArray()
            tilts.data = self.read_mpu()
            self.imu_publisher.publish(tilts)

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