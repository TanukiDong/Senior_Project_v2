import serial
import time
import math
import platform

class imu_reading:
    def __init__(self, ax, ay, az, wx, wy, wz, z=0):
        self.ax = ax
        self.ay = ay
        self.az = az
        self.wx = wx
        self.wy = wy
        self.wz = wz
        

    def get_angle(self):
        # Accelerometer-based pitch and roll (in radians)
        pitch_acc = math.atan2(self.ay, math.sqrt(self.ax**2 + self.az**2))
        roll_acc = math.atan2(-self.ax, math.sqrt(self.ay**2 + self.az**2))

        # Return the angles in degrees
        return (math.degrees(pitch_acc),math.degrees(roll_acc))  

    def __str__(self):
        return f"Accelerations (m/s**2): {self.ax}, {self.ay}, {self.az}\tVelocity (rad/s): {self.wx}, {self.wy}, {self.wz}"  

class Arduino:

    def __init__(self, com='/dev/ttyUSB3', baud=9600, timeout=1):
        """Initialize serial connection"""
        ser = serial.Serial(com, baud, timeout=timeout)  # Update COM port as needed
        time.sleep(2)  # Allow time for Arduino to initialize
        self.ser = ser

        # steer characteristic of each servos 6-9
        y = [(26,94,160),
             (22,92,163),
             (22,89,157),
             (25,94,162)] # 0deg, 90deg, 180deg
        self.coefs = [self.calculate_piecewise_linear_params(y_points=tup) for tup in y]

    def calculate_piecewise_linear_params(self,y_points, x_points=[0,90,180]):
        if len(x_points) != 3 or len(y_points) != 3:
            raise ValueError("Exactly three points are required.")
        
        # First segment (low x to mid x)
        slope1 = (y_points[1] - y_points[0]) / (x_points[1] - x_points[0])
        intercept1 = y_points[0] - slope1 * x_points[0]
        
        # Second segment (mid x to high x)
        slope2 = (y_points[2] - y_points[1]) / (x_points[2] - x_points[1])
        intercept2 = y_points[1] - slope2 * x_points[1]
        
        return (slope1, intercept1), (slope2, intercept2)

    def write(self,string):
        """Encode and Write the string to the Arduino"""
        self.ser.write(string.encode())

    def read(self):
        """Decode and Read the string from the Arduino"""
        return self.ser.readline().decode().strip()

    def read_sensor_value(self): # button on Pin6
        """Retrieve the value of the sensor on pin 6."""
        self.write('READ_SENSOR\n')
        time.sleep(0.1)
        data = self.read()
        return int(data) if data.isdigit() else None
    
    def control_servo_6(self,angle): # Servo on Pin6
        """Command servo on pin 6 to move to a specific angle."""
        command = f'SERVO6:{angle}\n'
        self.write(command)
        time.sleep(0.1)
        response = self.read()
        return response

    def control_servo_7(self,angle): # Servo on Pin7
        """Command servo on pin 7 to move to a specific angle."""
        command = f'SERVO7:{angle}\n'
        self.write(command)
        time.sleep(0.1)
        response = self.read()
        return response

    def control_servo_8(self,angle): #Servo on Pin8
        """Command servo on pin 8 to move to a specific angle."""
        command = f'SERVO8:{angle}\n'
        self.write(command)
        time.sleep(0.1)
        response = self.read()
        return response
    
    def control_servo_9(self, angle): #Servo on Pin9
        """Command servo on pin 9 to move to a specific angle."""
        command = f'SERVO9:{angle}\n'
        self.write(command)
        time.sleep(0.01)
        response = self.read()
        return response
    
    def control_servos(self, angle, verbose=True):
        """Command all servos to move to a specific angle."""
        # Note:
        # Servo 6: Top Right (Red)
        # Servo 7: Bottom Right
        # Servo 8: Bottom Left
        # Servo 9: Top Left (yellow)
        
        angle = 90 - angle # change units to match with the Arduino's
        new_angles = [coef1[0]*angle + coef1[1] 
                      if angle < 90 
                      else coef2[0]*angle + coef2[1] 
                      for coef1, coef2 in self.coefs] # transform angles
        for i in range(6, 10):
            # Calculate new local angle
            local_angle = new_angles[i-6]
            if local_angle < 0:
                local_angle = 0
            elif local_angle > 180:
                local_angle = 180
            local_angle = int(local_angle)
            # command the servo to move to the new angle
            command = f'SERVO{i}:{local_angle}\n'
            self.write(command)
            time.sleep(0.001) # TODO: change this
            response = self.read()
            if verbose:
                print(response)

    def control_servos_new(self, angle, verbose=False):
        angle = 90 - angle
        new_angles = [coef1[0]*angle + coef1[1] 
                      if angle < 90 
                      else coef2[0]*angle + coef2[1] 
                      for coef1, coef2 in self.coefs]
        new_angles = [angle_i if angle_i > 0 else 0 for angle_i in new_angles]
        new_angles = [angle_i if angle_i < 180 else 180 for angle_i in new_angles]
        command = f'SERVOS:{new_angles[0]},{new_angles[1]},{new_angles[2]},{new_angles[3]}\n'
        self.write(command)
        time.sleep(0.01)
        if verbose:
            response = self.read()
            print(response)

    def read_ultrasonic_distance(self):
        """Read the distance from the ultrasonic sensor in centimeters."""
        self.write('READ_ULTRASONIC\n')
        time.sleep(0.1)
        data = self.read()
        return int(data) if data.isdigit() else None

    def read_mpu6050(self):
        """Retrieve accelerometer and gyroscope data from the MPU6050."""
        self.write('READ_MPU6050\n')
        time.sleep(0.1)
        data = self.read()
        if data:
            try:
                ax, ay, az, gx, gy, gz = map(float, data.split(","))
                return imu_reading(ax,ay,az,gx,gy,gz)
            except ValueError:
                return None
        return None

    def tilted(self, threshold=10):
        """Return whether the robot is tilted over a certain threshold"""
        read = self.read_mpu6050()
        if read is not None:
            tiltx, tilty = read.get_angle()
            return abs(tiltx) >= threshold or abs(tilty) >= threshold
        return

def main():

    system = platform.system()
    if system == "Windows":
        com = "COM9"
    elif system == "Linux":
        com = "/dev/ttyUSB3"

    arduino = Arduino(com=com)

    dt = 0.5

    while True:
        # angle = input("Enter the angle: ")
        # if not angle.isnumeric():
        #     return
        # print(arduino.control_servos(int(angle)))
        # time.sleep(dt) 
        print(arduino.tilted())

if __name__ == '__main__':
    main()
