#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import tf
from math import pi

def model_states_callback(msg):
    robot_name = 'box_robot'  # Replace with your robot's model name
    
    if robot_name in msg.name:
        index = msg.name.index(robot_name)
        
        # Get position (x, y, z)
        position = msg.pose[index].position
        x = position.x
        y = position.y
        z = position.z

        # Get orientation (quaternion)
        orientation = msg.pose[index].orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        # Extract yaw (theta)
        theta = euler[2]
        
        # Optionally convert theta to degrees
        theta_deg = theta * (180.0 / pi)

        # Print out the position and orientation (theta)
        rospy.loginfo(f"Robot Position: x={x}, y={y}, theta={theta_deg} degrees")

def main():
    rospy.init_node('gazebo_model_position')
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
