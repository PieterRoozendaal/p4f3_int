#!/usr/bin/env python

"""
Auteur: Jurre Fikkers
Studentnumber: 2196902
Description: Main control script for robot operation
"""

# Import necessary libraries and modules
import rospy  # ROS Python library
import json  # JSON library for handling JSON data
import sys  # System-specific parameters and functions
import tf  # ROS transform library
import math  # Mathematical functions
import argparse  # Argument parser for command-line options
from geometry_msgs.msg import Pose  # ROS message for robot poses
from std_msgs.msg import String  # ROS message for standard string messages
from niryo_robot_python_ros_wrapper import NiryoRosWrapper  # Wrapper for Niryo robot
from robot_controller import MoveitRobotController  # Custom robot controller

# Global Variables
number_of_bins = 4  # Number of sorting bins
error_message = ""  # Error variable
object_list = ["", "obj1", "obj2", "obj3", "obj4"]  # List of objects to be sorted
start_robot = False  # Robot starting signal
emergency_stop = False #Robot emergency signal
sorting_bin = 1  # Variable to store the current sorting bin

# Define pickup offsets for each object
pickup_offsets = {
    "obj1": [0.0, 0.0, 0.0],  
    "obj2": [0.0, 0.0, 0.0],
    "obj3": [0.0, 0.0, 0.0],  
    "obj4": [0.0, 0.0, 0.0]  
}

class Main:
    def __init__(self, test_mode=False):
        rospy.on_shutdown(self.ros_shutdown)  # Register ros_shutdown to be called on shutdown
        self.robot_controller = MoveitRobotController("positions.json")  # Initialize robot controller with positions file
        self.gripper_controller = NiryoRosWrapper()  # Initialize gripper controller
        # Hieronder noodstop bij invoegen
        self.read_program_start = rospy.Subscriber('/robot_start', String, self.robot_start_callback)  # Subscriber for start signal
        self.read_inspection_results = rospy.Subscriber('/robot_sort_pose', String, self.inspection_result_callback)  # Subscriber for inspection results
        self.write_robot_state = rospy.Publisher('/robot_status', String, queue_size=1)  # Publisher for robot status
        rospy.sleep(0.5)  # Sleep to allow publisher to initialize
        self.robot_state = String(data="STANDBY")  # Initialize robot state to STANDBY
        self.write_robot_state.publish(self.robot_state)  # Publish initial robot state
        rospy.sleep(0.5)  # Sleep to allow state to publish
        self.test_mode = test_mode  # Set test mode

        if test_mode:
            self.run_test_mode()  # Run test mode if test_mode is True
        else:
            self.main()  # Otherwise, run main loop

    def run_test_mode(self):
        self.robot_controller.go_to_named_position('home')  # Move robot to home position
        global sorting_bin
        while not rospy.is_shutdown():
            sorting_bin = -1  # Reset sorting_bin for new input
            while sorting_bin not in range(1, 5) and not rospy.is_shutdown():  # Loop until a valid sorting_bin is entered
                try:
                    sorting_bin = int(input("Select sorting bin (1-4): "))  # Prompt user for sorting bin
                except ValueError:
                    rospy.loginfo("Invalid input. Please enter a number between 1 and 4.")  # Log invalid input
                rospy.sleep(1.0)  # Sleep to avoid rapid re-prompting
            self.start_robot = True  # Set start_robot to True to begin sorting
            self.main_loop()  # Run main loop

    def main(self):
        global start_robot
        self.robot_controller.go_to_named_position('home')  # Move robot to home position
        while not rospy.is_shutdown():  # Loop while ROS is running
            if start_robot:  # Check if start_robot flag is True
                self.main_loop()  # Run main loop if start_robot is True
            rospy.sleep(1)  # Sleep to avoid rapid looping

    def main_loop(self):
        global start_robot, sorting_bin
        self.robot_state.data = "ACTIVE"  # Set robot state to ACTIVE
        self.write_robot_state.publish(self.robot_state)  # Publish robot state
        rospy.sleep(0.5)  # Sleep to allow state to publish
        self.robot_controller.go_to_named_position('home')  # Move robot to home position

        if sorting_bin != -1 and not emergency_stop:  # Check if sorting_bin is valid and emergency stop is false
            object_type = object_list[sorting_bin]  # Get the object type from object_list based on sorting_bin
            self.pickup(object_type)  # Call pickup method with object_type
            self.robot_controller.go_to_named_position('home')  # Move robot to home position
            self.sort(sorting_bin)  # Call sort method with sorting_bin
            self.robot_controller.go_to_named_position('home')  # Move robot to home position

        self.robot_state.data = "STANDBY"  # Set robot state to STANDBY
        self.write_robot_state.publish(self.robot_state)  # Publish robot state
        rospy.sleep(0.5)  # Sleep to allow state to publish
        start_robot = False  # Reset start_robot flag
        self.robot_controller.go_to_named_position('home')  # Move robot to home position

    def robot_start_callback(self, data):
        global start_robot
        rospy.loginfo("Received start signal: %s", data.data)  # Log received start signal
        start_robot = True  # Set start_robot flag to True

        # Noodstop bij toevoegen

    def inspection_result_callback(self, data):
        global sorting_bin
        rospy.loginfo("Received inspection result: %s", data.data)  # Log received inspection result
        sorting_bin = int(data.data)  # Set sorting_bin to received data

        #hier nog een update maken voor 'pickup' vanuit camera data

    def pickup(self, object_type):
        pickup_position_name = 'pickup_test' if self.test_mode else 'pickup'  # Determine pickup position name based on test_mode
        pickup_position = self.robot_controller.positions[pickup_position_name]  # Get pickup position from robot controller

        # Getting values from .json
        x = pickup_position['position'][0]  # X-coordinate of pickup position
        y = pickup_position['position'][1]  # Y-coordinate of pickup position
        z = pickup_position['position'][2]  # Z-coordinate of pickup position
        roll_pickup = pickup_position['orientation_rpy'][0]  # Roll orientation of pickup position
        pitch_pickup = pickup_position['orientation_rpy'][1]  # Pitch orientation of pickup position
        yaw_pickup = pickup_position['orientation_rpy'][2]  # Yaw orientation of pickup position

        # Calculating final xyz including object offset
        offset = pickup_offsets[object_type]  # Get offset for the object type
        x_pickup = x + offset[0]  # Calculate final x-coordinate with offset
        y_pickup = y + offset[1]  # Calculate final y-coordinate with offset
        z_pickup = z + offset[2] + 0.05  # Calculate final z-coordinate with offset and additional height

        # Updating position 'pickup_above'
        self.robot_controller.update_position('pickup_above', [x_pickup, y_pickup, z_pickup], [roll_pickup, pitch_pickup, yaw_pickup])
        print("Updated position pickup_above")  # Print update message

        # Hover above pickup location
        self.robot_controller.go_to_named_position('pickup_above')  # Move to position above pickup location
        self.gripper_controller.open_gripper()  # Open the gripper
        rospy.sleep(1.0)  # Sleep to allow gripper to open

        # Go to pickup location
        self.robot_controller.go_to_named_position(pickup_position_name)  # Move to pickup location
        rospy.sleep(0.5)  # Sleep to allow robot to reach position
        self.gripper_controller.close_gripper()  # Close the gripper to pick up the object
        rospy.sleep(1.0)  # Sleep to allow gripper to close
        self.robot_controller.go_to_named_position('pickup_above')

    def sort(self, sorting_bin):
        position_name = ('bin_{}'.format(sorting_bin))  # Determine bin position name based on sorting_bin
        self.robot_controller.go_to_named_position(position_name)  # Move to bin position
        rospy.sleep(1.0)  # Sleep to allow robot to reach position

        # Open the gripper
        self.gripper_controller.open_gripper()  # Open the gripper to release the object
        rospy.sleep(1.0)  # Sleep to allow gripper to open

    def ros_shutdown(self):
        rospy.loginfo("Shutting down...")  # Log shutdown message
        self.robot_controller.shutdown()  # Call shutdown method on robot controller

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)  # Initialize ROS node
    test_mode = input("Run test mode (Y/n): ").strip().lower() == "y"  # Prompt user for test mode
    main_instance = Main(test_mode=test_mode)  # Create Main instance with test_mode
    rospy.spin()  # Keep the script running