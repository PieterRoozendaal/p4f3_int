#!/usr/bin/env python

"""
Auteur: Jurre Fikkers
Studentnumber: 2196902
Description: Control unit for the manipulator and its movement
"""

# Import necessary libraries and modules
import sys  # System-specific parameters and functions
import rospy  # ROS Python library
import moveit_commander  # MoveIt! commander for robot motion planning
import moveit_msgs.msg  # MoveIt! messages
import geometry_msgs.msg  # ROS messages for robot poses
import json  # JSON library for handling JSON data
import tf  # ROS transform library
import math  # Mathematical functions
from geometry_msgs.msg import Pose  # Import Pose message

class MoveitRobotController:
    def __init__(self, positions_file):
        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize robot commander and planning scene interface
        self.robot = moveit_commander.RobotCommander()  # Robot commander for accessing robot's kinematic model
        self.scene = moveit_commander.PlanningSceneInterface()  # Planning scene interface for adding and removing objects in the scene
        
        # Initialize move group commander for the arm
        self.arm_group = moveit_commander.MoveGroupCommander("arm")  # Move group commander for the arm
        self.arm_group.set_planning_time(20)  # Set planning time to 20 seconds
        self.arm_group.set_num_planning_attempts(10)  # Set number of planning attempts to 10
        
        # Publisher for displaying the planned path
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',  # Topic name
            moveit_msgs.msg.DisplayTrajectory,  # Message type
            queue_size=1  # Queue size
        )
        
        # Load predefined positions from the JSON file
        self.positions_file = positions_file  # Store the positions file path
        self.positions = self.load_positions(positions_file)  # Load positions from the JSON file

    def load_positions(self, positions_file):
        # Load positions from the specified JSON file
        with open(positions_file, 'r') as f:
            self.positions = json.load(f)  # Load JSON data into self.positions
            for key, value in self.positions.items():  # Iterate through each position in the JSON file
                if 'orientation_rpy' in value:  # Check if orientation_rpy is in the position data
                    orientation_rpy = value['orientation_rpy']  # Get the roll, pitch, and yaw values
                    orientation_rpy_rad = [math.radians(angle) for angle in orientation_rpy]  # Convert angles to radians
                    quaternion = tf.transformations.quaternion_from_euler(
                        orientation_rpy_rad[0], orientation_rpy_rad[1], orientation_rpy_rad[2]
                    )  # Convert euler angles to quaternion
                    value['orientation'] = quaternion  # Update position data with quaternion orientation
                if 'position_offset' in value:  # Check if position_offset is in the position data
                    position_offset = value['position_offset']  # Get the position offset values
                    value['position'] = [
                        value['position'][0] + position_offset[0],  # Apply x offset
                        value['position'][1] + position_offset[1],  # Apply y offset
                        value['position'][2] + position_offset[2]   # Apply z offset
                    ]
        return self.positions  # Return the updated positions

    def go_to_named_position(self, position_name):
        # Move the robot to a named position
        if position_name in self.positions:  # Check if the position name exists in the positions
            position_data = self.positions[position_name]  # Get the position data
            if 'position' in position_data and 'orientation' in position_data:  # Check if both position and orientation are available
                pose_goal = geometry_msgs.msg.Pose()  # Create a new Pose message
                pose_goal.position.x = position_data['position'][0]  # Set x position
                pose_goal.position.y = position_data['position'][1]  # Set y position
                pose_goal.position.z = position_data['position'][2]  # Set z position

                pose_goal.orientation.x = position_data['orientation'][0]  # Set x orientation
                pose_goal.orientation.y = position_data['orientation'][1]  # Set y orientation
                pose_goal.orientation.z = position_data['orientation'][2]  # Set z orientation
                pose_goal.orientation.w = position_data['orientation'][3]  # Set w orientation

                self.arm_group.set_pose_target(pose_goal)  # Set the target pose for the arm
                self.arm_group.go(wait=True)  # Plan and execute the motion to the target pose
                self.arm_group.stop()  # Stop the arm movement
                self.arm_group.clear_pose_targets()  # Clear the pose targets
            else:
                rospy.logwarn("Position '{}' in the positions file is incomplete.".format(position_name))  # Log a warning if the position data is incomplete
        else:
            rospy.logwarn("Position '{}' not found in the positions file.".format(position_name))  # Log a warning if the position name is not found

    def update_position(self, position_name, position, orientation_rpy):
        # Update the position and orientation for a named position
        if position_name in self.positions:  # Check if the position name exists in the positions
            # Open and read JSON file
            with open(self.positions_file, 'r') as f:
                data = json.load(f)  # Load JSON data into data variable
            # Update position and orientation for specified position
            data[position_name]['position'] = position  # Update position data
            data[position_name]['orientation_rpy'] = orientation_rpy  # Update orientation_rpy data

            # Write update back to JSON
            with open(self.positions_file, 'w') as f:
                json.dump(data, f, indent=4)  # Write updated data back to the JSON file

    def go_to_xyz(self, x, y, z):
        # Move the robot to specific XYZ coordinates
        rospy.loginfo("Moving to XYZ coordinates: {}, {}, {}".format(x, y, z))  # Log the target coordinates
        pose_goal = geometry_msgs.msg.Pose()  # Create a new Pose message
        pose_goal.position.x = x  # Set x position
        pose_goal.position.y = y  # Set y position
        pose_goal.position.z = z  # Set z position
        self.arm_group.set_pose_target(pose_goal)  # Set the target pose for the arm
        self.arm_group.go(wait=True)  # Plan and execute the motion to the target pose
        self.arm_group.stop()  # Stop the arm movement
        self.arm_group.clear_pose_targets()  # Clear the pose targets

    def publish_current_pose(self):
        # Get current pose of the end effector and publish it
        current_pose = self.arm_group.get_current_pose().pose  # Get the current pose of the end effector
        self.pose_publisher.publish(current_pose)  # Publish the current pose

    def shutdown(self):
        # Shutdown moveit_commander properly
        moveit_commander.roscpp_shutdown()  # Call the roscpp_shutdown method to properly shutdown MoveIt!

def get_user_choice():
    # Get the user's choice for the operation type
    try:
        choice = int(input("Choose operation (1: Named Position, 2: XYZ Coordinates): "))  # Prompt user for choice
        if choice in [1, 2]:  # Check if the choice is valid
            return choice  # Return the valid choice
        else:
            print("Invalid choice. Please enter 1 or 2.")  # Print invalid choice message
            return get_user_choice()  # Re-prompt for choice
    except ValueError:
        print("Invalid input. Please enter a number.")  # Print invalid input message
        return get_user_choice()  # Re-prompt for choice

def main():
    positions_file = "positions.json"  # Specify the positions file
    controller = MoveitRobotController(positions_file)  # Create a MoveitRobotController instance
    
    try:
        while not rospy.is_shutdown():  # Loop while ROS is running
            user_choice = get_user_choice()  # Get the user's choice for the operation type
            
            if user_choice == 1:
                position_name = input("Enter named position: ")  # Prompt user for named position
                controller.go_to_named_position(position_name)  # Move to the named position
            elif user_choice == 2:
                x = float(input("Enter X coordinate: "))  # Prompt user for X coordinate
                y = float(input("Enter Y coordinate: "))  # Prompt user for Y coordinate
                z = float(input("Enter Z coordinate: "))  # Prompt user for Z coordinate
                controller.go_to_xyz(x, y, z)  # Move to the specified XYZ coordinates
    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exception
    finally:
        controller.shutdown()  # Shutdown the controller properly

if __name__ == '__main__':
    main()  # Call the main function
    