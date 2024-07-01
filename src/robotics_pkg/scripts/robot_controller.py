import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import json
import tf
import math
from geometry_msgs.msg import Pose

class MoveitRobotController:
    def __init__(self, positions_file):
        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize robot commander and planning scene interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize move group commander for the arm
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_group.set_planning_time(20)
        self.arm_group.set_num_planning_attempts(10)
        
        # Publisher for displaying the planned path
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1
        )
        
        # Load predefined positions from the JSON file
        self.positions_file = positions_file
        self.positions = self.load_positions(positions_file)

    def load_positions(self, positions_file):
        with open(positions_file, 'r') as f:
            positions = json.load(f)
            for key, value in positions.items():
                if 'orientation_rpy' in value:
                    orientation_rpy = value['orientation_rpy']
                    orientation_rpy_rad = [math.radians(angle) for angle in orientation_rpy]
                    quaternion = tf.transformations.quaternion_from_euler(
                        orientation_rpy_rad[0], orientation_rpy_rad[1], orientation_rpy_rad[2]
                    )
                    value['orientation'] = quaternion
                if 'position_offset' in value:
                    position_offset = value['position_offset']
                    value['position'] = [
                        value['position'][0] + position_offset[0],
                        value['position'][1] + position_offset[1],
                        value['position'][2] + position_offset[2]
                    ]
            return positions

    def update_position(self, position_name, position, orientation):
        if position_name in self.positions:
            self.positions[position_name]['position'] = position
            self.positions[position_name]['orientation_rpy'] = orientation
            # Convert orientation from degrees to radians, then to quaternion
            quaternion = tf.transformations.quaternion_from_euler(
                math.radians(orientation[0]),
                math.radians(orientation[1]),
                math.radians(orientation[2])
            )
            self.positions[position_name]['orientation'] = quaternion
            rospy.loginfo("Position updated via controller")
        else:
            rospy.logwarn("Position name '{}' not found.".format(position_name))

    def go_to_named_position(self, position_name):
        if position_name in self.positions:
            position_data = self.positions[position_name]
            if 'position' in position_data and 'orientation' in position_data:
                pose_goal = geometry_msgs.msg.Pose()
                
                # Apply position offset if available
                if 'position_offset' in position_data:
                    offset = position_data['position_offset']
                    pose_goal.position.x = position_data['position'][0]
                    pose_goal.position.y = position_data['position'][1] 
                    pose_goal.position.z = position_data['position'][2] 
                else:
                    pose_goal.position.x = position_data['position'][0]
                    pose_goal.position.y = position_data['position'][1]
                    pose_goal.position.z = position_data['position'][2]

                pose_goal.orientation.x = position_data['orientation'][0]
                pose_goal.orientation.y = position_data['orientation'][1]
                pose_goal.orientation.z = position_data['orientation'][2]
                pose_goal.orientation.w = position_data['orientation'][3]
                
                self.arm_group.set_pose_target(pose_goal)
                plan = self.arm_group.go(wait=True)
                self.arm_group.stop()
                self.arm_group.clear_pose_targets()

                if plan:
                    rospy.loginfo("Successfully moved to position")
                else:
                    rospy.logwarn("Failed to move to position")
            else:
                rospy.logwarn("Position in the positions file is incomplete.")
        else:
            rospy.logwarn("Position not found in the positions file.")

    def go_to_xyz(self, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if plan:
            rospy.loginfo("Successfully moved to the XYZ coordinates.")
        else:
            rospy.logwarn("Failed to move to the XYZ coordinates.")

    def stop_all_movements(self):
        self.arm_group.stop()
        rospy.loginfo("Stopped all arm movements.")
        
    def shutdown(self):
        moveit_commander.roscpp_shutdown()

def get_user_choice():
    try:
        choice = int(input("Choose operation (1: Named Position, 2: XYZ Coordinates): "))
        if choice in [1, 2]:
            return choice
        else:
            print("Invalid choice. Please enter 1 or 2.")
            return get_user_choice()
    except ValueError:
        print("Invalid input. Please enter a number.")
        return get_user_choice()

def main():
    positions_file = "positions.json"
    controller = MoveitRobotController(positions_file)
    
    try:
        while not rospy.is_shutdown():
            user_choice = get_user_choice()
            
            if user_choice == 1:
                position_name = input("Enter named position: ")
                controller.go_to_named_position(position_name)
            elif user_choice == 2:
                x = float(input("Enter X coordinate: "))
                y = float(input("Enter Y coordinate: "))
                z = float(input("Enter Z coordinate: "))
                controller.go_to_xyz(x, y, z)
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()