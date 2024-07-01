#!/usr/bin/env python

"""
Auteur: Jurre Fikkers
Studentnumber: 2196902
Description: Main control script for robot operation
"""

import rospy
import json
import sys
import tf
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool, Int32
from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from robot_controller import MoveitRobotController

number_of_bins = 4
error_message = "error"
object_list = ["", "obj1", "obj2", "obj3", "obj4"]
hmi_signal = False
emergency_signal = False
sorting_bin = -1

pickup_offsets = {
    "obj1": [0.0, 0.0, 0.05],  
    "obj2": [0.0, 0.0, 0.05],
    "obj3": [0.0, 0.0, 0.05],  
    "obj4": [0.0, 0.0, 0.05]  
}

class Main:
    def __init__(self, test_mode=False):
        rospy.on_shutdown(self.ros_shutdown)
        self.robot_controller = MoveitRobotController("positions.json")
        self.gripper_controller = NiryoRosWrapper()
        
        self.read_program_start = rospy.Subscriber('/hmi_signal', Bool, self.hmi_signal_callback)
        self.read_emergency_stop = rospy.Subscriber('/emergency_signal', Bool, self.emergency_signal_callback)
        self.read_inspection_results = rospy.Subscriber('/robot_sort_pose', String, self.inspection_result_callback)
        
        self.write_robot_state = rospy.Publisher('/robot_status', String, queue_size=1)
        
        rospy.sleep(0.5)
        self.robot_state = String(data="STANDBY")
        self.write_robot_state.publish(self.robot_state)
        rospy.sleep(0.5)
        
        self.test_mode = test_mode

        if test_mode:
            self.run_test_mode()
        else:
            self.main()

    def run_test_mode(self):
        self.robot_controller.go_to_named_position('home')
        global sorting_bin
        while not rospy.is_shutdown():
            sorting_bin = -1
            while sorting_bin not in range(1, 5) and not rospy.is_shutdown():
                try:
                    sorting_bin = int(input("Select sorting bin (1-4): "))
                    x_test = float(input("Enter x coordinate: "))
                    y_test = float(input("Enter y coordinate: "))
                    orientation_test = float(input("Enter orientation: "))
                except ValueError:
                    rospy.loginfo("Invalid input. Please enter valid coordinates and orientation.")
                    rospy.sleep(1.0)
                    continue
            
            print("sorting_bin set as bin_{} via test_mode".format(sorting_bin))
            self.robot_controller.update_position('pickup', [x_test, y_test, 0.12], [0.0, 90.0, orientation_test])
            print("pickup position updated via test_mode")
            rospy.sleep(1.0)
            global hmi_signal
            hmi_signal = True
            self.main_loop()

    def main(self):
        global hmi_signal
        self.robot_controller.go_to_named_position('home')
        print("Moved to 'home' position")
        while not rospy.is_shutdown():
            if hmi_signal:
                self.main_loop()
            rospy.sleep(1)

    def main_loop(self):
        global hmi_signal, sorting_bin, emergency_signal
        self.robot_state.data = "ACTIVE"
        self.write_robot_state.publish(self.robot_state)
        rospy.sleep(0.5)
        self.robot_controller.go_to_named_position('home')

        if sorting_bin != -1 and not emergency_signal:
            object_type = object_list[sorting_bin]
            self.update_pickup_above(object_type)
            self.pickup(object_type)
            self.robot_controller.go_to_named_position('home')
            print("Moved to 'home' position")
            self.sort(sorting_bin)
            self.robot_controller.go_to_named_position('home')
            print("Moved to 'home' position")

        self.robot_state.data = "STANDBY"
        self.write_robot_state.publish(self.robot_state)
        rospy.sleep(0.5)

    def hmi_signal_callback(self, data):
        global hmi_signal
        rospy.loginfo("Received start signal: %s", data.data)
        hmi_signal = data.data

    def emergency_signal_callback(self, data):
        global emergency_signal
        rospy.loginfo("Received emergency stop signal: %s", data.data)
        emergency_signal = data.data
        if emergency_signal:
            self.emergency_stop()

    def emergency_stop(self):
        # Immediately stop all robot movements
        rospy.logwarn("Emergency stop activated! Halting all movements.")
        self.robot_controller.stop_all_movements()
        self.gripper_controller.stop_all_movements()
        self.robot_state.data = "EMERGENCY_STOP"
        self.write_robot_state.publish(self.robot_state)
        rospy.sleep(0.5)

    def inspection_result_callback(self, data):
        global sorting_bin
        rospy.loginfo("Received inspection result: %s", data.data)
        sorting_bin = int(data.data)

    def update_pickup_above(self, object_type):
        pickup_position = self.robot_controller.positions['pickup']

        x = pickup_position['position'][0]
        y = pickup_position['position'][1]
        z = pickup_position['position'][2]
        roll_pickup = pickup_position['orientation_rpy'][0]
        pitch_pickup = pickup_position['orientation_rpy'][1]
        yaw_pickup = pickup_position['orientation_rpy'][2]

        offset = pickup_offsets[object_type]
        x_pickup = x + offset[0]
        y_pickup = y + offset[1]
        z_pickup = z + offset[2]

        self.robot_controller.update_position('pickup_above', [x_pickup, y_pickup, z_pickup], [roll_pickup, pitch_pickup, yaw_pickup])
        print("Updated position pickup_above to: {} with orientation: {}".format([x_pickup, y_pickup, z_pickup], [roll_pickup, pitch_pickup, yaw_pickup]))
        rospy.sleep(2.0)
    
    def pickup(self, object_type):
        self.robot_controller.go_to_named_position('pickup_above')
        print("Moved to 'pickup_above' position")
        self.gripper_controller.open_gripper()
        rospy.sleep(1.0)

        self.robot_controller.go_to_named_position('pickup')
        print("Moved to 'pickup' position")
        rospy.sleep(0.5)
        self.gripper_controller.close_gripper()
        print("Grabbed object")
        rospy.sleep(1.0)
        self.robot_controller.go_to_named_position('pickup_above')
        print("Moved to 'pickup_above' position")

    def sort(self, sorting_bin):
        position_name = ('bin_{}'.format(sorting_bin))
        self.robot_controller.go_to_named_position(position_name)
        print("Moved to bin_{}".format(sorting_bin))
        rospy.sleep(1.0)
        self.gripper_controller.open_gripper()
        print("Released object")
        rospy.sleep(1.0)

    def ros_shutdown(self):
        rospy.loginfo("Shutting down...")
        self.robot_controller.shutdown()

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)
    test_mode = input("Run test mode (Y/n): ").strip().lower() == "y"
    main_instance = Main(test_mode=test_mode)
    rospy.spin()
