#!/usr/bin/env python

import rospy
from your_package_name.srv import AddTwoInts

def add_two_ints_client(a, b):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        response = add_two_ints(a, b)
        return response.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('add_two_ints_client')
    a, b = 5, 7
    rospy.loginfo(f"Requesting {a} + {b}")
    result = add_two_ints_client(a, b)
    rospy.loginfo(f"Sum: {result}")
