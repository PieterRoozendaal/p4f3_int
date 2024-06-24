#!/usr/bin/env python

import rospy
import geometry_msgs.msg

def pose_callback(data):
    rospy.loginfo("Received pose: x: {}, y: {}, z: {}".format(
        data.position.x,
        data.position.y,
        data.position.z
    ))

def pose_listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/robot_current_pose', geometry_msgs.msg.Pose, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        pose_listener()
    except rospy.ROSInterruptException:
        pass