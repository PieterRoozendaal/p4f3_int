#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from depthai_ros_msgs.msg import SpatialDetectionArray
from robotics_pkg.srv import lokalisatie, lokalisatieResponse

listener = None
service = None  # Define a global variable to hold the service object

def lokalisatie_detectie(req):
    global listener, service
    
    data = rospy.wait_for_message("/stereo_inertial_nn_publisher/color/detections", SpatialDetectionArray)

    xgem = 0
    ygem = 0
    zgem = 0

    for _ in range(10):
        if not data.detections:
            rospy.loginfo("Geen detecties gevonden.")
            return lokalisatieResponse(Xw=0, Yw=0, Naam="")

        eerste_detectie = data.detections[0]
        eerste_resultaat_id = eerste_detectie.results[0].id

        if eerste_resultaat_id == 1:
            rospy.loginfo("Resultaat van de herkenning is: dop_10")
            item = "Dop_10_0"
            naam = "Dop_10"
        elif eerste_resultaat_id == 2:
            rospy.loginfo("Resultaat van de herkenning is: Kleine Schroevendraaier")
            item = "Kleine_Schroevendraaier_0"
            naam = "KleineSchroevendraaier"
        elif eerste_resultaat_id == 3:
            rospy.loginfo("Resultaat van de herkenning is: Platte kop")
            item = "PlatteKop_0"
            naam = "PlatteKop"
        elif eerste_resultaat_id == 4:
            rospy.loginfo("Resultaat van de herkenning is: Spanningstester")
            item = "Spanningstester_0"
            naam = "Spanningstester"
        else:
            rospy.logwarn("Onjuiste waarde gelezen: %i", eerste_resultaat_id)
            return lokalisatieResponse(Xw=0, Yw=0, Naam="")

        # Start waarde met een '/' als TF.
        item = "/" + item

        try:
            # Wacht maximaal 1 seconde op de transform
            listener.waitForTransform('/world', item, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('/world', item, rospy.Time(0))
            rospy.loginfo("Translation: %s", str(trans))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF fout: %s", str(e))
            return lokalisatieResponse(Xw=0, Yw=0, Naam=naam)

        xgem += trans[0]
        ygem += trans[1]
        zgem += trans[2]

    xgem = xgem/10
    ygem = ygem/10
    zgem = zgem/10 
    return lokalisatieResponse(Xw=xgem, Yw=ygem, Naam=naam)

def listener_node():
    global listener, service
    rospy.init_node('lokalisatie_camera')
    listener = tf.TransformListener()
    
    service = rospy.Service('lokalisatie', lokalisatie, lokalisatie_detectie)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_node()
    except rospy.ROSInterruptException:
        pass
