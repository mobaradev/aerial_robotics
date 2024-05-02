#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.srv import SetMode, SetModeRequest

def callback(data):
    for detection in data.detections:
        rospy.loginfo(rospy.get_caller_id() + '\nDetected: %s', detection.id)
        rospy.loginfo(detection)

    if len(data.detections) == 0:
        rospy.loginfo("No detections")

def listener():

    rospy.loginfo(rospy.get_caller_id() + 'Listener starts')

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', AprilTagDetectionArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
