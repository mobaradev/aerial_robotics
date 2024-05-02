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

def set_mode(mode_name):
    rospy.init_node('talker', anonymous=True)
    
    set_mode_client = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
    rate = rospy.Rate(10) # 10hz
    
    o = SetModeRequest()
    o.custom_mode = mode_name

    feedback = set_mode_client.call(o).mode_sent

    # pub.publish(message)
    rate.sleep()
    print("mode set: " + str(feedback))

if __name__ == '__main__':
    #listener()
    set_mode("AUTO")
