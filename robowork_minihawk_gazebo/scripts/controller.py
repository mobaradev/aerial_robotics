#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
# from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool
from mavros_msgs.msg import OverrideRCIn
from tf import TransformListener
from nav_msgs.msg import Odometry
import time
import os

current_mode = ""
current_stage = 0

is_cqloiter_mode = False
is_in_descending_state = False

qloiter_start_time = 0
qland_preinit_time = 0

detections_counter = 0

def callback(data):
    global channel_a
    global channel_b
    global channel_c
    global channel_d
    global is_cqloiter_mode
    global is_in_descending_state
    global qloiter_start_time
    global qland_preinit_time
    global detections_counter

    if len(data.detections) > 1:
        print("[ERROR] - multiple detections")
 
    for detection in data.detections:
        detections_counter += 1

        if qland_preinit_time != 0:
            qland_preinit_time = 0

        if detections_counter < 100:
            print("Waiting " + str(detections_counter) + " / 100...")
            return

        #rospy.loginfo(rospy.get_caller_id() + '\nDetected: %s', detection.id)
        #rospy.loginfo(detection)

        yaw = detection.pose.pose.pose.orientation.x

        x = detection.pose.pose.pose.position.x
        y = detection.pose.pose.pose.position.y
        z = detection.pose.pose.pose.position.z
        os.system('clear')
        print("x: " + str(x))
        print("y: " + str(y))
        print("z: " + str(z))
        print("t: " + str(time.time() - qloiter_start_time))

        channel_a = 0
        channel_b = 0
        channel_c = 0
        channel_d = 0

        if yaw > 0.975 or yaw < 0.0025:
            # yaw is good, then proceed with x, y centering
            channel_d = 0

            if x > 0.65:
                channel_a = 35
            elif x < -0.65:
                channel_a = -35
            else:
                channel_a = 0

            if y > 0.8:
                channel_b = -85
            elif y > 0.6:
                channel_b = -65
            elif y > 0.25:
                channel_b = -35
            elif y < -0.8:
                channel_b = 85
            elif y < -0.6:
                channel_b = 64
            elif y < -0.25:
                channel_b = 35
            else:
                channel_b = 0
        else:
            # fix yaw
            channel_d = 100

        if z > 3:
            channel_c = -110
        else:
            channel_c = 0

        if not is_cqloiter_mode:
            is_cqloiter_mode = True
            is_in_descending_state = True

            set_mode("QLOITER")
            qloiter_start_time = time.time()
        else:
            if detections_counter%3 == 0:
                handle_position()



        # if current_stage == 1:
            # set_mode("GUIDED")
            # current_stage = 2

    if len(data.detections) == 0:
        rospy.loginfo("No detections")


        if is_in_descending_state and time.time() - qloiter_start_time > 10:
            # call QLAND mode
            if qland_preinit_time == 0:
                qland_preinit_time = time.time()
            elif time.time() - qland_preinit_time > 22:
                set_mode('QLAND')
                rospy.loginfo("QLAND mode")
            else:
                print(str(-1*(22 - (time.time() - qland_preinit_time))) + "s to start QLAND mode...")
                handle_position()

channel_a = 0
channel_b = 0
channel_c = 0
channel_d = 0

def handle_position():
    global channel_a
    global channel_b
    global channel_c
    global channel_d
    #rospy.init_node('center_nopde', anonymous=True)
    # print("center")
    pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size = 10)
    r = rospy.Rate(10) #10hz
    msg = OverrideRCIn()
    start = time.time()
    exec_time = 1
    flag = True

    steer_channel = 0 # roll/steer movement
    pitch_channel = 1 # pitch movement
    throttle_channel = 2 # throttle up/down movement
    yaw_channel = 3 # yaw movement

    msg.channels[0] = 1500 + channel_a
    msg.channels[1] = 1500 + channel_b
    msg.channels[2] = 1500 + channel_c
    msg.channels[3] = 1500 + channel_d

    while not rospy.is_shutdown() and flag:
        flag = False
        sample_time = time.time()
        if ((sample_time - start) > exec_time):
            flag = False

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

def listener():

    rospy.loginfo(rospy.get_caller_id() + 'Listener starts')

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', AprilTagDetectionArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def set_mode(mode_name):
    #rospy.init_node('talker', anonymous=True)
    
    set_mode_client = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
    rate = rospy.Rate(10) # 10hz
    
    o = SetModeRequest()
    o.custom_mode = mode_name

    feedback = set_mode_client.call(o).mode_sent

    # pub.publish(message)
    rate.sleep()
    print("mode set: " + str(feedback))
    current_mode = mode_name

def set_arming(arming_state):
    arming_client = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)
    response = arming_client(arming_state)
    return response.success


if __name__ == '__main__':
    print("Set arming: " + str(set_arming(True)))
    listener()
    set_mode("AUTO")
