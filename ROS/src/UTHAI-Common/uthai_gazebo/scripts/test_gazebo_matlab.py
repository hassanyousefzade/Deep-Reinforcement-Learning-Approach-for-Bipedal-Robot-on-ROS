#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time


def main():
    rospy.init_node("joint_control_node")
    LAP = rospy.Publisher(
        'uthai/l_ankle_pitch_position/command', Float64, queue_size=10)
    LAR = rospy.Publisher(
        'uthai/l_ankle_roll_position/command', Float64, queue_size=10)
    LHP = rospy.Publisher(
        'uthai/l_hip_pitch_position/command', Float64, queue_size=10)
    LHR = rospy.Publisher('uthai/l_hip_roll_position/command',
                          Float64, queue_size=10)
    LHY = rospy.Publisher('uthai/l_hip_yaw_position/command',
                          Float64, queue_size=10)
    LKP = rospy.Publisher(
        'uthai/l_knee_pitch_position/command', Float64, queue_size=10)
    RAP = rospy.Publisher(
        'uthai/r_ankle_pitch_position/command', Float64, queue_size=10)
    RAR = rospy.Publisher(
        'uthai/r_ankle_roll_position/command', Float64, queue_size=10)
    RHP = rospy.Publisher(
        'uthai/r_hip_pitch_position/command', Float64, queue_size=10)
    RHR = rospy.Publisher('uthai/r_hip_roll_position/command',
                          Float64, queue_size=10)
    RHY = rospy.Publisher('uthai/r_hip_yaw_position/command',
                          Float64, queue_size=10)
    RKP = rospy.Publisher(
        'uthai/r_knee_pitch_position/command', Float64, queue_size=10)
    # rate = rospy.Rate(100)
    time.sleep(1)
    LHY.publish(0.5)
    RHY.publish(-0.5)
    LAR.publish(0)
    LHR.publish(0)
    RAR.publish(0)
    RHR.publish(0)

    LHP.publish(0)
    LKP.publish(0)
    LAP.publish(0)
    RHP.publish(0)
    RKP.publish(0)
    RAP.publish(0)
    time.sleep(5)
    counter= 0
    while not rospy.is_shutdown():
        time.sleep(5)
        LHP.publish(1)
        LKP.publish(1)
        LAP.publish(1)
        RHP.publish(1)
        RKP.publish(1)
        RAP.publish(1)
        time.sleep(5)
        LHP.publish(1)
        LKP.publish(1)
        LAP.publish(1)
        RHP.publish(1)
        RKP.publish(1)
        RAP.publish(1)
        counter= counter+1
        if counter ==2 :
          break
        


if __name__ == '__main__':
    main()
