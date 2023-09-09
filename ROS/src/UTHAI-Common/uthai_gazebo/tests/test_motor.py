#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np


class Uthai(object):

    def __init__(self, ns='/uthai/'):
        self.ns = ns
        self.joints = None
        self.angles = None
        self.velocities = None
        self.efforts = None

        rospy.loginfo("Init Uthai robot")
        self._sub_joints = rospy.Subscriber(
            self.ns + "joint_states", JointState, self._cb_joints, queue_size=1)
        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")

        rospy.loginfo("Creating joint command publishers")
        self._pub_joints = {}
        for j in self.joints:
            self._pub_joints[j] = rospy.Publisher(self.ns + j + "_p_controller/command", Float64, queue_size=1)
        
        rospy.loginfo("Ready")
    
    
    def _cb_joints(self, msg):
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position
        self.velocities = msg.velocity
        self.efforts = msg.effort

    def get_angles(self):
        if self.joints is None:
            return None
        return dict(zip(self.joints,self.angles))

    def get_velocities(self):
        if self.joints is None:
            return None
        return dict(zip(self.joints,self.velocities))
    
    def set_angles(self,angles):
        for j,v in angles.items():
            if j not in self.joints:
                rospy.logerr("Invalid joint name" + j)
                continue
            self._pub_joints[j].publish(v)
            rospy.loginfo("{},{}".format(j,v))


def main():
    rospy.init_node("walker")
    rospy.loginfo("Instantiating Uthai Client")
    uthai = Uthai()
    # rospy.loginfo("{}".format(uthai.get_angles()))
    # rospy.loginfo("{}".format(uthai.get_velocities()))
    # rospy.sleep(3)
    rospy.loginfo("Set Angle")

    data =  {
        'r_j_knee_pitch': 0.0,
        'l_j_hip_pitch': 0.0,
        'r_j_ankle_pitch': 0.0,
        'r_j_ankle_roll': 0.0,
        'l_j_knee_pitch': 0.0,
        'l_j_ankle_roll': 0.0,
        'r_j_hip_yaw': 0.0,
        'l_j_ankle_pitch': 0.0,
        'r_j_hip_pitch': 0.0,
        'l_j_hip_yaw': 0.0,
        'l_j_hip_roll': 0.0,
        'r_j_hip_roll': 0.0
    }
    uthai.set_angles(data)
    rospy.loginfo("Complete")
    # rospy.sleep(3)
    while not rospy.is_shutdown():
        for x in range(1000):
            v = -x * np.pi/1000
            uthai.set_angles({'r_j_knee_pitch': v,'l_j_knee_pitch': v})
            rospy.sleep(0.005)


if __name__ == '__main__':
    main()
