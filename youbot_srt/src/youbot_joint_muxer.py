#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import JointState

roslib.load_manifest('youbot_srt')


class ybJointMuxer():
    def __init__(self, jsList=[]):
        self.jountStatesList = jsList
        self.sources = []
        for jStates in self.jountStatesList:
            self.sources.append(rospy.Subscriber(jStates, JointState,
                                self.source_cb))
        self.pub = rospy.Publisher('joint_states', JointState)
        self.names = []
        self.positions = []
        self.velocities = []
        self.efforts = []

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name in self.names:
                x = self.names.index(name)
                self.positions[x] = msg.position[i]
                self.velocities[x] = msg.velocity[i]
                if len(msg.effort) > 0:
                    self.efforts[x] = msg.effort[i]
                else:
                    self.efforts[x] = 0.0
            else:
                self.names.append(name)
                self.positions.append(msg.position[i])
                self.velocities.append(msg.velocity[i])
                if len(msg.effort) > 0:
                    self.efforts.append(msg.effort[i])
                else:
                    self.efforts.append(0.0)

    def publishJointStates(self):
        jState = JointState()
        jState.header.stamp = rospy.Time.now()
        jState.name = self.names
        jState.position = self.positions
        jState.velocity = self.velocities
        jState.effort = self.efforts
        self.pub.publish(jState)

    def ybspin(self):
        self.publishJointStates()
        rospy.sleep(0.1)


def ybmuxer():
    rospy.init_node('youbot_joint_muxer')
    ybJointList = ['/arm_1/joint_states', '/base/joint_states']
    youbotMuxer = ybJointMuxer(ybJointList)
    while not rospy.is_shutdown():
        youbotMuxer.ybspin()


if __name__ == '__main__':
    try:
        ybmuxer()
    except rospy.ROSInterruptException:
        pass
