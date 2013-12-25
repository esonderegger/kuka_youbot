#!/usr/bin/env python

import roslib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import GripperCommandAction
from sensor_msgs.msg import JointState
from actionlib import SimpleActionServer
import yaml
import datetime
import lib_dynamixel

roslib.load_manifest('ax12_controller')

ids = [1, 2, 3, 4, 5, 6, 7]
device = '/dev/ttyACM0'
dynamixelChain = lib_dynamixel.Dynamixel_Chain(device, 1000000, ids)

ax12Dir = roslib.packages.get_pkg_dir('ax12_controller')
configData = yaml.load(open(ax12Dir + '/config/ax12_arm_configuration.yaml'))
configJoints = configData['joints']

joints = []
for j in configData['joints']:
    name = j['name']
    if 'mimic_joints' in j:
        mimics = j['mimic_joints']
    else:
        mimics = []
    joints.append({'name': name, 'mimic_joints': mimics})


# def jointMover(jointID, position, velocity):
#     startTime = datetime.datetime.now()
#     activeJoint = joints[jointID]
#     for servo in activeJoint['servos']:
#         servo.move_angle(position, velocity, blocking=False)
#     endTime = datetime.datetime.now()
#     print 'jointMover took: ' + str(endTime - startTime) + 'for: ' + str(jointID)


def nonZeroVelocities(velocities):
    # sometimes moveit gives us velocities of 0.0, when we want something
    # very slow, but the lib_robotis takes 0 as "unlimited"
    outVels = []
    for velocity in velocities:
        if velocity == 0.0:
            outVels.append(0.1)
        else:
            outVels.append(velocity)
    return outVels


def matchServoVals(jNames, vals):
    outVals = []
    for x in range(len(configData['joints'])):
        for y in range(len(jNames)):
            if jNames[y] == configData['joints'][x]['name']:
                for servo in configData['joints'][x]['id']:
                    outVals.append(vals[y])
    return outVals


joint_state_pub = rospy.Publisher('/joint_states', JointState)
jointNames = []
for configJoint in configJoints:
    jointNames.append(configJoint['name'])
    if 'mimic_joints' in configJoint:
        for mimic in configJoint['mimic_joints']:
            jointNames.append(mimic['name'])

jPositions = []


def idValsToJoints(idVals):
    # this is ugly and way too specific for the ax-12 arm
    out = [idVals[0]]
    out.append(idVals[1])
    out.append(idVals[3])
    out.append(idVals[5])
    out.append(idVals[6])
    out.append(-idVals[6])
    out.append(idVals[6])
    out.append(-idVals[6])
    out.append(-idVals[6])
    out.append(-idVals[6])
    return out


def publishJointStates():
    jState = JointState()
    jState.header.stamp = rospy.Time.now()
    jState.name = jointNames
    angles, angvels, myids = dynamixelChain.read_angs_angvels(ids)
    global jPositions
    jPositions = idValsToJoints(angles)
    jState.position = jPositions
    jState.velocity = idValsToJoints(angvels)
    jState.effort = [0.0] * 10
    joint_state_pub.publish(jState)


class axServer:
    def __init__(self, name):
        self.fullname = name
        self.jointPositions = []
        self.jointVelocities = []
        self.jointAccelerations = []
        self.jointNamesFromConfig = []
        for configJoint in configJoints:
            self.jointNamesFromConfig.append(configJoint['name'])
            self.jointPositions.append(0.0)
            self.jointVelocities.append(0.0)
            self.jointAccelerations.append(0.0)
            if 'mimic_joints' in configJoint:
                for mimic in configJoint['mimic_joints']:
                    self.jointNamesFromConfig.append(mimic['name'])
                    self.jointPositions.append(0.0)
                    self.jointVelocities.append(0.0)
                    self.jointAccelerations.append(0.0)
        self.pointsQueue = []
        self.lastTimeFromStart = 0.0
        startPositions = [0.0, -0.9, -0.9, 0.0, 0.0, 0.0]
        startVelocities = [0.5]*6
        self.jointPositions[1] = -0.9
        dynamixelChain.move_angles_sync(ids[:6], startPositions, startVelocities)
        # self.joint_state_pub = rospy.Publisher('/joint_states', JointState)
        self.server = SimpleActionServer(self.fullname,
                                         FollowJointTrajectoryAction,
                                         execute_cb=self.execute_cb,
                                         auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        startTime = rospy.Time.now().to_sec()
        rospy.loginfo(goal)
        jNames = goal.trajectory.joint_names
        self.pointsQueue = goal.trajectory.points
        for a in range(len(self.pointsQueue)):
            while rospy.Time.now().to_sec() - startTime < self.pointsQueue[a].time_from_start.to_sec():
                rospy.sleep(0.01)
                print 'sleeping...'
            rospy.loginfo(rospy.Time.now().to_sec() - startTime)
            if rospy.Time.now().to_sec() - startTime - self.pointsQueue[a].time_from_start.to_sec() < 0.05:
                # self.executePoint(point, jNames)
                if a + 1 < len(self.pointsQueue):
                    self.executePoint(self.pointsQueue[a], self.pointsQueue[a + 1], jNames)
                else:
                    self.executePoint(self.pointsQueue[a], self.pointsQueue[a], jNames)
            # self.publishJointStates()
        self.lastTimeFromStart = rospy.Duration.from_sec(0.0)
        self.pointsQueue = []
        self.server.set_succeeded()

    # def executePoint(self, point, jNames):
    def executePoint(self, point, nextPoint, jNames):
        # rospy.loginfo(point)
        startTime = datetime.datetime.now()
        print 'point time from start is: ' + str(point.time_from_start.to_sec())
        for x in range(len(jNames)):
            for y in range(len(self.jointPositions)):
                if jNames[x] == self.jointNamesFromConfig[y]:
                    self.jointPositions[y] = point.positions[x]
                    self.jointVelocities[y] = point.velocities[x]
                    self.jointAccelerations[y] = point.accelerations[x]
                    # jointMover(y, point.positions[x],
                    #            nonZeroVelocity(point.velocities[x]))
        poss = matchServoVals(jNames, point.positions)
        poss = matchServoVals(jNames, nextPoint.positions)
        print poss
        vels = nonZeroVelocities(matchServoVals(jNames, point.velocities))
        print vels
        accs = matchServoVals(jNames, point.accelerations)
        print accs
        dynamixelChain.move_angles_sync(ids[:6], poss, vels)
        endTime = datetime.datetime.now()
        print 'executePoint took: ' + str(endTime - startTime)

    def publishJointStates(self):
        # rospy.loginfo(self.jointPositions)
        jState = JointState()
        jState.header.stamp = rospy.Time.now()
        jState.name = self.jointNamesFromConfig
        jState.position = self.jointPositions
        jState.velocity = self.jointVelocities
        jState.effort = self.jointAccelerations
        self.joint_state_pub.publish(jState)


class axGripperServer:
    def __init__(self, name):
        self.fullname = name
        self.currentAngle = 0.0
        # dynamixelChain.move_angle(7, 0.0, 0.5)
        dynamixelChain.move_angles_sync(ids[6:], [0.0], [0.5])
        self.server = SimpleActionServer(self.fullname,
                                         GripperCommandAction,
                                         execute_cb=self.execute_cb,
                                         auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo(goal)
        self.currentAngle = goal.command.position
        dynamixelChain.move_angles_sync(ids[6:], [self.currentAngle], [0.5])
        # dynamixelChain.move_angle(7, 0.1, 0.5)
        rospy.sleep(0.1)
        print jPositions[4]
        attempts = 0
        while abs(goal.command.position - jPositions[4]) > 0.1 and attempts < 20:
            rospy.sleep(0.1)
            print jPositions[4]
            attempts += 1
        if attempts < 20:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()


def axserver():
    rospy.init_node('ax12_action_server')
    armServer = axServer('ax12_arm' + '/follow_joint_trajectory')
    gripperServer = axGripperServer('ax12_gripper' + '/gripper_command')
    # rospy.spin()
    while not rospy.is_shutdown():
        publishJointStates()
        rospy.sleep(0.1)
        # if len(armServer.pointsQueue) == 0:
        #     armServer.publishJointStates()
        #     gripperServer.publishJointStates()
        #     rospy.sleep(0.1)


def axJointStates():
    rospy.init_node('ax12_action_server')
    while not rospy.is_shutdown():
        publishJointStates()
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        axserver()
        # axJointStates()
    except rospy.ROSInterruptException:
        pass
