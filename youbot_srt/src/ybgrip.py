import roslib
import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from brics_actuator.msg import Poison
import sys


rospy.init_node('srt_demo')
jppub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)


def moveGripper(gPublisher, floatVal):
    jp = JointPositions()
    myPoison = Poison()
    myPoison.originator = 'srt_demo'
    myPoison.description = 'whoknows'
    myPoison.qos = 0.0
    jp.poisonStamp = myPoison
    nowTime = rospy.Time.now()
    jvl = JointValue()
    jvl.timeStamp = nowTime
    jvl.joint_uri = 'gripper_finger_joint_l'
    jvl.unit = 'm'
    jvl.value = floatVal
    jp.positions.append(jvl)
    jvr = JointValue()
    jvr.timeStamp = nowTime
    jvr.joint_uri = 'gripper_finger_joint_r'
    jvr.unit = 'm'
    jvr.value = floatVal
    jp.positions.append(jvr)
    gPublisher.publish(jp)


if __name__ == '__main__':
    gripAngle = float(sys.argv[1])
    moveGripper(jppub, gripAngle)
    rospy.sleep(0.3)
    moveGripper(jppub, gripAngle)
