import roslib
import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from brics_actuator.msg import Poison
from moveit_commander import MoveGroupCommander

# gripper ranges from 0.0 (closed) to 0.0115 (open)
homeAngles = [0.05,0.05,-0.05,0.05,0.05]
armStarboardAppx = [1.2,0.9,-3.9,1.5,3.0]
armStarboardUp = [1.4,0.7,-3.8,1.4,3.0]
armStarboardDown = [1.4,0.55,-3.7,1.3,3.0]
armPortAppx = [4.1,1.1,-4.0,1.6,3.0]
armPortUp = [4.3,0.9,-4.0,1.4,3.0]
armPortDown = [4.3,0.6,-3.9,1.4,3.0]
armUp = [3.0,1.1,-2.5,1.6,3.0]
armPassthrough = [3.0,0.5,-0.8,0.5,3.0]

gripOpen = 0.0115
gripClosed = 0.0082

group = MoveGroupCommander("yb_arm")
rospy.init_node('srt_demo')
jppub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)


def moveToAngles(moveGroup, anglesList):
    g = {'arm_joint_1': anglesList[0]}
    g['arm_joint_2'] = anglesList[1]
    g['arm_joint_3'] = anglesList[2]
    g['arm_joint_4'] = anglesList[3]
    g['arm_joint_5'] = anglesList[4]
    moveGroup.set_joint_value_target(g)
    moveGroup.go()


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
    moveGripper(jppub, 0.0115)
    rospy.sleep(0.3)
    moveGripper(jppub, gripOpen)
    moveToAngles(group, armUp)
    rospy.sleep(1.0)
    moveToAngles(group, armPortAppx)
    moveToAngles(group, armPortUp)
    moveGripper(jppub, gripOpen)
    rospy.sleep(0.5)
    moveToAngles(group, armPortDown)
    rospy.sleep(0.5)
    moveGripper(jppub, gripClosed)
    rospy.sleep(0.5)
    moveToAngles(group, armPortUp)
    rospy.sleep(0.5)
    moveToAngles(group, armPassthrough)
    moveToAngles(group, armStarboardAppx)
    moveToAngles(group, armStarboardUp)
    rospy.sleep(0.5)
    moveToAngles(group, armStarboardDown)
    rospy.sleep(0.5)
    moveGripper(jppub, gripOpen)
    rospy.sleep(1.0)
    moveToAngles(group, armStarboardUp)
    rospy.sleep(0.5)
    moveToAngles(group, homeAngles)
    moveGripper(jppub, 0.001)
