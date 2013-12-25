from moveit_commander import MoveGroupCommander

# gripper ranges from 0.0 (closed) to 0.0115 (open)
homeAngles = [0.05,0.05,-0.05,0.05,0.05]
armStarboardUp = [1.4,0.7,-3.8,1.4,3.0]
armStarboardDown = [1.4,0.55,-3.7,1.3,3.0]
armPortUp = [4.3,0.9,-4.0,1.4,3.0]
armPortDown = [4.3,0.6,-3.9,1.4,3.0]
armUp = [3.0,1.1,-2.5,1.6,3.0]

group = MoveGroupCommander("yb_arm")


def moveToAngles(moveGroup, anglesList):
    g = {'arm_joint_1': anglesList[0]}
    g['arm_joint_2'] = anglesList[1]
    g['arm_joint_3'] = anglesList[2]
    g['arm_joint_4'] = anglesList[3]
    g['arm_joint_5'] = anglesList[4]
    moveGroup.set_joint_value_target(g)
    moveGroup.go()


if __name__ == '__main__':
    moveToAngles(group, homeAngles)
