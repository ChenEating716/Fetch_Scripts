#!/usr/bin/python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from sensor_msgs.msg import JointState

def initPos(state):
  sub.unregister()
  initPos = [0.0]*len(joint_names)
  for joint in state.name:
    if joint in joint_names:
      stateIdx = state.name.index(joint)
      idx = joint_names.index(joint)
      initPos[idx] = state.position[stateIdx]
  moveTorso(initPos[:])

def moveTorso(initPos):
  move_group = MoveGroupInterface('arm_with_torso', 'base_link')
  positions = [initPos[:]]
  newpos = positions[-1][:]
  newpos[0] = 0.1
  positions.append(newpos)
  for pos in positions:
    print pos[0]
    move_group.moveToJointPosition(joint_names, pos, wait=False)
    move_group.get_move_action().wait_for_result()
  move_group.get_move_action().cancel_all_goals()

if __name__ == '__main__':
  try:
    rospy.init_node('moveTorso')
    joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    sub = rospy.Subscriber('/joint_states', JointState, initPos)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
