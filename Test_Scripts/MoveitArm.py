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
  moveArm(initPos[:])

def moveArm(initPos):
  move_group = MoveGroupInterface('arm', 'base_link')
  positions = [initPos]
  for i in range(len(joint_names)):
    newpos = positions[-1][:]
    newpos[i] = 0.0
    positions.append(newpos)
  for pos in positions:
    move_group.moveToJointPosition(joint_names, pos, wait=False)
    move_group.get_move_action().wait_for_result()
  move_group.get_move_action().cancel_all_goals()

if __name__ == '__main__':
  try:
    rospy.init_node('moveArm')
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    sub = rospy.Subscriber('/joint_states', JointState, initPos)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
