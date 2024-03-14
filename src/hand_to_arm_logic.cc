//
// Created by biomech on 01.03.24.
//

#include "ur5_arm_control/hand_to_arm_logic.h"


hand_to_arm_logic::hand_to_arm_logic()
: rtde_control("172.17.0.2", 500.0, ur_rtde::RTDEControlInterface::FLAG_USE_EXT_UR_CAP, 50002)
, ur_speed(0.5)
, ur_acceleration(1.4)
{

}

hand_to_arm_logic::~hand_to_arm_logic()
{
  rtde_control.servoStop();
  rtde_control.stopScript();
}

joint_6dof hand_to_arm_logic::ur5_ik_model_1(double px, double py, double pz)
{
  joint_6dof joint_q;

  joint_q.phi_1 = px;
  joint_q.phi_2 = pz;
  joint_q.phi_3 = pz;

  return joint_q;
}

void hand_to_arm_logic::rtde_set_pose(cart_point pose_tool)
{
  std::vector<double> pose_q = {pose_tool.x, pose_tool.y, pose_tool.z, pose_tool.rx, pose_tool.ry, pose_tool.rz};

  rtde_control.moveJ_IK(pose_q, ur_speed, ur_acceleration, false);

}