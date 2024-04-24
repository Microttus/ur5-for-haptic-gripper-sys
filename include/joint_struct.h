//
// Created by Martin Økter on 23/04/2024.
//

#ifndef UR5_FOR_HAPTIC_GRIPPER_SYS_INCLUDE_UR5_ARM_CONTROL_JOINT_STRUCT_H_
#define UR5_FOR_HAPTIC_GRIPPER_SYS_INCLUDE_UR5_ARM_CONTROL_JOINT_STRUCT_H_

struct cart_point{
  double x = 0;
  double y = 0;
  double z = 0;
  double rx = 0;
  double ry = 0;
  double rz = 0;
};

struct joint_6dof{
  double phi_1 = 0;
  double phi_2 = 0;
  double phi_3 = 0;
  double phi_4 = 0;
  double phi_5 = 0;
  double phi_6 = 0;
};

struct xyz_values{
  double x = 0;
  double y = 0;
  double z = 0;
};

#endif //UR5_FOR_HAPTIC_GRIPPER_SYS_INCLUDE_UR5_ARM_CONTROL_JOINT_STRUCT_H_
