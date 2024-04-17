//
// Created by biomech on 01.03.24.
//

#ifndef UR5_ARM_CONTROL_SRC_HAND_TO_ARM_LOGIC_H_
#define UR5_ARM_CONTROL_SRC_HAND_TO_ARM_LOGIC_H_

# include <math.h>
# include <vector>

#include <ur_rtde/rtde_control_interface.h>

#include "../src/joint_struct.cc"


class hand_to_arm_logic {
 public:
  hand_to_arm_logic();
  ~hand_to_arm_logic();

  joint_6dof ur5_ik_model_1(double px, double py, double pz);

  void rtde_set_pose(cart_point pose_q);
  std::vector<double> acc_to_rot(double ax, double ay, double az, double offset_roll = 0, double offset_pitch = 0);

 private:
  ur_rtde::RTDEControlInterface rtde_control;
  double ur_speed;
  double ur_acceleration;


};

#endif //UR5_ARM_CONTROL_SRC_HAND_TO_ARM_LOGIC_H_
