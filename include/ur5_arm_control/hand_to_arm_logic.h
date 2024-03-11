//
// Created by biomech on 01.03.24.
//

#ifndef UR5_ARM_CONTROL_SRC_HAND_TO_ARM_LOGIC_H_
#define UR5_ARM_CONTROL_SRC_HAND_TO_ARM_LOGIC_H_

# include "math.h"

#include "../src/joint_struct.cc"

struct joint_6dof{
  float phi_1 = 0;
  float phi_2 = 0;
  float phi_3 = 0;
  float phi_4 = 0;
  float phi_5 = 0;
  float phi_6 = 0;
};

class hand_to_arm_logic {
 public:
  hand_to_arm_logic() = default;

  joint_6dof ur5_ik_model_1(float px, float py, float pz);

};

#endif //UR5_ARM_CONTROL_SRC_HAND_TO_ARM_LOGIC_H_
