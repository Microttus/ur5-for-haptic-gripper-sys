//
// Created by biomech on 14.03.24.
//

#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;


int main_next(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("172.17.0.2", 500.0, RTDEControlInterface::FLAG_USE_EXT_UR_CAP, 5900);

  //std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
  std::vector<double> pose_q = {0.5, -0.2, 0.2, -3.14, 0.0, 0.0};

  //rtde_control.moveJ(joint_q, 1.0, 1.4, false);
  rtde_control.moveJ_IK(pose_q, 0.5, 1.4, false);

  rtde_control.servoStop();
  rtde_control.stopScript();

  return 0;
}