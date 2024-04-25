//
// Created by biomech on 01.03.24.
//

# include <math.h>
# include <vector>

#include "../include/ur5_arm_control/hand_to_arm_logic.h"

//"172.17.0.2 - 5900
// 10.42.0.251 - 50002
hand_to_arm_logic::hand_to_arm_logic()
: rtde_control("172.17.0.2", 500.0, ur_rtde::RTDEControlInterface::FLAG_USE_EXT_UR_CAP, 50002)
, ur_speed(2)
, ur_acceleration(1.4)
{
  integrator_time.push_back(std::chrono::steady_clock::now());
  integrator_time.push_back(std::chrono::steady_clock::now());
  integrator_time.push_back(std::chrono::steady_clock::now());
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
  joint_q.phi_2 = py;
  joint_q.phi_3 = pz;

  return joint_q;
}

void hand_to_arm_logic::rtde_set_pose(cart_point pose_tool)
{
  std::vector<double> pose_q = {pose_tool.x, pose_tool.y, pose_tool.z, pose_tool.rx, pose_tool.ry, pose_tool.rz};

  rtde_control.moveJ_IK(pose_q, ur_speed, ur_acceleration, false);

}

std::vector<double> hand_to_arm_logic::acc_to_rot(double ax, double ay, double az, double offset_roll, double offset_pitch) {
  double phi = 0; //roll
  double theta = 0; //pitch
  double omega = 0; //yaw

  // Calculate roll
  if (az != 0) {
    phi = -atan2(ay, az);
  }

  // Calculate pitch
  double devider = sqrt(pow(ay, 2) + pow(az, 2));
  if (devider != 0) {
    theta = -atan2(ax, devider);
  }

  // Apply offset
  phi = phi + offset_roll;
  theta = theta + offset_pitch;

  // Calcualate yaw
  omega = atan2(sin(phi), cos(phi)* cos(theta));


  std::vector<double> rotation_of_palm({phi, theta, omega});

  return rotation_of_palm;
}

double hand_to_arm_logic::complimentary_filter(double main_val, double second_val, double last_value, double alpha) {

  double main_after_last = last_value - main_val;
  double calc_val = (main_after_last * (1-alpha)) + (second_val * alpha);

  return calc_val;
}

double hand_to_arm_logic::integrator(double new_val, double last_val, int id) {
  std::chrono::steady_clock::time_point new_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(new_time - integrator_time.at(id)).count();

  integrator_time.at(id) = new_time;

  double double_time = static_cast<double>(duration) * 0.001;

  double integrated_time = last_val + (new_val * double_time);

  return integrated_time;
}