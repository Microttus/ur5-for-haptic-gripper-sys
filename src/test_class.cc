//
// Created by Martin Økter on 11/03/2024.
//

#include <cmath>
#include <iostream>

struct joint_6dof{
  float phi_1;
  float phi_2;
  float phi_3;
  float phi_4;
  float phi_5;
  float phi_6;
};

class test_ik {
 public:
  test_ik()
  {

  };

  joint_6dof ur5_ik_model_1(float px, float py, float pz)
  {
    joint_6dof joint_angles;

    joint_angles.phi_1 = atan2(py-(d_i[6]*r_i[2][3]), px-(d_i[6]*r_i[1][3])) + M_PI/2 - acos(d_i[4]/sqrt(pow(py-(d_i[6]*r_i[2][3]),2) + pow(px - (d_i[6]*r_i[1][3]),2)));

    joint_angles.phi_5 = acos(((px*sin(joint_angles.phi_1)-py* cos(joint_angles.phi_1)-d_i[4]))/d_i[6]);

    joint_angles.phi_6 = atan2((cos(joint_angles.phi_1)*r_i[2][2] - sin(joint_angles.phi_1)*r_i[1][2])/ sin(joint_angles.phi_5), (sin(joint_angles.phi_1)*r_i[1][1]-cos(joint_angles.phi_1)*r_i[2][1])/ sin(joint_angles.phi_5));

    float A_3_1 = d_i[5]*sin(joint_angles.phi_6)*(cos(joint_angles.phi_1)*r_i[1][1] + sin(joint_angles.phi_1)*r_i[2][1]);
    float A_3_2 = d_i[5]*cos(joint_angles.phi_5)*(cos(joint_angles.phi_1)*r_i[1][2] + sin(joint_angles.phi_1)*r_i[2][2]);
    float A_3_3 = d_i[6]*(cos(joint_angles.phi_1)*r_i[1][3] + sin(joint_angles.phi_1)*r_i[2][3]);
    float A_3 = (A_3_1 + A_3_2 - A_3_3 + px*cos(joint_angles.phi_1) + py*sin(joint_angles.phi_1));

    float B_3 = d_i[5]*(sin(joint_angles.phi_6)*r_i[3][1]+ cos(joint_angles.phi_6)*r_i[3][2]) - d_i[6]*r_i[3][3] + pz -d_i[1];

    float divider = 2*a_i[2]*a_i[3];

    if (divider != 0) {
      joint_angles.phi_3 = -acos((pow(A_3,2) + pow(B_3,2) - pow(a_i[2],2) - pow(a_i[3],2))/(divider));
    } else {
      joint_angles.phi_3 = acos(0);
    }

    joint_angles.phi_2 = asin((-1* sin(joint_angles.phi_3)*a_i[3])/(sqrt(pow(A_3,2)+ pow(B_3,2)))) + atan2(B_3, A_3);

    float A_4 = cos(joint_angles.phi_1)*r_i[1][1] + sin(joint_angles.phi_1)*r_i[2][1];
    float B_4 = cos(joint_angles.phi_1)*r_i[1][2] + sin(joint_angles.phi_1)*r_i[2][2];
    float C_4 = cos(joint_angles.phi_1)*r_i[1][3] + sin(joint_angles.phi_1)*r_i[2][3];
    float D_4 = cos(joint_angles.phi_6)*r_i[3][1] + sin(joint_angles.phi_6)*r_i[3][2];

    float c_4 = cos(joint_angles.phi_2 + joint_angles.phi_3)*(cos(joint_angles.phi_5)* cos(joint_angles.phi_6)*A_4 - cos(joint_angles.phi_5)* sin(joint_angles.phi_6)*B_4 - sin(joint_angles.phi_5)*C_4) + sin(joint_angles.phi_2 + joint_angles.phi_3)*(cos(joint_angles.phi_5)*D_4 - sin(joint_angles.phi_5)*r_i[3][3]);
    float s_4 = sin(joint_angles.phi_2 + joint_angles.phi_3)*(-1* cos(joint_angles.phi_5)* cos(joint_angles.phi_6)*A_4 + cos(joint_angles.phi_5)* sin(joint_angles.phi_6)*B_4 + sin(joint_angles.phi_5)*C_4) + cos(joint_angles.phi_2 + joint_angles.phi_3)*(cos(joint_angles.phi_5)*D_4 - sin(joint_angles.phi_5)*r_i[3][3]);

    joint_angles.phi_4 = atan2(s_4, c_4);

    return joint_angles;
  }

  joint_6dof ur5_ik_model_3(float px, float py, float pz)
  {
    joint_6dof joint_angles;

    float A_1 = px - d_i[6]*r_i[1][3];
    float B_1 = d_i[6]*r_i[2][3] - py;

    joint_angles.phi_1 = atan2(A_1, B_1) + atan2(sqrt(pow(A_1,2) + pow(B_1,2) - pow(d_i[4],3)), d_i[4]);

    float c_5 = sin(joint_angles.phi_1)*r_i[1][3] - cos(joint_angles.phi_1)*r_i[2][3];
    float s_5 = sqrt(pow(sin(joint_angles.phi_1)*r_i[1][1] - cos(joint_angles.phi_1)*r_i[2][1], 2) + pow(sin(joint_angles.phi_1)*r_i[1][2]- cos(joint_angles.phi_1)*r_i[2][2], 2));
    float sign_s2 = (s_5 > 0.0) - (s_5 < 0.0);//(x > 0) - (x < 0)

    joint_angles.phi_5 = -atan2(s_5, c_5);
    joint_angles.phi_6 = atan2((cos(joint_angles.phi_1)*r_i[2][2]- sin(joint_angles.phi_1)*r_i[1][2])/sign_s2, (sin(joint_angles.phi_1)*r_i[1][1]- cos(joint_angles.phi_1)*r_i[2][1])/sign_s2);

    float A_234 = cos(joint_angles.phi_1)*r_i[1][1] + sin(joint_angles.phi_1)*r_i[2][1];
    float phi_234 = atan2(cos(joint_angles.phi_5)* cos(joint_angles.phi_6)*r_i[3][1] - sin(joint_angles.phi_6)*A_234,
                          cos(joint_angles.phi_5)* cos(joint_angles.phi_6)*A_234 + sin(joint_angles.phi_6)*r_i[3][1]);

    float A_2 = 2*a_i[2]*(d_i[1] - pz - d_i[5]* cos(phi_234) - d_i[6]* sin(joint_angles.phi_5)* sin(phi_234));
    float B_2 = 2*a_i[2]*(d_i[5]* sin(phi_234) - d_i[6]* sin(joint_angles.phi_5)* cos(phi_234) - cos(joint_angles.phi_1)*px - sin(joint_angles.phi_1)*py);
    float C_2_1 = pow(a_i[3],2) - pow(a_i[2],2) - pow(d_i[5], 2) - (pz - d_i[1])*(2*d_i[5]*cos(phi_234) + 2*d_i[6]*sin(joint_angles.phi_5)*sin(phi_234)+pz-d_i[1]);
    float C_2_2 = (cos(joint_angles.phi_1) *px + sin(joint_angles.phi_1)*py)*(2*d_i[5]*sin(phi_234) - 2*d_i[6]* sin(joint_angles.phi_5)* cos(phi_234) - cos(joint_angles.phi_1)*px - sin(joint_angles.phi_1)*py) - (pow(d_i[6],2)* pow(sin(joint_angles.phi_5),2));
    float C_2 = C_2_1 + C_2_2;

    joint_angles.phi_2 = atan2(A_2, B_2) + atan2(sqrt(pow(A_2,2) + pow(B_2, 2) - pow(C_2, 2)), C_2);

    float phi_34 = phi_234 - joint_angles.phi_2;
    float A_3 = (-cos(joint_angles.phi_1)* sin(joint_angles.phi_2)*px - sin(joint_angles.phi_1)* sin(joint_angles.phi_2)*py + cos(joint_angles.phi_2)*pz - cos(joint_angles.phi_2)*d_i[1] + cos(phi_34)*d_i[5] + sin(phi_34)* sin(joint_angles.phi_5)*d_i[6])/(a_i[3]);
    float B_3 = (cos(joint_angles.phi_1)* cos(joint_angles.phi_2)*px + sin(joint_angles.phi_1)* cos(joint_angles.phi_2)*py + sin(joint_angles.phi_2)*pz - sin(joint_angles.phi_2)*d_i[1] - sin(phi_34)*d_i[5] + cos(phi_34)* sin(joint_angles.phi_5)*d_i[6] - a_i[2])/(a_i[3]);

    joint_angles.phi_3 = atan2(A_3, B_3);
    joint_angles.phi_4 = phi_34 - joint_angles.phi_3;

    // Correction
    //joint_angles.phi_2 = joint_angles.phi_2 + M_PI;
    if (joint_angles.phi_2 > M_PI){
      joint_angles.phi_2 = joint_angles.phi_2 - 2*M_PI;
    }


    return joint_angles;
  }

  joint_6dof ur5_ik_model_4(float px, float py, float pz)
  {
    joint_6dof joint_angles;

    float A_1 = px - d_i[6]*r_i[1][3];
    float B_1 = d_i[6]*r_i[2][3] - py;

    joint_angles.phi_1 = atan2(A_1, B_1) + atan2(sqrt(pow(A_1,2) + pow(B_1,2) - pow(d_i[4],3)), d_i[4]);

    float c_5 = sin(joint_angles.phi_1)*r_i[1][3] - cos(joint_angles.phi_1)*r_i[2][3];
    float s_5 = sqrt(pow(sin(joint_angles.phi_1)*r_i[1][1] - cos(joint_angles.phi_1)*r_i[2][1], 2) + pow(sin(joint_angles.phi_1)*r_i[1][2]- cos(joint_angles.phi_1)*r_i[2][2], 2));
    float sign_s2 = (s_5 > 0.0) - (s_5 < 0.0);//(x > 0) - (x < 0)

    joint_angles.phi_5 = atan2(s_5, c_5);
    joint_angles.phi_6 = atan2((cos(joint_angles.phi_1)*r_i[2][2] - sin(joint_angles.phi_1)*r_i[1][2])/ sin(joint_angles.phi_5), (sin(joint_angles.phi_1)*r_i[1][1]-cos(joint_angles.phi_1)*r_i[2][1])/ sin(joint_angles.phi_5));
    //joint_angles.phi_6 = atan2((cos(joint_angles.phi_1)*r_i[2][2]- sin(joint_angles.phi_1)*r_i[1][2])/sign_s2, (sin(joint_angles.phi_1)*r_i[1][1]- cos(joint_angles.phi_1)*r_i[2][1])/sign_s2);

    float A_234 = cos(joint_angles.phi_1)*r_i[1][1] + sin(joint_angles.phi_1)*r_i[2][1];
    float phi_234 = atan2(cos(joint_angles.phi_5)* cos(joint_angles.phi_6)*r_i[3][1] - sin(joint_angles.phi_6)*A_234,
                          cos(joint_angles.phi_5)* cos(joint_angles.phi_6)*A_234 + sin(joint_angles.phi_6)*r_i[3][1]);

    float K_C = cos(joint_angles.phi_1)*px + sin(joint_angles.phi_1)*py - sin(phi_234)*d_i[5] + cos(phi_234)* sin(joint_angles.phi_5)*d_i[6];
    float K_S = pz - d_i[1] + cos(phi_234)*d_i[5] + sin(phi_234)* sin(joint_angles.phi_5)*d_i[6];

    joint_angles.phi_3 = -atan2(sqrt(1- pow((pow(K_S,2) + pow(K_C,2) - pow(a_i[2],2) - pow(a_i[3],2))/(2*a_i[2]*a_i[3]),2)), (pow(K_S, 2) + pow(K_C, 2) - pow(a_i[2], 2) - pow(a_i[3], 2))/(2*a_i[2]*a_i[3]));
    joint_angles.phi_2 = atan2(K_S, K_C) - atan2(a_i[3]* sin(joint_angles.phi_3), a_i[3]* cos(joint_angles.phi_3)+a_i[2]);

    joint_angles.phi_4 = phi_234 - joint_angles.phi_2 - joint_angles.phi_3;

    return joint_angles;
  }

  float a_i[7] = {0, M_PI/2, 0.425, 0.392, M_PI/2, -M_PI/2, 0};
  float d_i[7] = {0, 0.0892, 0, 0, 0.1093, 0.09475, 0.0825};
  float r_i[4][4] = {{0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, -1}};
};


int main() {
  test_ik joint_test;

  joint_6dof ur5_test_joint = joint_test.ur5_ik_model_1(0.5, 0.0, 0.2);
  joint_6dof ur5_test_joint_3 = joint_test.ur5_ik_model_3(0.5, 0.0, 0.2);
  joint_6dof ur5_test_joint_4 = joint_test.ur5_ik_model_4(-0.5, 0.2, 0.2);

  // Fakk meg
  //https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html

  float trans_const = 180/M_PI;

  std::cout << "Joint 1 -> " << ur5_test_joint.phi_1*trans_const << " - Joint 1 -> " << ur5_test_joint_3.phi_1*trans_const << " - Joint 1 -> " << ur5_test_joint_4.phi_1*trans_const << std::endl;
  std::cout << "Joint 2 -> " << ur5_test_joint.phi_2*trans_const << " - Joint 2 -> " << ur5_test_joint_3.phi_2*trans_const << " - Joint 2 -> " << ur5_test_joint_4.phi_2*trans_const << std::endl;
  std::cout << "Joint 3 -> " << ur5_test_joint.phi_3*trans_const << " - Joint 3 -> " << ur5_test_joint_3.phi_3*trans_const << " - Joint 3 -> " << ur5_test_joint_4.phi_3*trans_const << std::endl;
  std::cout << "Joint 4 -> " << ur5_test_joint.phi_4*trans_const << " - Joint 4 -> " << ur5_test_joint_3.phi_4*trans_const << " - Joint 4 -> " << ur5_test_joint_4.phi_4*trans_const << std::endl;
  std::cout << "Joint 5 -> " << ur5_test_joint.phi_5*trans_const << " - Joint 5 -> " << ur5_test_joint_3.phi_5*trans_const << " - Joint 5 -> " << ur5_test_joint_4.phi_5*trans_const << std::endl;
  std::cout << "Joint 6 -> " << ur5_test_joint.phi_6*trans_const << " - Joint 6 -> " << ur5_test_joint_3.phi_6*trans_const << " - Joint 6 -> " << ur5_test_joint_4.phi_6*trans_const << std::endl;
};