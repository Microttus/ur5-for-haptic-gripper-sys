//
// Created by Oktma on 01.03.24.
//
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "../include/ur5_arm_control/hand_to_arm_logic.h"

using namespace std::chrono_literals;

class HandToArmLogicInterface : public rclcpp::Node
{
 public:
  HandToArmLogicInterface()
  : Node("hand_to_arm_bridge")
  {
    ur5_arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);    // Finger force publisher
    hand_pos_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/icearm_input", 10, std::bind(&HandToArmLogicInterface::updtae_hand_pos, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(5s, std::bind(&HandToArmLogicInterface::timer_callback, this));
    //scaled_joint_trajectory_controller/joint_trajectory
    std::signal(SIGINT, &HandToArmLogicInterface::onShutdown);

    RCLCPP_INFO(this->get_logger(), "Setup completed");
  }

 private:
  void timer_callback(){
    publish_tcp_pos();
  }

  void publish_tcp_pos(){
    auto ur5_msg = trajectory_msgs::msg::JointTrajectory();

    ur5_msg.header.stamp = this->now();
    ur5_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint",
                                         "elbow_joint", "wrist_1_joint",
                                         "wrist_2_joint", "wrist_3_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.785, -1.57, 0.0, 0.0, 0.0, 0.785};
    point.velocities = {};  // Add velocities if needed
    point.accelerations = {};  // Add accelerations if needed
    point.effort = {};  // Add effort values if needed
    point.time_from_start = rclcpp::Duration(4, 0);

    ur5_msg.points.push_back(point);

    ur5_arm_pub_->publish(ur5_msg);

    RCLCPP_INFO(this->get_logger(), "New point sent");
  }

  void updtae_hand_pos(geometry_msgs::msg::Twist::SharedPtr msg){
    // Update position
    double posX = msg->linear.x;
    double posY = msg->linear.y;
    double posZ = msg->linear.z;

    //RobotArm.set_goal_point(posX, posY, posZ);

    RCLCPP_INFO(this->get_logger(), "New point received");
  }

  static void onShutdown(int signum) {
    if (signum == SIGINT) {
      // Perform cleanup operations before shutting down
      RCLCPP_INFO(rclcpp::get_logger("hand_to_arm_bridge"), "Gracefully shutdown initiated ...");
      // Add your cleanup logic here

      // Call the ROS 2 shutdown function
      rclcpp::shutdown();
    }
  }

  // ROS2 and publishers declarations
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ur5_arm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr hand_pos_sub_;
};



std::atomic<bool> shutdown_requested(false);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandToArmLogicInterface>();

  // Create a SingleThreadedExecutor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Add your node to the executor
  executor.add_node(node);

  // Run the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}