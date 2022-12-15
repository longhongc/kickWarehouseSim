// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// @file main.cpp
// @brief Main function of robot manager
// @author: Chang-Hong Chen
// @email: longhongc@gmail.com

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot_manager/robot_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Start robot manager node");

  auto robot_manager = std::make_shared<RobotManager>();
  rclcpp::spin(robot_manager);
  rclcpp::shutdown();

  return 0;
}
