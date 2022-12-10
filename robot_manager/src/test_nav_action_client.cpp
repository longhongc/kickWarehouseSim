// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_manager/nav_action_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Start nav action client test");

  auto waypoint = geometry_msgs::msg::PoseStamped();
  waypoint.pose.position.x = 10;
  auto waypoint2 = geometry_msgs::msg::PoseStamped();
  waypoint2.pose.position.x = 17;
  waypoint2.pose.position.y = -17;

  auto test_client = std::make_shared<NavActionClient>();
  bool success = test_client->sendGoal(waypoint);

  auto start = rclcpp::Clock(RCL_ROS_TIME).now();
  bool check = false;
  while(rclcpp::ok() && success) {
      // if (test_client->getResult()) {
      //   RCLCPP_INFO_STREAM(
      //   rclcpp::get_logger("rclcpp"),
      //   "Test finish");
      //   break;
      // }
      auto elapsed = rclcpp::Clock(RCL_ROS_TIME).now() - start;
      if (elapsed.seconds() > 15 && check) {
        success = test_client->sendGoal(waypoint2);
        check = false;
      }
      rclcpp::spin_some(test_client);
  }

  rclcpp::shutdown();
  return 0;
}

