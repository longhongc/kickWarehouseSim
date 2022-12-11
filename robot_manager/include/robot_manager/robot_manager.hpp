// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#ifndef ROBOT_MANAGER__ROBOT_MANAGER_HPP_
#define ROBOT_MANAGER__ROBOT_MANAGER_HPP_

#include <memory>
#include <map>
#include <queue>
#include <vector>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_manager/nav_action_client.hpp"

enum class RobotManagerState;
enum class WaypointState;

class RobotManager : public rclcpp::Node
{
public:
  RobotManager();

private:
  void initialize();
  void controlCycleCallback();
  void runRoutine();
  bool runWaypoint();

  void waypointsMarkerCallback();

  RobotManagerState robot_manager_state_;

  rclcpp::TimerBase::SharedPtr control_cycle_timer_;
  std::shared_ptr<NavActionClient> nav_to_pose_client_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_marker_pub_;
  rclcpp::TimerBase::SharedPtr waypoints_marker_timer_;

  // Todo: readin parameters
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoint_name_to_pose_;
  std::map<std::string, int> waypoint_name_to_index_;

  std::vector<std::string> routine_by_name_;
  std::vector<geometry_msgs::msg::PoseStamped> routine_by_pose_;

  std::vector<WaypointState> waypoints_state_;
  std::queue<std::string> current_queue_;
  std::string current_waypoint_;

  rclcpp::Duration NAVIGATION_TIMEOUT{20s};
};

#endif // ROBOT_MANAGER__ROBOT_MANAGER_HPP_

