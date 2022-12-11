// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com


#include "robot_manager/robot_manager.hpp"

enum class RobotManagerState{INIT, IDLE, ON_ROUTINE, FINISH};
enum class WaypointState{DEFAULT, FAIL, COMPLETE};

RobotManager::RobotManager()
: Node("robot_manager"), 
  robot_manager_state_{RobotManagerState::INIT}
{
  this->initialize();
}

void RobotManager::initialize()
{
  nav_to_pose_client_ = nullptr;
  waypoints_marker_pub_ = 
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "waypoints_marker", 
        10);

  waypoints_marker_timer_ =
    this->create_wall_timer(
    200ms,
    std::bind(&RobotManager::waypointsMarkerCallback, this)
    );

  // Temporary waypoints and routine
  auto waypoint_A = geometry_msgs::msg::PoseStamped();
  waypoint_A.pose.position.x = 10;

  auto waypoint_B = geometry_msgs::msg::PoseStamped();
  waypoint_B.pose.position.x = 17;
  waypoint_B.pose.position.y = -17;

  auto waypoint_C = geometry_msgs::msg::PoseStamped();
  waypoint_C.pose.position.y = -5;

  // Create waypoint dictionary
  waypoint_name_to_pose_["A"] = waypoint_A;
  waypoint_name_to_pose_["B"] = waypoint_B;
  waypoint_name_to_pose_["C"] = waypoint_C;

  waypoint_name_to_index_["A"] = 0;
  waypoint_name_to_index_["B"] = 1;
  waypoint_name_to_index_["C"] = 2;

  // Create fake routine
  std::vector<std::string> routine{"A", "B", "C"};

  // Create routine
  for (auto & name: routine) {
   routine_by_name_.push_back(name);
   routine_by_pose_.push_back(waypoint_name_to_pose_[name]);
   waypoints_state_.push_back(WaypointState::DEFAULT);
   current_queue_.push(name);
  }

  control_cycle_timer_ =
    this->create_wall_timer(
    100ms,
    std::bind(&RobotManager::controlCycleCallback, this)
    );

  robot_manager_state_ = RobotManagerState::ON_ROUTINE;
}

void RobotManager::controlCycleCallback()
{
  if (nav_to_pose_client_ == nullptr) {
    nav_to_pose_client_ = std::make_shared<NavActionClient>(this->shared_from_this());
  }

  switch(robot_manager_state_) {
    case RobotManagerState::INIT:
      this->initialize();
      return;

    case RobotManagerState::IDLE:
      break;

    case RobotManagerState::ON_ROUTINE:
      this->runRoutine();
      break;

    case RobotManagerState::FINISH:
      this->reset();
      break;

    default:
      break;
  }
}

void RobotManager::runRoutine()
{
  bool on_task = this->nav_to_pose_client_->onTask();
  if (on_task) {
    return;
  }

  bool have_result = this->nav_to_pose_client_->getResult();

  if (have_result) {
    auto current_waypoint_pose = waypoint_name_to_pose_[current_waypoint_];
    auto current_waypoint_index = waypoint_name_to_index_[current_waypoint_];

    RCLCPP_INFO_STREAM(this->get_logger(),
      "Finished waypoint: " <<
      "[x: " << current_waypoint_pose.pose.position.x <<
      ", y: " << current_waypoint_pose.pose.position.y << "]");

    // Todo: Analyze result
    waypoints_state_[current_waypoint_index] = WaypointState::COMPLETE;

    // Finish routine
    if (this->current_queue_.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "Finished routine");

      robot_manager_state_ = RobotManagerState::FINISH;
      return;
    }
  }

  this->runWaypoint();
}

bool RobotManager::runWaypoint()
{
  // Reset action client for next waypoint
  this->nav_to_pose_client_->reset();
  auto next_waypoint = current_queue_.front();
  auto next_waypoint_pose = waypoint_name_to_pose_[next_waypoint];
  bool send_goal_success = 
    nav_to_pose_client_->sendGoal(next_waypoint_pose);

  if (send_goal_success) {
    this->current_waypoint_ = next_waypoint;
    this->current_queue_.pop();
  }
  
  return send_goal_success;
}

void RobotManager::reset()
{
  for (int i=0; i < int(waypoints_state_.size()); ++i) {
    waypoints_state[i] = WaypointState::DEFAULT;
  }
}

void RobotManager::waypointsMarkerCallback()
{
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "robot_manager"; 
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 0.7;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 0.05;

  auto marker_array = visualization_msgs::msg::MarkerArray();

  for (int i=0; i < int(waypoints_state_.size()); ++i) {
    marker.id = i; 
    marker.pose = routine_by_pose_[i].pose;

    switch(waypoints_state_[i]) {
      case WaypointState::DEFAULT:
        marker.color.r = 1.5;
        marker.color.g = 1.0;
        break;
      case WaypointState::FAIL:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        break;
      case WaypointState::COMPLETE:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        break;
    }
    marker_array.markers.push_back(marker);
  }

  waypoints_marker_pub_->publish(marker_array);
}
