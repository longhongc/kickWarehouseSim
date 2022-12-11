// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com


#include "robot_manager/robot_manager.hpp"

enum class RobotManagerState{INIT, IDLE, ON_ROUTINE, FINISH};
enum class WaypointState{IN_QUEUE, FAIL, COMPLETE};

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
    this->create_publisher<visualization_msgs::msg::Marker>(
        "/waypoints_marker", 
        10);

  waypoints_marker_timer_ =
    this->create_wall_timer(
    100ms,
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
  waypoint_dict_["A"] = waypoint_A;
  waypoint_dict_["B"] = waypoint_B;
  waypoint_dict_["C"] = waypoint_C;

  // Create fake routine
  std::vector<std::string> routine{"A", "B", "C"};

  // Create routine
  for (auto & id: routine) {
   routine_pose_.push_back(waypoint_dict_[id]);
   current_routine_queue_.push(waypoint_dict_[id]);
   waypoints_state_.push_back(WaypointState::IN_QUEUE);
  }

  control_cycle_timer_ =
    this->create_wall_timer(
    100ms,
    std::bind(&RobotManager::controlCycleCallback, this)
    );

  // nav_to_pose_client_ = std::make_shared<NavActionClient>();
  // action_client_executor_.add_node(nav_to_pose_client_);
  // action_client_executor_.spin();
  robot_manager_state_ = RobotManagerState::ON_ROUTINE;
  // rclcpp::spin(nav_to_pose_client_);
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
      // rclcpp::spin_some(nav_to_pose_client_);
      break;

    case RobotManagerState::FINISH:
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
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Finished waypoint: " <<
      "[x: " << this->current_waypoint_.pose.position.x <<
      ", y: " << this->current_waypoint_.pose.position.y << "]");

    // Finish routine
    if (this->current_routine_queue_.empty()) {
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
  auto next_waypoint = current_routine_queue_.front();
  bool send_goal_success = 
    nav_to_pose_client_->sendGoal(next_waypoint);

  if (send_goal_success) {
    this->current_waypoint_ = next_waypoint;
    this->current_routine_queue_.pop();
  }
  
  return send_goal_success;
}

void RobotManager::waypointsMarkerCallback()
{
  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "robot_manager"; 
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.g = 1.0;

  waypoints_marker_pub_->publish(marker);
}
