// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com


#include <vector>
#include <string>
#include <memory>
#include <queue>
#include <utility>

#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"

#include "robot_manager/robot_manager.hpp"

enum class RobotManagerState {INIT, IDLE, ON_ROUTINE, FINISH};
enum class WaypointState {DEFAULT, FAIL, COMPLETE};

using namespace std::placeholders;

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

  // Create a service for setting routine
  set_routine_service_ = this->create_service<SetRoutine>(
    "set_routine",
    std::bind(&RobotManager::setRoutineCallback, this, _1, _2));


  // Set ros parameter
  // Parameter: origin
  auto origin_param_desc =
    rcl_interfaces::msg::ParameterDescriptor{};

  origin_param_desc.description =
    "\nThis parameter is the origin of the robot spawning point "
    "relative to the map.";

  std::vector<double> origin_vec{0, 0, 0, 0, 0, 0, 0};
  this->declare_parameter("origin", origin_vec, origin_param_desc);

  origin_vec =
    this->get_parameter("origin").as_double_array();

  origin_ = geometry_msgs::msg::Pose();
  origin_.position.x = origin_vec.at(0);
  origin_.position.y = origin_vec.at(1);
  origin_.position.z = origin_vec.at(2);
  origin_.orientation.x = origin_vec.at(3);
  origin_.orientation.y = origin_vec.at(4);
  origin_.orientation.z = origin_vec.at(5);
  origin_.orientation.w = origin_vec.at(6);

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Robot origin: [x: " << origin_.position.x <<
      ", y: " << origin_.position.y <<
      ", z: " << origin_.position.z << "]");

  // Parameter: waypoints_name
  auto waypoints_name_desc =
    rcl_interfaces::msg::ParameterDescriptor{};

  waypoints_name_desc.description =
    "\nThis parameter contains the name of waypoints.";

  this->declare_parameter("waypoints_name", waypoints_name_, waypoints_name_desc);

  waypoints_name_ =
    this->get_parameter("waypoints_name").as_string_array();

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Loading waypoints ...");

  auto waypoints_desc =
    rcl_interfaces::msg::ParameterDescriptor{};


  // Parameters: waypoints
  for (int i = 0; i < static_cast<int>(waypoints_name_.size()); ++i) {
    std::string name = waypoints_name_[i];

    std::vector<double> pose_vec{0, 0, 0, 0, 0, 0, 0};

    waypoints_desc.description =
      "\nThis parameter is the pose of waypoint " + name + ".";

    this->declare_parameter("waypoints." + name, pose_vec, waypoints_desc);
    pose_vec =
      this->get_parameter("waypoints." + name).as_double_array();

    auto waypoint_pose = geometry_msgs::msg::Pose();
    waypoint_pose.position.x = pose_vec.at(0);
    waypoint_pose.position.y = pose_vec.at(1);
    waypoint_pose.position.z = pose_vec.at(2);
    waypoint_pose.orientation.x = pose_vec.at(3);
    waypoint_pose.orientation.y = pose_vec.at(4);
    waypoint_pose.orientation.z = pose_vec.at(5);
    waypoint_pose.orientation.w = pose_vec.at(6);

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Waypoint " << name << ": [" <<
        "x: " << waypoint_pose.position.x <<
        ", y: " << waypoint_pose.position.y <<
        ", z: " << waypoint_pose.position.z << "]");

    // Add waypoint to dictionary
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.pose = waypoint_pose;
    waypoint_name_to_pose_[name] = pose_stamped;
    waypoint_name_to_index_[name] = i;
    waypoints_state_[name] = WaypointState::DEFAULT;
  }

  control_cycle_timer_ =
    this->create_wall_timer(
    100ms,
    std::bind(&RobotManager::controlCycleCallback, this)
    );

  robot_manager_state_ = RobotManagerState::IDLE;
}

void RobotManager::controlCycleCallback()
{
  if (nav_to_pose_client_ == nullptr) {
    nav_to_pose_client_ = std::make_shared<NavActionClient>(this->shared_from_this());
  }

  switch (robot_manager_state_) {
    case RobotManagerState::INIT:
      this->initialize();
      return;

    case RobotManagerState::IDLE:
      RCLCPP_INFO_STREAM_ONCE(
        this->get_logger(),
        "Waiting for routine ...");
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

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Finished waypoint: " <<
        "[x: " << current_waypoint_pose.pose.position.x <<
        ", y: " << current_waypoint_pose.pose.position.y << "]");

    // Todo: Analyze result
    waypoints_state_[current_waypoint_] = WaypointState::COMPLETE;

    // Finish routine
    if (this->current_queue_.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
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
  for (auto & [_, state] : waypoints_state_) {
    state = WaypointState::DEFAULT;
  }

  this->routine_by_name_.clear();
  this->routine_by_pose_.clear();
  this->current_waypoint_.clear();

  std::queue<std::string> empty_queue;
  std::swap(current_queue_, empty_queue);
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

  for (auto & [name, state] : waypoints_state_) {
    marker.id = waypoint_name_to_index_[name];
    marker.pose = waypoint_name_to_pose_[name].pose;

    switch (state) {
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

void RobotManager::setRoutineCallback(
  const std::shared_ptr<SetRoutine::Request> request,
  std::shared_ptr<SetRoutine::Response> response)
{
  bool on_task = this->nav_to_pose_client_->onTask();
  if (on_task) {
    response->success = false;
    response->msg = "Robot is executing previous routine";
    return;
  }

  this->reset();
  routine_by_name_ = request->routine;

  if (routine_by_name_.empty()) {
    response->success = false;
    response->msg = "Routine is empty";
    return;
  }

  std::string routine_sequence = "O ";

  // Create routine
  for (auto & name : routine_by_name_) {
    routine_sequence += "-> ";
    routine_sequence += name;
    if (waypoint_name_to_pose_.find(name) ==
      waypoint_name_to_pose_.end())
    {
      response->success = false;
      response->msg = "Waypoint " + name + " is not set";
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Rceived invalid routine. " <<
          "Waypoint " << name << " is not set");
      return;
    }
    routine_by_pose_.push_back(waypoint_name_to_pose_[name]);
    current_queue_.push(name);
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Recevied routine " << routine_sequence);

  robot_manager_state_ = RobotManagerState::ON_ROUTINE;
  response->success = true;
  response->msg = "Receved routine, ready to execute";
}
