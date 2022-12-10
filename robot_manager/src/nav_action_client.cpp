// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#include <functional>

#include "robot_manager/nav_action_client.hpp"

using namespace std::placeholders;

enum class ClientState {UNSET, READY, ON_TASK, PREEMPT, FINISH};

NavActionClient::NavActionClient()
: Node("nav_action_client"),
  client_state_{ClientState::UNSET}
{
  this->initialize();
}

bool NavActionClient::initialize()
{
  if (this->client_state_ == ClientState::ON_TASK) {
    RCLCPP_ERROR(this->get_logger(),
    "Action client is on previous goal");
    return false;
  }

  this->client_ptr_ =
    rclcpp_action::create_client<NavToPoseAction>(
    this,
    "/navigate_to_pose");
  this->client_state_ = ClientState::READY;
  return true;
}

bool NavActionClient::sendGoal(geometry_msgs::msg::PoseStamped & waypoint)
{
  switch(this->client_state_) {
      case ClientState::UNSET:
        RCLCPP_ERROR(this->get_logger(),
        "Action client has not initialized");
        return false;

      case ClientState::READY:
      case ClientState::FINISH:
        RCLCPP_INFO(this->get_logger(),
        "Action client is ready to send goal");
        break;

      case ClientState::ON_TASK:
        RCLCPP_WARN(this->get_logger(),
        "New goal received. Action client preempt");
        this->navigation_time_ = rclcpp::Duration(0);
        this->client_state_ = ClientState::PREEMPT;
        break;

      default:
        RCLCPP_ERROR(this->get_logger(),
        "Unknown client state");
        return false;
  }

  if (!this->client_ptr_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Action server not available after waiting 3 seconds");
    return false;
  }

  this->goal_pose_ = waypoint;
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = waypoint;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options =
      rclcpp_action::Client<NavToPoseAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      std::bind(&NavActionClient::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavActionClient::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavActionClient::resultCallback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  this->client_state_ = ClientState::ON_TASK;
  return true;
}

bool NavActionClient::getResult()
{
  switch(this->client_state_) {
    case ClientState::UNSET:
      RCLCPP_ERROR(this->get_logger(),
      "Goal timeout");
      return true;

    case ClientState::FINISH:
      RCLCPP_INFO(this->get_logger(),
      "Action client is ready to send goal");
      return true;

    // case ClientState::ON_TASK:
    //   RCLCPP_ERROR(this->get_logger(),
    //   "Action client is on previous goal");
    //   return false;

    default:
      RCLCPP_ERROR(this->get_logger(),
      "No result yet");
  }

  return false;
}

void NavActionClient::goalResponseCallback(
    std::shared_future<GoalHandleNavToPose::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result"); 
  }
}

void NavActionClient::feedbackCallback(
    GoalHandleNavToPose::SharedPtr,
    const std::shared_ptr<const NavToPoseAction::Feedback> feedback)
{
    this->current_pose_ = feedback->current_pose;
    this->navigation_time_ = feedback->navigation_time;

    // Todo: add a print func for pose
    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(),
        *(this->get_clock()),
        1000,
        "Goal pose: [x: " << this->goal_pose_.pose.position.x <<
        ", y: " << this->goal_pose_.pose.position.y << "]");

    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(),
        *(this->get_clock()),
        1000,
        "Current pose: [x: " << this->current_pose_.pose.position.x <<
        ", y: " << this->current_pose_.pose.position.y << "]");

    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(),
        *(this->get_clock()),
        1000,
        "Navigation time: " << this->navigation_time_.seconds());

    if (this->navigation_time_ > this->NAVIGATION_TIMEOUT) {
        RCLCPP_ERROR(this->get_logger(),
          "Goal timeout");
        this->client_ptr_->async_cancel_goals_before(this->now());
        this->client_state_ = ClientState::UNSET;

    }
}

void NavActionClient::resultCallback(const GoalHandleNavToPose::WrappedResult & result)
{
  if (this->client_state_ != ClientState::PREEMPT) {
    this->client_state_ = ClientState::FINISH;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Goal was aborted");
        return; 
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
  }
}
