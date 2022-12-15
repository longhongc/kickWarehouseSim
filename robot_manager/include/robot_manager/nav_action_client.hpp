// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#ifndef ROBOT_MANAGER__NAV_ACTION_CLIENT_HPP_
#define ROBOT_MANAGER__NAV_ACTION_CLIENT_HPP_

#include <memory>
#include <thread>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;

using NavToPoseAction = nav2_msgs::action::NavigateToPose;
using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPoseAction>;

/**
 * @Brief  The internal state of the NavActionClient class
 */
enum class ClientState;

/**
 * @Brief  A client interface for sending action to Nav2
 */
class NavActionClient
{
public:
  /**
   * @Brief  Constructor
   *
   * @Param node  This class is a subclass in robot manager.
   *              It requires a shared pointer of the robot manager
   *              class to use some of the functions.
   */
  explicit NavActionClient(std::shared_ptr<rclcpp::Node> node);

  /**
   * @Brief  Destructor
   */
  ~NavActionClient();

  /**
   * @Brief  Initialization of the NavACtionClient class
   *
   * @Returns
   */
  bool initialize();

  /**
   * @Brief  Reset the action client state
   *         Prepare for a new round of action request
   */
  void reset();

  /**
   * @Brief  Send action request
   *
   * @Param waypoint The target pose to move to
   *
   * @Returns  True if send goal success
   */
  bool sendGoal(geometry_msgs::msg::PoseStamped & waypoint);

  /**
   * @Brief  Check if NavActionClient is executing a waypoint request
   *
   * @Returns  True if NavActionClient is executing the previous waypoint
   *           command
   */
  bool onTask();


  /**
   * @Brief  Check if NavActionClient has finished the previous command
   *
   * @Returns  True if NavActionClient has finished the prevous command
   */
  bool getResult();

private:
  /**
   * @Brief Callback function for goal response from action server
   *
   * @Param future Goal handler for action client
   */
  void goalResponseCallback(std::shared_future<GoalHandleNavToPose::SharedPtr> future);

  /**
   * @Brief Callback function for feedback from action server
   *
   * @Param GoalHandleNavToPose::SharedPtr
   * @Param feedback Contains information of
   *        robot pose, execution time, and numbers of recoveries
   */
  void feedbackCallback(
    GoalHandleNavToPose::SharedPtr,
    const std::shared_ptr<const NavToPoseAction::Feedback> feedback);

  /**
   * @Brief  Callback function for results from action server
   *
   * @Param result Whether the goal has been successfully executed
   */
  void resultCallback(const GoalHandleNavToPose::WrappedResult & result);

  /**
   * @Brief  Shared pointer of the node for sub class argument
   */
  std::shared_ptr<rclcpp::Node> node_;

  /**
   * @Brief  The current execution state
   */
  ClientState client_state_;

  /**
   * @Brief  navigate_to_pose action client
   */
  rclcpp_action::Client<NavToPoseAction>::SharedPtr client_ptr_;

  /**
   * @Brief  Current robot pose
   */
  geometry_msgs::msg::PoseStamped current_pose_;

  /**
   * @Brief  Current waypoint pose
   */
  geometry_msgs::msg::PoseStamped goal_pose_;

  /**
   * @Brief  Navigation time for current waypoint
   */
  rclcpp::Duration navigation_time_{0s};

  // Todo: pass in navigation timeout
  /**
   * @Brief  Timeout for robot failure on waypoint
   */
  rclcpp::Duration NAVIGATION_TIMEOUT{300s};
};
#endif  // ROBOT_MANAGER__NAV_ACTION_CLIENT_HPP_
