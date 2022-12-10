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

using namespace std::chrono_literals;

using NavToPoseAction = nav2_msgs::action::NavigateToPose; 
using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPoseAction>;

enum class ClientState;

class NavActionClient : public rclcpp::Node
{
public:
    NavActionClient();
    bool initialize();
    bool sendGoal(geometry_msgs::msg::PoseStamped & waypoint);
    bool getResult();

private:
    void goalResponseCallback(std::shared_future<GoalHandleNavToPose::SharedPtr> future);
    void feedbackCallback(
        GoalHandleNavToPose::SharedPtr,
        const std::shared_ptr<const NavToPoseAction::Feedback> feedback);
    void resultCallback(const GoalHandleNavToPose::WrappedResult & result);

    ClientState client_state_;

    rclcpp_action::Client<NavToPoseAction>::SharedPtr client_ptr_;
    
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    rclcpp::Duration navigation_time_{0s};
    rclcpp::Duration NAVIGATION_TIMEOUT{20s};
};
#endif // ROBOT_MANAGER__NAV_ACTION_CLIENT_HPP_
