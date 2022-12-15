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
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_manager/nav_action_client.hpp"
#include "robot_manager_msgs/srv/set_routine.hpp"

/**
 * @Brief  The internal state of RobotManager class
 */
enum class RobotManagerState;

/**
 * @Brief  The state of a waypoint
 *
 */
enum class WaypointState;

using SetRoutine = robot_manager_msgs::srv::SetRoutine;

/**
 * @Brief  A interface to control and interact with the robot
 */
class RobotManager : public rclcpp::Node
{
public:
  /**
   * @Brief  Constructor
   */
  RobotManager();

private:
  /**
   * @Brief  Initialize the RobotManager class
   */
  void initialize();

  /**
   * @Brief  Main finite state machine control loop
   */
  void controlCycleCallback();

  /**
   * @Brief Execute a routine
   */
  void runRoutine();

  /**
   * @Brief Execute a waypoint
   *
   * @Returns True if a waypoint command is sent successfully
   */
  bool runWaypoint();

  /**
   * @Brief Reset the RobotManager class for a new round of routine
   */
  void reset();

  /**
   * @Brief Callback function for timer to update marker on RVIZ
   */
  void waypointsMarkerCallback();

  /**
   * @Brief  Callback for set_routine service
   *
   * @Param request The routine string vector
   * @Param response Whether service is sent successfully
   */
  void setRoutineCallback(
    const std::shared_ptr<SetRoutine::Request> request,
    std::shared_ptr<SetRoutine::Response> response);

  /**
   * @Brief  The current execution state of RobotManager
   */
  RobotManagerState robot_manager_state_;

  /**
   * @Brief  The main control loop timer
   */
  rclcpp::TimerBase::SharedPtr control_cycle_timer_;

  /**
   * @Brief  An action client subclass to communicate with Nav2
   */
  std::shared_ptr<NavActionClient> nav_to_pose_client_;

  /**
   * @Brief  Marker publisher to publish waypoints to RVIZ
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_marker_pub_;

  /**
   * @Brief  Timer for updating marker
   */
  rclcpp::TimerBase::SharedPtr waypoints_marker_timer_;

  /**
   * @Brief  Service server of set_routine
   */
  rclcpp::Service<SetRoutine>::SharedPtr set_routine_service_;

  /**
   * @Brief  The name of all waypoints
   */
  std::vector<std::string> waypoints_name_;

  /**
   * @Brief  A dictionary that map waypoint name to its pose
   */
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoint_name_to_pose_;

  /**
   * @Brief  A dictionary that map waypoint name to its id
   */
  std::map<std::string, int> waypoint_name_to_index_;

  /**
   * @Brief  The state of every waypoints, whether it is in task queue or finished
   */
  std::map<std::string, WaypointState> waypoints_state_;

  /**
   * @Brief  The routine sequence in name
   */
  std::vector<std::string> routine_by_name_;

  /**
   * @Brief  The routine sequence in pose
   */
  std::vector<geometry_msgs::msg::PoseStamped> routine_by_pose_;

  /**
   * @Brief  The current waypoints queue that contains waypoints
   *         ready to be executed
   */
  std::queue<std::string> current_queue_;

  /**
   * @Brief  The current waypoint that is executing
   */
  std::string current_waypoint_;

  /**
   * @Brief  The origin of the robot with respect to map
   */
  geometry_msgs::msg::Pose origin_;

  /**
   * @Brief  Timeout for robot failure on waypoint
   */
  rclcpp::Duration NAVIGATION_TIMEOUT{20s};
};

#endif  // ROBOT_MANAGER__ROBOT_MANAGER_HPP_
