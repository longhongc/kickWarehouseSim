#include <functional>
#include <memory>
#include <thread>

// #include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;


class MinimalActionClient : public rclcpp::Node
{
public:
    using follow_waypoints_action = nav2_msgs::action::NavigateToPose; 
    // using follow_waypoints_action = nav2_msgs::action::FollowWaypoints; 
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<follow_waypoints_action>;

    explicit MinimalActionClient(const rclcpp::NodeOptions & options)
    : Node("minimal_action_client", options)
    {
        this->client_ptr_ = 
            rclcpp_action::create_client<follow_waypoints_action>(
                this,
                "/navigate_to_pose");

        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&MinimalActionClient::send_goal, this));
    }

    void send_goal()
    {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(),
                    "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto waypoints_1 = geometry_msgs::msg::PoseStamped();
        waypoints_1.pose.position.x = 10;
        // auto waypoints_2 = geometry_msgs::msg::PoseStamped();
        // waypoints_2.pose.position.x = 17;
        // waypoints_2.pose.position.y = -17;
        // auto waypoints_3 = geometry_msgs::msg::PoseStamped();
        // waypoints_3.pose.position.x = 10.5;
        // waypoints_3.pose.position.y = -7.5;
        // auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
        // goal_msg.poses.push_back(waypoints_1);
        // goal_msg.poses.push_back(waypoints_2);
        // goal_msg.poses.push_back(waypoints_3);
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = waypoints_1;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = 
            rclcpp_action::Client<follow_waypoints_action>::SendGoalOptions();

        send_goal_options.goal_response_callback = 
            std::bind(&MinimalActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&MinimalActionClient::feedback_callback, this, _1, _2); 
        send_goal_options.result_callback = 
            std::bind(&MinimalActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        waypoint_start_time = this->now();
    }

private:
    rclcpp_action::Client<follow_waypoints_action>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    int current_waypoints = -1;
    rclcpp::Time waypoint_start_time;

    void goal_response_callback(
        std::shared_future<GoalHandleFollowWaypoints::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result"); 
        }

    }

    void feedback_callback(
        GoalHandleFollowWaypoints::SharedPtr goal_handle_ptr,
        const std::shared_ptr<const follow_waypoints_action::Feedback> feedback)
    {
        // if (this->current_waypoints != feedback->current_waypoint) {
            // this->current_waypoints = feedback->current_waypoint; 
            // waypoint_start_time = this->now();
        // }

        // RCLCPP_INFO_STREAM(this->get_logger(), "Current waypoints " << 
                // feedback->current_waypoint);

        auto elapsed = this->now() - this->waypoint_start_time;
        RCLCPP_INFO_STREAM(this->get_logger(), "Execution time: " << 
                elapsed.seconds());
        if (elapsed.seconds() > 20) {
            this->client_ptr_->async_cancel_goals_before(this->now());
            // auto cancel_result_future = this->client_ptr_->async_cancel_all_goals();
            // this->current_waypoints = -1;
            this->client_ptr_ = 
                rclcpp_action::create_client<follow_waypoints_action>(
                this,
                "/navigate_to_pose");

            this->send_goal();

            //   rclcpp::FutureReturnCode::SUCCESS)
            // {
            //   RCLCPP_ERROR(this->get_logger(), "failed to cancel goal");
            //   rclcpp::shutdown();
            //   return;
            // }
            // RCLCPP_INFO(this->get_logger(), "goal is being canceled");
            // rclcpp::shutdown();
        }
    }

    void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return; 
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        // RCLCPP_INFO_STREAM(
                // this->get_logger(),
                // "Number of missed waypoints: " <<
                // result.result->missed_waypoints.size());

        rclcpp::shutdown();
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(MinimalActionClient)
