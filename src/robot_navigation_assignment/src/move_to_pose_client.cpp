#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robot_navigation_assignment/action/move_to_pose.hpp"

class MoveToPoseClient : public rclcpp::Node
{
public:
    using MoveToPose = robot_navigation_assignment::action::MoveToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

    MoveToPoseClient()
    : Node("move_to_pose_client")
    {
        // action client
        client_ = rclcpp_action::create_client<MoveToPose>(
        this,
        "move_to_pose"
        );
        // subscribe to /goal_pose
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&MoveToPoseClient::goal_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "MoveToPoseClient initialized.");
    }

private:
    // declare action client
    rclcpp_action::Client<MoveToPose>::SharedPtr client_;
    //  declare subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    // callback for /goal_pose
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        // wait for server
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available.");
        return;
        }
        // create goal, goal message
        MoveToPose::Goal goal;
        goal.target_pose = msg->pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal to action server...");
        auto options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();

        options.feedback_callback =
            std::bind(&MoveToPoseClient::feedback_callback, this,
                    std::placeholders::_1, std::placeholders::_2);

        options.result_callback =
            std::bind(&MoveToPoseClient::result_callback, this,
                    std::placeholders::_1);

        // send the goal 
        client_->async_send_goal(goal, options);

    }

    // feedback callback
    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const MoveToPose::Feedback> feedback)
    {
        // handle feedback  
    RCLCPP_INFO(
        this->get_logger(),
        "Current pose: x=%f, y=%f, theta=%f",
        feedback->current_x,
        feedback->current_y,
        feedback->current_theta
    );
    }

    //  result callback
    void result_callback(const GoalHandle::WrappedResult & result)
    {
        // handle result
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            if (result.result->success) {
                RCLCPP_INFO(this->get_logger(), "Final result: SUCCESS");
            } else {
                RCLCPP_WARN(this->get_logger(), "Final result: FAILED (server returned success=false)");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Action did not succeed (ABORTED or CANCELED)");
        }
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPoseClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}