#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_navigation_assignment/action/move_to_pose.hpp"

namespace robot_navigation_assignment
{
    class NavigationServer : public rclcpp::Node {
    public:
        NavigationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        rclcpp_action::Server<robot_navigation_assignment::action::MoveToPose>::SharedPtr action_server_;

        // callback declarations
        // handle goal callback declaration
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const robot_navigation_assignment::action::MoveToPose::Goal> goal);
        // handle cancel callback declaration
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_navigation_assignment::action::MoveToPose>> goal_handle);
        // handle accepted callback delcaration
        void handle_accepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_navigation_assignment::action::MoveToPose>> goal_handle);
        // execute funcrion declaration
        void execute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_navigation_assignment::action::MoveToPose>> goal_handle);
        };
} 

RCLCPP_COMPONENTS_REGISTER_NODE(robot_navigation_assignment::NavigationServer)

robot_navigation_assignment::NavigationServer::NavigationServer(
    const rclcpp::NodeOptions & options)
: Node("navigation_server", options)
{
    using MoveToPose = robot_navigation_assignment::action::MoveToPose; // rename

    action_server_ = rclcpp_action::create_server<MoveToPose>(
        this,
        "move_to_pose",
        std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
    );
}

// goal response implementation 
rclcpp_action::GoalResponse
robot_navigation_assignment::NavigationServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const robot_navigation_assignment::action::MoveToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// cancel response implementation 
rclcpp_action::CancelResponse
robot_navigation_assignment::NavigationServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_navigation_assignment::action::MoveToPose>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
}

// handle accepted implementation
void robot_navigation_assignment::NavigationServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_navigation_assignment::action::MoveToPose>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted");

    std::thread{[this, goal_handle]() { // separated thread to handle this 
        this->execute(goal_handle);
    }}.detach();
}

// execution function implementation 
void robot_navigation_assignment::NavigationServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_navigation_assignment::action::MoveToPose>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    const auto goal = goal_handle->get_goal(); // getting the goal
    auto feedback = std::make_shared<robot_navigation_assignment::action::MoveToPose::Feedback>();
    auto result   = std::make_shared<robot_navigation_assignment::action::MoveToPose::Result>();
    
    for (int i = 1; i <= 10; ++i) {
            
        // check goal cancel
        if (goal_handle->is_canceling()) {
            result->success = false;
            RCLCPP_INFO(this->get_logger(), "Goal canceled"); // log message
            goal_handle->canceled(result); // temporary result
            return;
        }
        feedback->progress = i * 10;  // so the log has feedback on the goal status
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Progress: %d%%", feedback->progress); // progress log print
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // execution time delay
    }
    result->success = true; // if the process succeded to finish
    RCLCPP_INFO(this->get_logger(), "Goal succeeded"); // log message: success
    goal_handle->succeed(result); // result success

}