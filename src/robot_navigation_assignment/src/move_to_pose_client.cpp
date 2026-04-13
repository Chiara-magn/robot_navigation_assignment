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
        // subscribe to /goal_pose
    }

private:
    // declare action client
    //  declare subscription

    // TODO: callback for /goal_pose
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // wait for server
        // create goal
        // send goal
    }

    // feedback callback
    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const MoveToPose::Feedback> feedback)
    {
        // handle feedback
    }

    //  result callback
    void result_callback(const GoalHandle::WrappedResult & result)
    {
        // handle result
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