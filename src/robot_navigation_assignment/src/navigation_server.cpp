#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_navigation_assignment/action/move_to_pose.hpp"
//
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
//



namespace robot_navigation_assignment
{
    class NavigationServer : public rclcpp::Node {
    public:
        NavigationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        rclcpp_action::Server<robot_navigation_assignment::action::MoveToPose>::SharedPtr action_server_;

        // TF2: buffer e listener per le trasformazioni
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Publisher per i comandi di velocità
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

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

        // TF2 setup: creo il buffer e il listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher per i comandi di velocità
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
    
rclcpp::Rate rate(10.0);  // 10 Hz

    while (rclcpp::ok()) {

        // 1. Controllo se il goal è stato cancellato
        if (goal_handle->is_canceling()) {
            result->success = false;
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            goal_handle->canceled(result);
            return;
        }

         // 2. Trasformo la goal pose nel frame del robot

        // Creo una PoseStamped nel frame "map" (dove l'utente dà la goal)
        geometry_msgs::msg::PoseStamped goal_pose_map;
        goal_pose_map.header.frame_id = "map";
        goal_pose_map.header.stamp = this->now();
        goal_pose_map.pose = goal->target_pose;  // la pose ricevuta dall'action

        // Qui salveremo la goal trasformata nel frame del robot
        geometry_msgs::msg::PoseStamped goal_pose_robot;

        try {
            // Trasformo la goal pose da "map" → "base_link"
            goal_pose_robot = tf_buffer_->transform(
                goal_pose_map,
                "base_link",
                tf2::durationFromSec(0.1)
            );
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            rate.sleep();
            continue;   // riprova nel prossimo ciclo
        }


        // 3. Ottengo la posizione attuale del robot nel frame "map"
        geometry_msgs::msg::TransformStamped tf_robot;

        try {
            // Trasformazione da map → base_link
            tf_robot = tf_buffer_->lookupTransform(
                "map",          // frame di riferimento
                "base_link",    // frame del robot
                tf2::TimePointZero
            );
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            rate.sleep();
            continue;
        }

        // Coordinate attuali del robot
        double robot_x = tf_robot.transform.translation.x;
        double robot_y = tf_robot.transform.translation.y;

        // Orientamento attuale del robot (yaw)
        tf2::Quaternion q(
            tf_robot.transform.rotation.x,
            tf_robot.transform.rotation.y,
            tf_robot.transform.rotation.z,
            tf_robot.transform.rotation.w
        );
        double robot_yaw = tf2::getYaw(q);

        // 4. Calcolo distanza e angolo verso la goal

        // Coordinate della goal trasformata nel frame del robot
        double gx = goal_pose_robot.pose.position.x;
        double gy = goal_pose_robot.pose.position.y;

        // Distanza dal robot alla goal
        double distance = std::sqrt(gx*gx + gy*gy);

        // Angolo verso la goal (nel frame del robot)
        double angle = std::atan2(gy, gx);

        // 5. Controllo del movimento

        geometry_msgs::msg::Twist cmd;

        // Se siamo lontani dalla goal
        if (distance > 0.05) {

            // Ruota verso la goal
            cmd.angular.z = 1.0 * angle;

            // Avanza solo se l'angolo è piccolo (cioè se sei quasi allineata)
            if (std::fabs(angle) < 0.2) {
                cmd.linear.x = 0.2;
            }
        } 
        else {
            // Se siamo vicini, fermiamo il robot
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            break;  // goal raggiunta → usciamo dal while
        }

        // Pubblico il comando di velocità
        cmd_vel_pub_->publish(cmd);

// 6. Feedback reale all'action client

        feedback->progress = std::clamp(
            (1.0 - std::min(distance / 2.0, 1.0)) * 100.0,
            0.0,
            100.0
        );

        goal_handle->publish_feedback(feedback);


        rate.sleep();
    }

    result->success = true; // if the process succeded to finish
    RCLCPP_INFO(this->get_logger(), "Goal succeeded"); // log message: success
    goal_handle->succeed(result); // result success

}
