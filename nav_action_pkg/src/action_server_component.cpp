#include "nav_action_pkg/action_server_component.hpp"
#include "tf2/utils.h" // Required for tf2::getYaw
#include <cmath>

namespace nav_action_pkg {
ActionServerComponent:: ActionServerComponent(const rclcpp::NodeOptions & options)
: Node("action_server_component", options)
{
  //Initialize Publisher for velocities
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

  //Initialize TF2 Buffer and Listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //Initialize Action Server
  action_server_ = rclcpp_action::create_server<Navigate>(
               this,
               "navigate_robot",
               std::bind(&ActionServerComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
               std::bind(&ActionServerComponent::handle_cancel, this, std::placeholders::_1),
               std::bind(&ActionServerComponent::handle_accepted, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Action Server Component Initialized.");
}

rclcpp_action::GoalResponse ActionServerComponent::handle_goal(
               const rclcpp_action::GoalUUID & uuid,
               std::shared_ptr<const Navigate::Goal> goal)
{
               RCLCPP_INFO(this->get_logger(), "Received goal request: x=%.2f, y=%.2f, theta=%.2f", goal->x, goal->y, goal->theta);
               (void)uuid;
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerComponent::handle_cancel(
               const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
               RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal");
               (void)goal_handle;
               return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerComponent::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
               //Execute the goal in a separate thread so we don't block the executor
               std::thread{std::bind(&ActionServerComponent::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ActionServerComponent::execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
               RCLCPP_INFO(this->get_logger(), "Executing goal");
               const auto goal = goal_handle->get_goal();
               auto feedback = std:;make_shared<Navigate::Feedback>();
               auto result = std::make_shared<Navigate::Result>();

               rclcpp::Rate loop_rate(10);  //10 hz control loop
               geometry_msgs::msg::Twist cmd_vel_msg;

               //Control Parameters
               double k_rho = 0.5;   //Linear velocity gain
               double k_alpha = 1.5  //Angular velocity gain
               double distance_tolerance = 0.1;

               while(rclcpp::ok()){
                              //Check if there is a cancel request
                              if(goal_handle->is_canceling()){
                                             result->success = false;
                                             goal_handle->canceled(result);
                                             RCLCPP_INFO(this->get_logger(), "Goal canceled");

                                             //stop the robot
                                             cmd_vel_msg.linear.x= 0.0;
                                             cmd_vel_msg.angular.z = 0.0;
                                             cmd_vel_pub_->publish(cmd_vel_msg);
                                             return;
                              }

               }

               //Get current robot pose via TF2
               geometry_msgs::msg::TransformStamped transformstamped;
               try{
                              //Assuming 'odom' is the fixed frame and 'base_link' is the robot frame
                              transformstamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
               }              catch (const tf2::TransformException & ex) {
                              RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
                              loop_rate.sleep();
                              continue;
               }

               double current_x = transformStamped.transform.translation.x;
               double current_y = transformStamped.transform.translation.y;
               double current_yaw = tf2::getYaw(transformStamped.transform.rotation);

               //Calculate error
               double delta_x = goal->x - current_x;
               couble delta_y = goal->y - current_y;
               double distance_error = std::sqrt(delta_x * delta_x + delta_y * delta_y);

               //Calculate angle to the target point
               double target_angle = std::atan2(delta_y, delta_x);
               double angle_error = target_angle - current_yaw;

               //Normalize angle error [-pi,pi]
               while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
               while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

               //Publish feedback
               feedback->distance_to_target = distance_error;
               goal_handle->publish_feedback(feedback);

               //Check if goal is reached
               if(distance_error < distance_tolerance){
                              break;
               }

               //Proportional control law
               cmd_vel_msg.linear.x = k_rho * distance_error;
               cmd_vel_msg.angular.z = k_alpha * angle_error;

               cmd_vel_pub_->publish(cmd_vel_msg);
               loop_rate.sleep();
               }

               if(rclcpp::ok()){
                              //stop robot upon reaching the target
                              cmd_vel_msg.linear.x = 0.0;
                              cmd_vel_msg.angular.z = 0.0;
                              cmd_vel_pub_->publish(cmd_vel_msg);

                              result->success = true;
                              goal_handle->succeed(result);
                              RCLCPP_INFO(this->get_logger(), "Goal Succeeded!");

               }


}  // namespace nav_action_pkg

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_pkg::ActionServerComponent)
