#include "nav_action_pkg/action_server_component.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace nav_action_pkg
{

ActionServerComponent::ActionServerComponent(const rclcpp::NodeOptions & options)
: Node("action_server_component", options)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize Action Server to listen for navigate_robot action requests
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
  // Accept the incoming goal and automatically execute it
  RCLCPP_INFO(this->get_logger(), "Received goal request: x=%.2f, y=%.2f, theta=%.2f", goal->x, goal->y, goal->theta);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerComponent::handle_cancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  // Accept any incoming cancellation requests to stop the robot
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerComponent::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  // Execute the navigation logic in a separate thread to avoid blocking the ROS 2 executor
  std::thread{std::bind(&ActionServerComponent::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ActionServerComponent::execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Navigate::Feedback>();
  auto result = std::make_shared<Navigate::Result>();

  // Set the control loop rate to 10 Hz
  rclcpp::Rate loop_rate(10);
  geometry_msgs::msg::Twist cmd_vel_msg;

  // Proportional control gains
  double k_rho = 0.5;    // Gain for linear velocity (moving towards the target distance)
  double k_alpha = 1.5;  // Gain for angular velocity (turning towards the target angle)

  // Tolerances to consider the target reached
  double distance_tolerance = 0.1;

  bool position_reached = false;   // Flag to track if the (x, y) coordinate is reached
  double theta_tolerance = 0.05;   // Tolerance for the final orientation

  // Main navigation loop
  while (rclcpp::ok()) {
    // 1. Check if the client requested to cancel the goal
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");

      // Stop the robot before returning
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      return;
    }

    // Obtain the robot's current pose using TF2 (from odom to base_link)
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
      loop_rate.sleep();
      continue;
    }

    // Extract current coordinates and yaw angle from the transform
    double current_x = transformStamped.transform.translation.x;
    double current_y = transformStamped.transform.translation.y;
    double current_yaw = tf2::getYaw(transformStamped.transform.rotation);

    // Compute error values
    double delta_x = goal->x - current_x;
    double delta_y = goal->y - current_y; // FIXED: "couble" to "double"
    double distance_error = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    // Check if the (x, y) location has been reached
    if (distance_error < distance_tolerance) {
      position_reached = true;
    }

    // Publish feedback to the client
    feedback->distance_to_target = distance_error;
    goal_handle->publish_feedback(feedback);

    // Compute control velocities
    if (!position_reached) {
      // Phase 1: Move towards the target position (x, y)
      double target_angle = std::atan2(delta_y, delta_x);
      double angle_error = target_angle - current_yaw;

      // Normalize the angle error to be within [-pi, pi]
      while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
      while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

      // Apply proportional control logic
      cmd_vel_msg.linear.x = k_rho * distance_error;
      cmd_vel_msg.angular.z = k_alpha * angle_error;

      // Limit the maximum linear speed
      if (cmd_vel_msg.linear.x > 0.5) cmd_vel_msg.linear.x = 0.5;
    } else {
      // Phase 2: Rotate to target orientation (theta) once position is reached
      double angle_error = goal->theta - current_yaw;

      // Normalize the angle error to be within [-pi, pi]
      while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
      while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

      // Check if the orientation is perfectly aligned
      if (std::abs(angle_error) < theta_tolerance) {
        break; // Completely reached the target pose, exit the loop!
      }

      // Turn in place
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = k_alpha * angle_error;
    }

    // Limit the maximum angular speed to prevent excessive spinning
    if (cmd_vel_msg.angular.z > 1.0) cmd_vel_msg.angular.z = 1.0;
    if (cmd_vel_msg.angular.z < -1.0) cmd_vel_msg.angular.z = -1.0;

    // Publish the computed command velocities to the robot
    cmd_vel_pub_->publish(cmd_vel_msg);
    loop_rate.sleep();
  } // FIXED: Ensured the while loop closes correctly here

  // Handle successful goal completion
  if (rclcpp::ok()) {
    // Stop the robot completely
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Mark the action as succeeded and return the result
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
  }
}

}  // namespace nav_action_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_pkg::ActionServerComponent)