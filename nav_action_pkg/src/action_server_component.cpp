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
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerComponent::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::thread{std::bind(&ActionServerComponent::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ActionServerComponent::execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Navigate::Feedback>(); // FIXED: std::
  auto result = std::make_shared<Navigate::Result>();

  rclcpp::Rate loop_rate(10);
  geometry_msgs::msg::Twist cmd_vel_msg;

  double k_rho = 0.5;
  double k_alpha = 1.5;
  double distance_tolerance = 0.1;

  bool position_reached = false;
  double theta_tolerance = 0.05;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");

      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      return;
    }

    geometry_msgs::msg::TransformStamped transformStamped; // FIXED: Capital S
    try {
      transformStamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
      loop_rate.sleep();
      continue;
    }

    double current_x = transformStamped.transform.translation.x;
    double current_y = transformStamped.transform.translation.y;
    double current_yaw = tf2::getYaw(transformStamped.transform.rotation);

    double delta_x = goal->x - current_x;
    double delta_y = goal->y - current_y; // FIXED: "couble" to "double"
    double distance_error = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    if (distance_error < distance_tolerance) {
      position_reached = true;
    }

    feedback->distance_to_target = distance_error;
    goal_handle->publish_feedback(feedback);

    if (!position_reached) {
      double target_angle = std::atan2(delta_y, delta_x);
      double angle_error = target_angle - current_yaw;

      while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
      while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

      cmd_vel_msg.linear.x = k_rho * distance_error;
      cmd_vel_msg.angular.z = k_alpha * angle_error;

      if (cmd_vel_msg.linear.x > 0.5) cmd_vel_msg.linear.x = 0.5;
    } else {
      // Rotate to target orientation (theta)
      double angle_error = goal->theta - current_yaw;

      while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
      while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

      if (std::abs(angle_error) < theta_tolerance) {
        break; // Completely reached the target pose
      }

      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = k_alpha * angle_error;
    }

    if (cmd_vel_msg.angular.z > 1.0) cmd_vel_msg.angular.z = 1.0;
    if (cmd_vel_msg.angular.z < -1.0) cmd_vel_msg.angular.z = -1.0;

    cmd_vel_pub_->publish(cmd_vel_msg);
    loop_rate.sleep();
  } // FIXED: Ensured the while loop closes correctly here

  if (rclcpp::ok()) {
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
  }
}

}  // namespace nav_action_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_pkg::ActionServerComponent)