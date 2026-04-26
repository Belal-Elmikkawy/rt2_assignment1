#include "nav_action_pkg/action_client_component.hpp"

namespace nav_action_pkg
{
ActionClientComponent::ActionClientComponent(const rclcpp::NodeOptions & options)
: Node("action_client_component", options)
{
  // 1. Create the Action Client
  client_ptr_ = rclcpp_action::create_client<Navigate>(this, "navigate_robot");

  // 2. Subscribe to the Python UI topic
  ui_sub_ = this->create_subscription<nav_action_pkg::msg::UserCommand>(
    "user_command_topic", 10,
    std::bind(&ActionClientComponent::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Action Client Component Initialized.");
}

void ActionClientComponent::command_callback(const nav_action_pkg::msg::UserCommand::SharedPtr msg)
{
  if (msg->is_cancel) {
    if (active_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Sending cancel request to server...");
      client_ptr_->async_cancel_goal(active_goal_handle_);
      active_goal_handle_.reset();
    } else {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
    }
    return;
  }

  if (active_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling previous goal before sending a new one...");
    client_ptr_->async_cancel_goal(active_goal_handle_);
    active_goal_handle_.reset();
  }

  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available! Cannot send goal.");
    return;
  }

  auto goal_msg = Navigate::Goal();
  goal_msg.x = msg->x;
  goal_msg.y = msg->y;
  goal_msg.theta = msg->theta;

  RCLCPP_INFO(this->get_logger(), "Sending new goal to server: x=%.2f, y=%.2f", goal_msg.x, goal_msg.y);

  auto send_goal_options = rclcpp_action::Client<Navigate>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ActionClientComponent::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&ActionClientComponent::result_callback, this, std::placeholders::_1);

  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void ActionClientComponent::goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    active_goal_handle_ = goal_handle;
  }
}

void ActionClientComponent::result_callback(const GoalHandleNavigate::WrappedResult & result)
{
  active_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "SUCCESS: Robot reached the target!");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "CANCELED: Robot stopped.");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Goal aborted or unknown error.");
      break;
  }
}
}  // namespace nav_action_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_pkg::ActionClientComponent)