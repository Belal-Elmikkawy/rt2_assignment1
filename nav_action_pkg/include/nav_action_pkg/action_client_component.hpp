#ifndef NAV_ACTION_PKG__ACTION_CLIENT_COMPONENT_HPP_
#define NAV_ACTION_PKG__ACTION_CLIENT_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_action_pkg/action/navigate.hpp"
#include "nav_action_pkg/msg/user_command.hpp"

namespace nav_action_pkg
{
class ActionClientComponent : public rclcpp::Node
{
public:
  using Navigate = nav_action_pkg::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  explicit ActionClientComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Client<Navigate>::SharedPtr client_ptr_;
  rclcpp::Subscription<nav_action_pkg::msg::UserCommand>::SharedPtr ui_sub_;
  std::shared_ptr<GoalHandleNavigate> active_goal_handle_; // Stores the active goal so we can cancel it

  void command_callback(const nav_action_pkg::msg::UserCommand::SharedPtr msg);
  void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle);
  void result_callback(const GoalHandleNavigate::WrappedResult & result);
};
}  // namespace nav_action_pkg

#endif  // NAV_ACTION_PKG__ACTION_CLIENT_COMPONENT_HPP_