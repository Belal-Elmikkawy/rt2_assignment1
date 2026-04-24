#ifndef NAV_ACTION_PKG__ACTION_SERVER_COMPONENT_HPP_
#define NAV_ACTION_PKG__ACTION_SERVER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_action_pkg/action/navigate.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namspace nav_action_pkg
{
class ActionServerComponent : public rclcpp::Node
{
public:
  using Navigate = nav_action_pkg::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  explicit ActionServerComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  //TF2 for robot localization
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //Action server callbacks
  rclccpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_otr<const Navigate::goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::share_ptr<GoalHandleNavigate> goal_handle);
  
  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  //The main execution loop
  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);
};
}  // namespace nav_action_pkg

#endif 