#ifndef REAL_ROBOT_PLUGINS__IS_STOP_COMMAND_HPP_
#define REAL_ROBOT_PLUGINS__IS_STOP_COMMAND_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace real_robot_plugins
{
class IsStopCommand : public BT::ConditionNode
{
public:
  IsStopCommand(const std::string & name, const BT::NodeConfiguration & conf);


  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_sub_;
  

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  

  std::string voice_topic_;
  std::chrono::milliseconds bt_loop_duration_;
  bool is_stop_received_;
};
}  

#endif  