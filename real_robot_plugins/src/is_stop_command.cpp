#include "real_robot_plugins/is_stop_command.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace real_robot_plugins
{
IsStopCommand::IsStopCommand(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  voice_topic_("/voice_commands"),
  is_stop_received_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
  
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  
  voice_sub_ = node_->create_subscription<std_msgs::msg::String>(
    voice_topic_, 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data == "stop") is_stop_received_ = true;
    }, options);
}

BT::NodeStatus IsStopCommand::tick()
{
  callback_group_executor_.spin_all(bt_loop_duration_);
  if (is_stop_received_) {
    is_stop_received_ = false; 
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
} 

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<real_robot_plugins::IsStopCommand>("IsStopCommand");
}