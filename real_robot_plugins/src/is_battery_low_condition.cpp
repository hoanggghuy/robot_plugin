#include "real_robot_plugins/is_battery_low_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace real_robot_plugins
{
IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
  initialize();
  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
}

void IsBatteryLowCondition::initialize()
{
  getInput("min_battery", min_battery_);
  getInput("is_voltage", is_voltage_);
  createROSInterfaces();
}

void IsBatteryLowCondition::createROSInterfaces()
{
  std::string battery_topic_new;
  getInput("battery_topic", battery_topic_new);

  // Only create a new subscriber if the topic has changed or subscriber is empty
  if (battery_topic_new != battery_topic_ || !battery_sub_) {
    battery_topic_ = battery_topic_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_,
      10,
      std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
      options);
  }
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  callback_group_executor_.spin_all(bt_loop_duration_);
  if (is_battery_low_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(
  const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    // msg->percentage is 0.0-1.0, min_battery_ is 0-100
    is_battery_low_ = (msg->percentage * 100.0) <= min_battery_;
  }
}



} 

// extern "C" __attribute__ ((visibility ("default"))) 
// void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
// {
//   factory.registerNodeType<real_robot_plugins::IsBatteryLowCondition>("IsBatteryLow");
// }


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<real_robot_plugins::IsBatteryLowCondition>("IsBatteryLow");
}