#include "real_robot_plugins/is_battery_low_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace real_robot_plugins
{
IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  // Lấy node ROS 2 từ Blackboard để tạo Subscriber
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10,
    std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1));
}

void IsBatteryLowCondition::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{

  last_battery_percentage_ = msg->percentage * 100.0;
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  float min_battery;
  if (!getInput("min_battery", min_battery)) {
    min_battery = 10.0; // Giá trị mặc định nếu XML không ghi
  }

  if (last_battery_percentage_ < min_battery) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Pin yếu: %.2f%%", last_battery_percentage_);
    return BT::NodeStatus::SUCCESS; // Pin thấp -> Kích hoạt nhánh đi sạc
  }

  return BT::NodeStatus::FAILURE; // Pin ổn -> Tiếp tục làm việc bình thường
}
} 

// Đăng ký Plugin 
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<real_robot_plugins::IsBatteryLowCondition>("IsBatteryLow");
}