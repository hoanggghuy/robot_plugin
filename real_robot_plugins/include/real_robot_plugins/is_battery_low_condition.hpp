#ifndef IS_BATTERY_LOW_CONDITION_HPP_
#define IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace real_robot_plugins
{
class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  IsBatteryLowCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<float>("min_battery", 10.0, "Ngưỡng pin (%)") };
  }

  BT::NodeStatus tick() override;

private:

  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  float last_battery_percentage_ = 100.0; 
};
} 
#endif