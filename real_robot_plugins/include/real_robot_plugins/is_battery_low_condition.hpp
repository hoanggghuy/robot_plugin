#ifndef IS_BATTERY_LOW_CONDITION_HPP_
#define IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace real_robot_plugins
{
  class IsBatteryLowCondition : public BT::ConditionNode
  {
    public:
      IsBatteryLowCondition(
        const std::string & name,
        const BT::NodeConfiguration & conf);
      
      IsBatteryLowCondition() = delete; 

      BT::NodeStatus tick() override; 

      void initialize();
      void createROSInterfaces();

      static BT::PortsList providedPorts()
      {
        return {
          BT::InputPort<double>("min_battery", 15.0, "Minimum battery percentage to trigger the condition"),
          BT::InputPort<std::string>("battery_topic", "/battery_status", "Battery topic name"),
          BT::InputPort<bool>("is_voltage", false, "Use voltage instead of percentage")
        };
      }

    private:
      void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
      
      rclcpp::Node::SharedPtr node_;
      rclcpp::CallbackGroup::SharedPtr callback_group_;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
      rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_; 
      
      std::string battery_topic_;
      double min_battery_;
      bool is_voltage_;
      bool is_battery_low_;
      std::chrono::milliseconds bt_loop_duration_;
  };
}
#endif