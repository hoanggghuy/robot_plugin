#ifndef IS_ELEVATOR_DOOR_OPEN_HPP_
#define IS_ELEVATOR_DOOR_OPEN_HPP_
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace real_robot_plugins
{
  class IsElevatorDoorOpen : public BT::ConditionNode
  {
    public:
      IsElevatorDoorOpen(
        const std::string & name,
        const BT::NodeConfiguration & config);
      
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

        std::string topic_name_ ;
        double threshold_distance_;
        double angle_range_;
  };
}
#endif