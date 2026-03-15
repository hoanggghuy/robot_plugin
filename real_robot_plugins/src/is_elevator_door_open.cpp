#include "real_robot_plugins/is_elevator_door_open.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace real_robot_plugins
{

IsElevatorDoorOpen::IsElevatorDoorOpen(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::ConditionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("is_elevator_door_open_condition");
    
    if (!getInput("topic_name", topic_name_)) {
        topic_name_ = "/scan";
    }

    if (!getInput("threshold_distance", threshold_distance_)) {
        threshold_distance_ = 1.5;
    }

    angle_range_ = 0.1745; 

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_name_, qos, std::bind(&IsElevatorDoorOpen::scanCallback, this, std::placeholders::_1));
}

BT::PortsList IsElevatorDoorOpen::providedPorts()
{
    return {
        BT::InputPort<std::string>("topic_name"),
        BT::InputPort<double>("threshold_distance")
    };
}

void IsElevatorDoorOpen::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    last_scan_ = msg;
}

BT::NodeStatus IsElevatorDoorOpen::tick()
{
    rclcpp::spin_some(node_);

    if (!last_scan_) {
        return BT::NodeStatus::FAILURE;
    }

    int center_index = last_scan_->ranges.size() / 2;
    int samples = std::max(1, static_cast<int>(angle_range_ / last_scan_->angle_increment));
    int start_index = std::max(0, center_index - samples / 2);
    int end_index = std::min(static_cast<int>(last_scan_->ranges.size() - 1), center_index + samples / 2);

    bool door_open = true;
    for (int i = start_index; i <= end_index; ++i) {
        float range = last_scan_->ranges[i];
        if (range < last_scan_->range_min || range > last_scan_->range_max) continue;
        
        if (range < threshold_distance_) {
            door_open = false;
            break;
        }
    }

    return door_open ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<real_robot_plugins::IsElevatorDoorOpen>("IsElevatorDoorOpen");
}