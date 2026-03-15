#ifndef LOAD_NEW_MAP_HPP_
#define LOAD_NEW_MAP_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace real_robot_plugins
{

class LoadNewMap : public BT::SyncActionNode
{
public:
    LoadNewMap(
        const std::string & name,
        const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

}

#endif