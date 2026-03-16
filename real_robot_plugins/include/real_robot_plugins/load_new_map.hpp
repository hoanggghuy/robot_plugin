#ifndef LOAD_NEW_MAP_HPP_
#define LOAD_NEW_MAP_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <future>
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
namespace real_robot_plugins
{

class LoadNewMap : public BT::StatefulActionNode
{
public:
    LoadNewMap(
        const std::string & name,
        const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_client_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Time start_time_;
    double timeout_duration_=10.0; // seconds
    std::string map_url_;
    double x_, y_, yaw_;
    std::shared_future<nav2_msgs::srv::LoadMap::Response::SharedPtr> future_result_;
};

}

#endif