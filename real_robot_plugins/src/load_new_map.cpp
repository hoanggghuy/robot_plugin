#include "real_robot_plugins/load_new_map.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace real_robot_plugins
{

LoadNewMap::LoadNewMap(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("load_map_action_node");
    map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
}

BT::PortsList LoadNewMap::providedPorts()
{
    return {
        BT::InputPort<std::string>("map_url"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("yaw")
    };
}

BT::NodeStatus LoadNewMap::tick()
{
    std::string map_url;
    double x, y, yaw;

    if (!getInput("map_url", map_url) || !getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
        return BT::NodeStatus::FAILURE;
    }

    if (!map_client_->wait_for_service(std::chrono::seconds(2))) {
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_url;

    auto result = map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = node_->get_clock()->now();
    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);

    return BT::NodeStatus::SUCCESS;
}

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<real_robot_plugins::LoadNewMap>("LoadNewMap");
}