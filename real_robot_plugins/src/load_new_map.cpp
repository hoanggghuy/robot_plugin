#include "real_robot_plugins/load_new_map.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace real_robot_plugins
{

LoadNewMap::LoadNewMap(const std::string & name, const BT::NodeConfiguration & config)
    : BT::StatefulActionNode(name, config) 
{
    if (!config.blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
        node_ = rclcpp::Node::make_shared("load_map_fallback_node");
    }
    map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    global_costmap_client_ =
    node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entirely_global_costmap");

    local_costmap_client_ =
        node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
            "/local_costmap/clear_entirely_local_costmap");
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
BT::NodeStatus LoadNewMap::onStart()
{
    if (!getInput("map_url", map_url_) || !getInput("x", x_) || !getInput("y", y_) || !getInput("yaw", yaw_)) {
        return BT::NodeStatus::FAILURE;
    }

    if (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        return BT::NodeStatus::FAILURE;
    }
    start_time_ = node_->get_clock()->now();
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_url_;
    future_result_ = map_client_->async_send_request(request).share();
    
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus LoadNewMap::onRunning()
{
    // rclcpp::spin_some(node_);
    auto status = future_result_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
        auto response = future_result_.get();
        if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Load map failed");
            return BT::NodeStatus::FAILURE;
        }
        auto clear_req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

        global_costmap_client_->async_send_request(clear_req);
        local_costmap_client_->async_send_request(clear_req);
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = node_->get_clock()->now();
        pose_msg.pose.pose.position.x = x_;
        pose_msg.pose.pose.position.y = y_;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        pose_pub_->publish(pose_msg);
        return BT::NodeStatus::SUCCESS;
    }
    if ((node_->get_clock()->now() - start_time_).seconds() > timeout_duration_) {
        RCLCPP_ERROR(node_->get_logger(), "Load map timed out");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}
void LoadNewMap::onHalted()
{
}
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<real_robot_plugins::LoadNewMap>("LoadNewMap");
}