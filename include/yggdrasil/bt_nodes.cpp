//
// Created by shivan on 7/30/24.
//

#include "bt_nodes.h"

namespace yggdrasil {


BT::NodeStatus nodes::CheckCommand::onStart()
{

    service_ = node_->create_service<std_srvs::srv::Trigger>("command",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) -> void
    {
        (void)request;
        response->success = true;
        response->message = "Service request received";
        service_message_ = response->message;
        RCLCPP_INFO(node_->get_logger(), "Service request received: %s", response->message.c_str());
        service_request_received_ = true;
    });

    return BT::NodeStatus::RUNNING;
}

void nodes::CheckCommand::onHalted()
{
    service_request_received_ = false;
}

BT::NodeStatus nodes::CheckCommand::onRunning()
{
    if (service_request_received_)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "onRunning(), Success!");
        setOutput("service_message", service_message_);
        service_request_received_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    // Continue to wait
    return BT::NodeStatus::RUNNING;
}


void VehicleTree::Spin()
{
    rclcpp::Rate r(rate_);
    while (rclcpp::ok())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), print_header_<<"Spin(), Start of loop");
        rclcpp::spin_some(this->get_node_base_interface());
        if(tree_)
        {
            tree_->tickExactlyOnce();
        } else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), print_header_<<"Spin(), TREE DOES NOT EXIST");
        }
        r.sleep();
    }
    rclcpp::shutdown();
}

bool VehicleTree::SetTree(const std::string& xml_tree)
{
    bool ret_val = false;
    if(!xml_tree.empty())
    {
        tree_ = std::make_unique<BT::Tree>(Singleton<BT::BehaviorTreeFactory>::getInstance().createTreeFromText(xml_tree));
        ret_val = true;
    }
    RCLCPP_INFO_STREAM(this->get_logger() , print_header_<<"SetTree(), successful");
    return ret_val;
}


} // yggdrasil