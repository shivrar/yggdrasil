//
// Created by shivan on 7/30/24.
//

#include "bt_nodes.h"

namespace yggdrasil {

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

bool VehicleTree::SetTree(const std::string& xml_tree, BT::BehaviorTreeFactory &factory)
{
    bool ret_val = false;
    if(!xml_tree.empty())
    {
        tree_ = std::make_unique<BT::Tree>(factory.createTreeFromText(xml_tree));
        ret_val = true;
    }
    return ret_val;
}


} // yggdrasil