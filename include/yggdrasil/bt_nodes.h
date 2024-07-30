//
// Created by shivan on 7/30/24.
//

#ifndef BT_NODES_H
#define BT_NODES_H

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/bt_action_node.hpp>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace yggdrasil {
namespace actions
{
    BT::NodeStatus inline TreeTickOver()
    {
        std::cout << "[ TreeTickOver: OK ]" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

}

class VehicleTree final : public rclcpp::Node {
public:
    explicit VehicleTree(const std::string &node_name)
    : Node(node_name),
    print_header_(node_name+"::VehcileTree::"),
    rate_()
    {
        if(!this->has_parameter("rate"))
        {
            rate_ = 2;
        } else
        {
            this->get_parameter("rate", rate_);
        }
    }

    void Spin();

    bool SetTree(const std::string&, BT::BehaviorTreeFactory &);


private:
    std::unique_ptr<BT::Tree> tree_;
    std::string print_header_;
    double rate_; // [Hz]

};

} // yggdrasil

#endif //BT_NODES_H
