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
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace yggdrasil {
namespace nodes
{
    BT::NodeStatus inline TreeTickOver()
    {
        std::cout << "[ TreeTickOver: OK ]" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    class CheckCommand final : public BT::StatefulActionNode
    {
    public:
        // Any TreeNode with ports must have a constructor with this signature
        CheckCommand(const std::string& name, const BT::NodeConfiguration& config, const rclcpp::Node::SharedPtr& node)
        : BT::StatefulActionNode(name, config), service_request_received_(false), service_message_(), node_(node)
        {}

        ~CheckCommand() override = default;

        static BT::PortsList providedPorts()
        {
            return { BT::OutputPort<std::string>("service_message") };
        }

        // this function is invoked once at the beginning.
        BT::NodeStatus onStart() override;

        // If onStart() returned RUNNING, we will keep calling
        // this method until it return something different from RUNNING
        BT::NodeStatus onRunning() override;

        // callback to execute if the action was aborted by another node
        void onHalted() override;

    private:
        bool service_request_received_;
        std::string  service_message_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    };
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
