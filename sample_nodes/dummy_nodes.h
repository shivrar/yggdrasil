#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/bt_action_node.hpp>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace BT;

// let's define these, for brevity
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciAction: public RosActionNode<Fibonacci>
{
public:
  FibonacciAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<Fibonacci>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("order")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override
  {
    // get "order" from the Input port
    getInput("order", goal.order);
    // return true, if we were able to set the goal correctly.
    return true;
  }

  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : wr.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(node_.lock()->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << BT::toStr(error));
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(node_.lock()->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};

namespace DummyNodes
{

using BT::NodeStatus;

NodeStatus CheckBattery();
NodeStatus TreeTickOver();
NodeStatus CheckTemperature();
NodeStatus SayHello();

class GripperInterface
{
public:
  GripperInterface() : _opened(true)
  {}

  NodeStatus open();

  NodeStatus close();

private:
  bool _opened;
};

//--------------------------------------

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  NodeStatus tick() override;
};

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  // You must override the virtual function tick()
  NodeStatus tick() override;

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }
};

//Same as class SaySomething, but to be registered with SimpleActionNode
NodeStatus SaySomethingSimple(BT::TreeNode& self);

// Example os Asynchronous node that use StatefulActionNode as base class
class SleepNode : public BT::StatefulActionNode
{
public:
  SleepNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    // amount of milliseconds that we want to sleep
    return { BT::InputPort<int>("msec") };
  }

  NodeStatus onStart() override
  {
    int msec = 0;
    getInput("msec", msec);
    if(msec <= 0)
    {
      // no need to go into the RUNNING state
      return NodeStatus::SUCCESS;
    }
    else
    {
      using namespace std::chrono;
      // once the deadline is reached, we will return SUCCESS.
      deadline_ = system_clock::now() + milliseconds(msec);
      return NodeStatus::RUNNING;
    }
  }

  /// method invoked by an action in the RUNNING state.
  NodeStatus onRunning() override
  {
    if(std::chrono::system_clock::now() >= deadline_)
    {
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::RUNNING;
    }
  }

  void onHalted() override
  {
    // nothing to do here...
    std::cout << "SleepNode interrupted" << std::endl;
  }

private:
  std::chrono::system_clock::time_point deadline_;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  static GripperInterface grip_singleton;

  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
  factory.registerSimpleCondition("CheckTemperature", std::bind(CheckTemperature));
  factory.registerSimpleAction("SayHello", std::bind(SayHello));
  factory.registerSimpleAction("OpenGripper",
                               std::bind(&GripperInterface::open, &grip_singleton));
  factory.registerSimpleAction("CloseGripper",
                               std::bind(&GripperInterface::close, &grip_singleton));
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<SaySomething>("SaySomething");
}
// Tester for a change
using namespace BT;

class TimerNode : public SyncActionNode
{
public:
  TimerNode(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config), timer_started_(false) {}

  static PortsList providedPorts()
  {
    return { InputPort<unsigned>("duration") };
  }

  NodeStatus tick() override
  {
    unsigned duration;
    if (!getInput("duration", duration))
    {
      throw BT::RuntimeError("missing required input [duration]");
    }

    auto now = std::chrono::steady_clock::now();

    if (!timer_started_)
    {
      start_time_ = now;
      timer_started_ = true;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);

    if (elapsed.count() >= duration)
    {
      timer_started_ = false;
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::RUNNING;
    }
  }

private:
  bool timer_started_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace DummyNodes

#endif  // SIMPLE_BT_NODES_H
