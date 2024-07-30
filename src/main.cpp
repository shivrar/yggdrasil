//
// Created by shivan on 7/30/24.
//

#include <movebase_node.h>

#include "yggdrasil/bt_nodes.h"
#include "dummy_nodes.h"

auto xml_text_reactive = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <ReactiveSequence name="root">
            <BatteryOK/>
            <Sequence>
                <SaySomething   message="mission started..." />
                <MoveBase       goal="1;2;3"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
            <TreeTickOver/>
        </ReactiveSequence>
     </BehaviorTree>

 </root>
 )";

int main(int argc, char **argv)
{
    BehaviorTreeFactory factory;

    rclcpp::init(argc, argv);

    factory.registerSimpleCondition("BatteryOK", std::bind(DummyNodes::CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");
    factory.registerNodeType<DummyNodes::TimerNode>("Wait");
    factory.registerSimpleCondition("TreeTickOver", std::bind(yggdrasil::actions::TreeTickOver));

    yggdrasil::VehicleTree node("TreeNode");
    node.SetTree(xml_text_reactive, factory);
    node.Spin();
    return 0;
}