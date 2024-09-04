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
                <CheckCommand   service_message="{service_message}"/>
                <SaySomething   message="{service_message}" />
                <SaySomething   message="mission started..." />
                <MoveBase       goal="1;2;3"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
            <ReturnRunning/>
        </ReactiveSequence>
     </BehaviorTree>

 </root>
 )";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    yggdrasil::VehicleTree node("TreeNode");
    RCLCPP_INFO(node.get_logger(), "Starting node");
    node.SetTree(xml_text_reactive);
    node.Spin();
    return 0;
}