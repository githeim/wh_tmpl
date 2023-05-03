#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <chrono>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "bt_plugins.h"


// For loggers
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"



static const char* xml_text_reactive = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <ReactiveSequence name="root">
            <BatteryOK/>
            <MoveToGoal x="11.2" y="21.5" yaw="1.57" />
            <OpenDoor Door="FrontDoor"/>
            <MoveToGoal x="5.2" y="3.5" yaw="-1.57" />
            <OpenDoor Door="SideDoor"/>
        </ReactiveSequence>
     </BehaviorTree>
 </root>
 )";




int main(int argc, char *argv[]) {
  printf("\033[1;33m[%s][%d] :x: WW_PrintLetter_WW \033[m\n",
      __FUNCTION__,__LINE__);

  (void)xml_text_reactive;

  BT::BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<MoveAction>("MoveToGoal");
  factory.registerNodeType<OpenDoor>("OpenDoor");

  // :x: Use XML file or Text variable
  //auto tree = factory.createTreeFromText(xml_text_reactive);
  auto tree = factory.createTreeFromFile("resource/test_tree.xml");

  // :x: stdout text logger
  // BT::StdCoutLogger logger_cout(tree);

  // :x: for Groot runtime monitoring
  BT::PublisherZMQ publisher_zmq(tree);

  BT::NodeStatus status = tree.tickRoot();
  while(status == BT::NodeStatus::RUNNING ||
      status == BT::NodeStatus::FAILURE)
  {

    std::cout << "--- ticking\n";
    status = tree.tickRoot();
    usleep(300000);

  }

  return 0;
}
