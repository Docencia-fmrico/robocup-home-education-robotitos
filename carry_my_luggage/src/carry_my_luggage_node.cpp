#include <string>
#include <memory>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ros/package.h"

#include "carry_my_luggage/DetectObject.h"
#include "carry_my_luggage/FollowPerson.h"
#include "carry_my_luggage/GotoArena.h"
#include "carry_my_luggage/GotoDoor.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "carry_my_luggage");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerNodeType<visual_behavior::MakeSound>("GotoDoor");
  factory.registerNodeType<visual_behavior::DetectBall>("GotoArena");
  factory.registerNodeType<visual_behavior::DetectBall>("FollowPerson");
  factory.registerNodeType<visual_behavior::Foward>("DetectObject");
  
  auto blackboard = BT::Blackboard::create();

  std::string pkgpath = ros::package::getPath("carry_my_luggage");
  std::string xml_file = pkgpath + "/carry_my_luggage_xml/carry_my_luggage.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    tree.rootNode()->executeTick();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}