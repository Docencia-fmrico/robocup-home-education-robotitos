#include <string>
#include <memory>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ros/package.h"

#include "find_my_mates/DetectPerson.h"
#include "find_my_mates/GotoReferee.h"
#include "find_my_mates/GotoArena.h"
#include "find_my_mates/GotoPerson.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_my_mates");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerNodeType<find_my_mates::DetectPerson>("DetectPerson");
  factory.registerNodeType<find_my_mates::DetectPerson>("GotoArena");
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<find_my_mates::GotoReferee>(name, "move_base", config);
    };

  factory.registerBuilder<find_my_mates::GotoReferee>("GotoReferee", builder);

  BT::NodeBuilder builder1 =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<find_my_mates::GotoPerson>(name, "move_base", config);
    };

  factory.registerBuilder<find_my_mates::GotoPerson>("GotoPerson", builder1);

  auto blackboard = BT::Blackboard::create();

  std::string pkgpath = ros::package::getPath("find_my_mates");
  std::string xml_file = pkgpath + "/find_my_mates_xml/find_my_mates1.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    tree.rootNode()->executeTick();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}