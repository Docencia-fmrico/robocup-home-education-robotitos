#include "carry_my_luggage/GotoArena.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/LaserScan.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "carry_my_luggage/BTNavAction.h"
#include <string.h>

namespace carry_my_luggage
{

GotoArena::GotoArena(
    const std::string& name, 
    const std::string & action_name,
    const BT::NodeConfiguration& config)
: BTNavAction(name, action_name, config)
{
}

void
GotoArena::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}


void
GotoArena::on_halt() {
    cancel_goal();
}

void
GotoArena::on_start() {
    move_base_msgs::MoveBaseGoal goal;
    double param;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    n_.getParam("/clase_pos/pasillo/x_position", param);
    goal.target_pose.pose.position.x = param;
    n_.getParam("/clase_pos/pasillo/y_position", param);
    goal.target_pose.pose.position.y = param;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    n_.getParam("/clase_pos/pasillo/z_orientation", param);
    goal.target_pose.pose.orientation.z = param;
    goal.target_pose.pose.orientation.w = 1.0;

    set_goal(goal);

    ROS_INFO("Move start");
}

BT::NodeStatus
GotoArena::on_tick()
{
    if (counter_++ == 20)
    {
        move_base_msgs::MoveBaseGoal goal;
        double param;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        n_.getParam("/clase_pos/inicial/x_position", param);
        goal.target_pose.pose.position.x = param;
        n_.getParam("/clase_pos/inicial/y_position", param);
        goal.target_pose.pose.position.y = param;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        n_.getParam("/clase_pos/inicial/z_orientation", param);
        goal.target_pose.pose.orientation.z = param;
        goal.target_pose.pose.orientation.w = 1.0;

        set_goal(goal);
    }

    return BT::NodeStatus::RUNNING;
}

}   // namespace carry_my_luggage
