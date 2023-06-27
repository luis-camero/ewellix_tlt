#include "ewellix_tlt/single_joint_position_server.h"


SingleJointPositionActionServer::SingleJointPositionActionServer(const std::string& server_name, ros::NodeHandle& nh, SerialComTlt* srl):
    server_name_(server_name),
    nh_(nh),
    as_(nh , server_name,
        boost::bind(&SingleJointPositionActionServer::goal_received_callback, this, _1),
        boost::bind(&SingleJointPositionActionServer::preempt_received_callback, this, _1),
        false
        )
{
    srl_ = srl;
    as_.start();
}

SingleJointPositionActionServer::~SingleJointPositionActionServer(){
    return;
}

void SingleJointPositionActionServer::goal_received_callback(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle new_goal){
    ROS_INFO("new goal received");
}


void SingleJointPositionActionServer::preempt_received_callback(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal){
    ROS_INFO("goal preempted");
}

void SingleJointPositionActionServer::stop_all_movement(){
    ROS_INFO("stop all movement");
}

bool SingleJointPositionActionServer::is_goal_acceptable(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal){
    ROS_INFO("is goal acceptable");
    return true;
}

bool SingleJointPositionActionServer::is_goal_tolerance_respected(bool enable_prints, bool check_time_tolerance){
    ROS_INFO("is goal tolerance respected");
    return true;
}

