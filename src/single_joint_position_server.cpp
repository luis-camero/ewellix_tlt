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
    ROS_INFO("New Goal Received");
    bool success = true;

    if(is_goal_acceptable(new_goal)){
        goal_ = new_goal;
        goal_.setAccepted();
    }
    else{
        new_goal.setRejected();
        return;
    }

    bool moving = true;
    srl_->current_target_ = goal_.getGoal()->position;
    while(moving)
    {
        ROS_INFO("Moving...");
        sleep(1);
        moving = srl_->process_target_;
    }
    if(success)
    {
        ROS_INFO("Done.");
    }
    else
    {
        ROS_INFO("Failed.");
    }
}

void SingleJointPositionActionServer::preempt_received_callback(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal){
    ROS_INFO("goal preempted");
    srl_->stop();
    sleep(1);
    goal_received_callback(goal);
}

void SingleJointPositionActionServer::stop_all_movement(){
    ROS_INFO("stop all movement");
    srl_->stop();
}

bool SingleJointPositionActionServer::is_goal_acceptable(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal){
    ROS_INFO("is goal acceptable");
    return true;
}

bool SingleJointPositionActionServer::is_goal_tolerance_respected(bool enable_prints, bool check_time_tolerance){
    ROS_INFO("is goal tolerance respected");
    return true;
}

