#include "ewellix_tlt/joint_trajectory_action_server.h"

JointTrajectoryActionServer::JointTrajectoryActionServer(const std::string server_name, ros::NodeHandle &nh, SerialComTlt &srl):
    server_name_(server_name),
    nh_(nh),
    server_(nh, server_name, boost::bind(&JointTrajectoryActionServer::goalReceivedCb, this, _1), boost::bind(&JointTrajectoryActionServer::preemptReceivedCb, this, _1), false),
    server_state_(ActionServerState::INITIALIZING)
{
    srl_ = &srl;
    server_.start();
    setServerState(ActionServerState::IDLE);
}

JointTrajectoryActionServer::~JointTrajectoryActionServer(){
    return;
}

void JointTrajectoryActionServer::goalReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    ROS_INFO("JTAServer::goalReceived");
}

void JointTrajectoryActionServer::preemptReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    ROS_INFO("JATServer::goalPreempted");
}

bool JointTrajectoryActionServer::isGoalAcceptable(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    if(!goal_handle.isValid()){
        ROS_INFO("Unaccetable goal. Goal is invalid.");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoalConstPtr goal = goal_handle.getGoal();

    if(goal->trajectory.joint_names.size() != joint_names_.size()){
        ROS_INFO("Unacceptable goal. Joint number mismatch.");
        return false;
    }

    if (joint_names_ != goal->trajectory.joint_names)
    {
        ROS_INFO("Unacceptable goal. Joint names mismatch.");
        return false;
    }
    return true;
}

bool JointTrajectoryActionServer::isGoalToleranceRespected(bool check_time_tolerance){
    return true;
}

void JointTrajectoryActionServer::stopAllMovement(){
    srl_->motionStop();
}

void JointTrajectoryActionServer::setServerState(ActionServerState s){
    server_state_lock_.lock();
    server_state_ = s;
    server_state_lock_.unlock();
}
