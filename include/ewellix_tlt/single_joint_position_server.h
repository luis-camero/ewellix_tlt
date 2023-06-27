#ifndef SINGLE_JOINT_H
#define SINGLE_JOINT_H

#include "serial_com_tlt.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/SingleJointPositionAction.h>

#include <chrono>

class SingleJointPositionActionServer
{
    public:
        SingleJointPositionActionServer() = delete;
        SingleJointPositionActionServer(const std::string& server_name, ros::NodeHandle& nh, SerialComTlt* srl);
        ~SingleJointPositionActionServer();

    private:
        ros::NodeHandle nh_;
        std::string server_name_;
        actionlib::ActionServer<control_msgs::SingleJointPositionAction> as_;
        SerialComTlt* srl_;
        control_msgs::SingleJointPositionFeedback feedback_;
        actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal_;
        std::chrono::system_clock::time_point start_;
        std::chrono::system_clock::time_point end_;

        // Tolerance
        double default_goal_time_tolerance_;
        double default_goal_tolerance_;

        // Action Server Callbacks
        void goal_received_callback(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle new_goal);
        void preempt_received_callback(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal);
        void stop_all_movement();

        // Action Server Checks
        bool is_goal_acceptable(actionlib::ActionServer<control_msgs::SingleJointPositionAction>::GoalHandle goal);
        bool is_goal_tolerance_respected(bool enable_prints, bool check_time_tolerance);

        // Action Server State
        //void set_server_state(ActionServerState s);
};

#endif //SINGLE_JOINT
