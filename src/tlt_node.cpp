#include "ewellix_tlt/tlt_node.h"


TltNode::TltNode(ros::NodeHandle private_nh):
    srv_joint_traj_(
        private_nh,
        "follow_joint_trajectory",
        boost::bind(&TltNode::cbJointTrajReceived, this, _1),
        boost::bind(&TltNode::cbJointTrajPreempted, this, _1),
        false
    )
{

    // Publishers
    pub_column_pose_ = private_nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    // Variables
    string port;
    int baudrate;

    // Parameters
    private_nh.param<string>("ewellix/port", port, "/dev/ttyUSB0");
    private_nh.param<int>("ewellix/baudrate", baudrate, 38400);


    // Subscribers
    sub_column_size_ = private_nh.subscribe("/ewellix/size", 1, &TltNode::cbColumnSize,this);
    sub_column_duration_up_ = private_nh.subscribe("/ewellix/duration_up", 1, &TltNode::cbDurationUp,this);
    sub_column_duration_down_ = private_nh.subscribe("/ewellix/duration_down", 1, &TltNode::cbDurationDown,this);

    sub_motor1_ticks_ = private_nh.subscribe("/ewellix/motor1_ticks", 1, &TltNode::cbMotor1Ticks, this);
    sub_motor2_ticks_ = private_nh.subscribe("/ewellix/motor2_ticks", 1, &TltNode::cbMotor2Ticks, this);

    sub_joy_ = private_nh.subscribe("/joy", 1, &TltNode::cbJoy,this);

    // Action
    // srv_joint_traj_(
    //     private_nh,
    //     "follow_joint_trajectory",
    //     boost::bind(&TltNode::cbJointTrajReceived, this, _1),
    //     boost::bind(&TltNode::cbJointTrajPreempted, this, _1),
    //     false
    // );


    cout << "connecting to serial " << port << " at baud rate " << baudrate <<endl;
    if(srl_.startSerialCom(port,baudrate)){

        vector<unsigned char> params;
        com_thread_ = thread(&SerialComTlt::comLoop,&srl_); //  RC thread
        join_states_thread_ = thread(&TltNode::publishJoinStates,this); //  RC thread

        if(srl_.startRs232Com()){        // Com started
            cout << "TltNode::TltNode - Remote function activation : Done " << endl;
        }
        else{
            cout << "TltNode::TltNode - Remote function activation : Fail! " << endl;
            abort();
        }
    }
    else{
        cout << "TltNode::TltNode - Serial Com : Fail! " << endl;
        abort();
    }
}

TltNode::~TltNode(){
    srl_.stopRs232Com();    //Com stopped
    srl_.run_= false;       // stop RC thread loop
    com_thread_.join();
    join_states_thread_.join();
}


void TltNode::publishJoinStates(){
    sensor_msgs::JointState joint_states;
    ros::Rate rateController = ros::Rate(20);
    while(ros::ok()) {
        joint_states.header.frame_id = "";
        joint_states.header.stamp = ros::Time::now();
        joint_states.name.resize(4);
        joint_states.position.resize(4);
        joint_states.name[0] = "ewellix_lift_top_joint";
        joint_states.position[0] = srl_.current_pose_;
        joint_states.name[1] = "mot1";
        joint_states.position[1] = srl_.mot1_pose_;
        joint_states.name[2] = "mot2";
        joint_states.position[2] = srl_.mot2_pose_;
        joint_states.name[3] = "mot1_mot2";
        joint_states.position[3] = srl_.mot1_pose_ + srl_.mot2_pose_;
        pub_column_pose_.publish(joint_states);
        rateController.sleep();
    }
}

void TltNode::cbJointTrajReceived(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    // Trajectory Points to List of Heights
    for (unsigned int i = 0; i < goal_handle.getGoal()->trajectory.points.size(); i++)
    {
        float error = 0.005;
        const auto traj_point = goal_handle.getGoal()->trajectory.points.at(i);
        srl_.setColumnSize(traj_point.positions[0]);
        while(srl_.getColumnSize() > traj_point.positions[0] - error && srl_.getColumnSize() < traj_point.positions[0] + error)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

void TltNode::cbJointTrajPreempted(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    srl_.stop();
}

void TltNode::cbColumnSize( std_msgs::Float32 msg){
    srl_.current_target_ = msg.data;
    cout << "TltNode::cbColumnSize - Set current target: " << msg.data << endl;
}

void TltNode::cbDurationUp( std_msgs::Int16 msg){
   srl_.moveUp(msg.data);
}

void TltNode::cbDurationDown( std_msgs::Int16 msg){
    srl_.moveDown(msg.data);
}

void TltNode::cbMotor1Ticks(std_msgs::Int32 msg){
    srl_.moveMot1(msg.data);
}

void TltNode::cbMotor2Ticks(std_msgs::Int32 msg){
    srl_.moveMot2(msg.data);
}

void TltNode::cbJoy( sensor_msgs::Joy msg){

    srl_.go_up_ = msg.buttons[13];
    srl_.go_down_ = msg.buttons[14];

}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "tlt_node");
    ros::NodeHandle nh, private_nh("~");

    TltNode n(private_nh);

    ros::spin();
    return 0;
}
