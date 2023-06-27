#include "ewellix_tlt/tlt_node.h"


TltNode::TltNode(ros::NodeHandle private_nh)
{
    // Publishers
    pub_column_pose_ = private_nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    // Variables
    string port;
    int baudrate;

    // Parameters
    private_nh.param<string>("ewellix/port", port, "/dev/ttyUSB0");
    private_nh.param<int>("ewellix/baudrate", baudrate, 38400);

    // Action Server
    act_srv_single_joint_position_ = new SingleJointPositionActionServer(
        "test_server/single_joint_position",
        private_nh,
        &srl_);

    // Services
    srv_init_sequence_ = private_nh.advertiseService("init_sequence", &TltNode::srvInitSequence, this);

    // Subscribers
    sub_column_size_ = private_nh.subscribe("/ewellix/size", 1, &TltNode::cbColumnSize,this);
    sub_column_duration_up_ = private_nh.subscribe("/ewellix/duration_up", 1, &TltNode::cbDurationUp,this);
    sub_column_duration_down_ = private_nh.subscribe("/ewellix/duration_down", 1, &TltNode::cbDurationDown,this);

    sub_motor1_ticks_ = private_nh.subscribe("/ewellix/motor1_ticks", 1, &TltNode::cbMotor1Ticks, this);
    sub_motor2_ticks_ = private_nh.subscribe("/ewellix/motor2_ticks", 1, &TltNode::cbMotor2Ticks, this);

    sub_joy_ = private_nh.subscribe("/joy", 1, &TltNode::cbJoy,this);

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
        int size = 1;
        joint_states.name.resize(size);
        joint_states.position.resize(size);
        joint_states.velocity.resize(size);
        joint_states.effort.resize(size);
        joint_states.name[0] = "ewellix_lift_top_joint";
        joint_states.position[0] = srl_.current_pose_;
        joint_states.velocity[0] = 0.0;
        joint_states.effort[0] = 0.1;
        //joint_states.name[1] = "mot1";
        //joint_states.position[1] = srl_.mot1_pose_;
        //joint_states.name[2] = "mot2";
        //joint_states.position[2] = srl_.mot2_pose_;
        //joint_states.name[3] = "mot1_mot2";
        //joint_states.position[3] = srl_.mot1_pose_ + srl_.mot2_pose_;
        pub_column_pose_.publish(joint_states);
        rateController.sleep();
    }
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

bool TltNode::srvInitSequence(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res){
    ROS_INFO("srvInitSequence: Move Up 20 Seconds");
    srl_.moveUp(20);
    ROS_INFO("srvInitSequence: Move Down 20 Seconds");
    srl_.moveDown(20);
    res.success = true;
    res.message = "Moved arm to max and min positions.";
    return true;
}

void actSingleJointPosition(const control_msgs::SingleJointPositionGoalConstPtr &goal){
    bool success = true;
    ROS_INFO("executing single joint position");

}



int main(int argc, char *argv[]){

    ros::init(argc, argv, "tlt_node");
    ros::NodeHandle nh, private_nh("~");

    TltNode n(private_nh);

    ros::spin();
    return 0;
}
