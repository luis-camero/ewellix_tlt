#include "ewellix_tlt/serial_com_tlt.h"


SerialComTlt::SerialComTlt(){
    run_ = true;
    debug_ = false;
    go_up_ = false;
    go_down_ = false;
    stop_loop_ = false;
    com_started_ = false;
    process_target_ = false;
    manual_target_ = false;
    mot1_pose_ = 0;
    mot2_pose_ = 0;
    mot_ticks_ = 0;

    last_target_ = 0.0;
    current_pose_ = 0.0;
    current_target_ = 0.0;
}


SerialComTlt::~SerialComTlt(){
    stop();
    com_started_=false;
    stop_loop_ = true;
    run_= false;
    if(serial_tlt_.isOpen()){
        serial_tlt_.close();
        cout << "SerialComTlt::~SerialComTlt - COM CLOSED !" << endl;
    }
}

bool SerialComTlt::startSerialCom(string port, int baud_rate){

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

    serial_tlt_.setPort(port);
    serial_tlt_.setBaudrate(baud_rate);
    serial_tlt_.setTimeout(timeout);

    try{
        serial_tlt_.open();
        cout << "SerialComTlt::startSerialCom - COM OPEN !" << endl;
        return true;
    }
    catch (serial::IOException e){
        cout << "SerialComTlt::startSerialCom - serial::IOException: " << e.what() << endl;
        return false;
    }

}

/*
* Activate the remote function
*/
bool SerialComTlt::startRs232Com(){

    vector<unsigned char> params = {0x01};
    sendCmd("RO",params);
    bool result = sendCmd("RO",params); // double call to wake up the column after long delay without com
    if(result){
        com_started_=true;
        stop_loop_=true;
        cout << "SerialComTlt::startRs232Com - Remote function activated" << endl;
        return true;
    }
    else{
        return false;
    }

}

/*
* Deactivate the remote function
*/
bool SerialComTlt::stopRs232Com(){
    com_started_=false;
    usleep(100);
    vector<unsigned char> params = {};
    bool result = sendCmd("RA",params);

    if(result){
        cout << "SerialComTlt::stopRs232Com - Remote function deactivated" << endl;
        return true;
    }
    else{
        return false;
    }
}
/*
* Move both with input pose (ntick)
*/
void SerialComTlt::moveMotAll(int mot1_pose, int mot2_pose){
    vector<unsigned char> params;
    vector<unsigned char> pose_hex;
    unsigned char a;
    unsigned char b;
    unsigned char c;
    unsigned char d;


    // Motor1
    pose_hex = intToBytes(mot1_pose);
    a = *(pose_hex.end()-4);
    b = *(pose_hex.end()-3);
    c = *(pose_hex.end()-2);
    d = *(pose_hex.end()-1);

    params = {0x06,0x00,0x21,0x30,d,c,b,a};
    sendCmd("RT",params);      // set pose
    params = {0x04,0x00,0x11,0x30,0x64,0x00};
    sendCmd("RT",params);      //set speed 100%

    // Motor2
    pose_hex = intToBytes(mot2_pose);
    a = *(pose_hex.end()-4);
    b = *(pose_hex.end()-3);
    c = *(pose_hex.end()-2);
    d = *(pose_hex.end()-1);

    params = {0x06,0x00,0x22,0x30,d,c,b,a};
    sendCmd("RT",params);      // set pose
    params = {0x04, 0x00,0x12,0x30,0x64,0x00};
    sendCmd("RT",params);      //set speed 100%

    // Move All Motors
    params = {0x07,0x09,0xff};
    sendCmd("RE",params);
}

/*
* Move Mot1 with input pose (ntick)
*/
void SerialComTlt::moveMot1(int pose){
    mot1_pose_target_ = pose;

    vector<unsigned char> pose_hex = intToBytes(pose);
    unsigned char a = *(pose_hex.end()-4);
    unsigned char b = *(pose_hex.end()-3);
    unsigned char c = *(pose_hex.end()-2);
    unsigned char d = *(pose_hex.end()-1);

    // P1, P2: 0x06, 0x00: 6 Byte Message
    // P3, P4: 0x21, 0x30: 3021 : Remote Position A1
    // P5, P6, P7, P8: Position in Big Endian
    vector<unsigned char> params = {0x06,0x00,0x21,0x30,d,c,b,a};

    sendCmd("RT",params);      // set pose

    params = {0x04,0x00,0x11,0x30,0x64,0x00};
    sendCmd("RT",params);      //set speed 100%

    params = {0x00,0x09,0xff};
    sendCmd("RE",params);      // move
}

/*
* Move Mot2 with input pose (ntick)
*/
void SerialComTlt::moveMot2(int pose){
    mot2_pose_target_ = pose;

    vector<unsigned char> pose_hex = intToBytes(pose);
    unsigned char a = *(pose_hex.end()-4);
    unsigned char b = *(pose_hex.end()-3);
    unsigned char c = *(pose_hex.end()-2);
    unsigned char d = *(pose_hex.end()-1);

    vector<unsigned char> params = {0x06, 0x00,0x22,0x30,d,c,b,a};

    sendCmd("RT",params);      // set pose
    params = {0x04,0x00,0x12,0x30,0x64,0x00};
    sendCmd("RT",params);      //set speed 100%

    params = {0x01,0x09,0xff};
    sendCmd("RE",params);      // move

}

/*
* Control column size
*/
void SerialComTlt::setColumnSize(float m){
    // Check Target is in Bounds
    if(m > ALL_MOTOR_METERS) m = ALL_MOTOR_METERS;
    if(m < 0) m = 0;

    // Check if Target is Current
    if(getColumnSize() == m){
      return;
    }

    // Convert Meters to Ticks
    float mot1_meters = MOTOR1_METER_RATIO * m;
    float mot2_meters = MOTOR2_METER_RATIO * m;

    int mot1_ticks = static_cast<int>(mot1_meters * MOTOR1_METERS_TO_TICKS);
    int mot2_ticks = static_cast<int>(mot2_meters * MOTOR2_METERS_TO_TICKS);

    if(mot1_ticks > MOTOR1_TICKS) mot1_ticks = MOTOR1_TICKS;
    if(mot2_ticks > MOTOR2_TICKS) mot2_ticks = MOTOR2_TICKS;

    cout << "SerialComTlt:setColumnSize - ";
    cout << "size: " << m << " ";
    cout << "mot1_ticks: " << mot1_ticks << " ";
    cout << "mot2_ticks: " << mot2_ticks << " ";
    cout << endl;
    moveMotAll(mot1_ticks + MOTOR1_TICK_OFFSET, mot2_ticks + MOTOR2_TICK_OFFSET);
}

/*
*  Move up both motors
*/
void SerialComTlt::moveUp(){
    moveM1Up();
    moveM2Up();
}

/*
*  Move down both motors
*/
void SerialComTlt::moveDown(){
    moveM1Down();
    moveM2Down();

}
/*
*  Move up both motors during n seconds
*/
void SerialComTlt::moveUp(int n){
    moveM1Up();
    moveM2Up();
    sleep(n);
    stop();
}

/*
*  Move down both motors during n seconds
*/
void SerialComTlt::moveDown(int n){
    moveM1Down();
    moveM2Down();
    sleep(n);
    stop();
}

void SerialComTlt::stop(){
    stopAll();
}

/*
Get Commands: 
 - pose: current tick position
 - speed: current speed as percentage
 - status: initialized, retracting, extending
*/
void SerialComTlt::getPoseM1(){
    sendCmd("RG",GET_POSE_M1);
    if (debug_) cout << "SerialComTlt::getPoseM1: " << mot1_pose_ << endl;
}
void SerialComTlt::getPoseM2(){
    sendCmd("RG",GET_POSE_M2);
    if (debug_) cout << "SerialComTlt::getPoseM2: " << mot2_pose_ << endl;
}
void SerialComTlt::getPercentSpeedM1(){
    sendCmd("RG",GET_SPEED_M1);
    if (debug_) cout << "SerialComTlt::getPercentSpeedM1: " << mot1_percent_speed_ << endl;
}
void SerialComTlt::getPercentSpeedM2(){
    sendCmd("RG",GET_SPEED_M2);
    if (debug_) cout << "SerialComTlt::getPercentSpeedM2: " << mot2_percent_speed_ << endl;
}
void SerialComTlt::getStatusM1(){
    sendCmd("RG",GET_STATUS_M1);
    if (debug_){
        cout << "SerialComTlt::getStatusM1: ";
        cout << "Initialized=" << mot1_initialized_;
        cout << "Retracting=" << mot1_retracting_;
        cout << "Extending=" << mot1_extending_;
        cout << endl;
    }
}
void SerialComTlt::getStatusM2(){
    sendCmd("RG",GET_STATUS_M2);
    if (debug_){
        cout << "SerialComTlt::getStatusM2: ";
        cout << "Initialized=" << mot2_initialized_;
        cout << "Retracting=" << mot2_retracting_;
        cout << "Extending=" << mot2_extending_;
        cout << endl;
    }
}
/*
Set Commands
 - pose: target tick position
 - speed: target speed as percentage
 - 
*/
void SerialComTlt::setPoseM1(unsigned int pose){
    std::vector<unsigned char> params;
    // Flip Integer
    std::vector<unsigned char> pose_hex = intToBytes(pose);
    // Append Command and Pose
    params.insert(params.end(), SET_POSE_M1.begin(), SET_POSE_M1.end());
    params.insert(params.end(), pose_hex.begin(), pose_hex.end());
    sendCmd("RT", params);
}
void SerialComTlt::setPoseM2(unsigned int pose){
    std::vector<unsigned char> params;
    // Flip Integer
    std::vector<unsigned char> pose_hex = intToBytes(pose);
    // Append Command and Pose
    params.insert(params.end(), SET_POSE_M2.begin(), SET_POSE_M2.end());
    params.insert(params.end(), pose_hex.begin(), pose_hex.end());
    sendCmd("RT", params);
}
void SerialComTlt::setPercentSpeedM1(unsigned int percent){
    if(percent > 100) percent = 100;
    if(percent < 0) percent = 0;
    std::vector<unsigned char> params;
    std::vector<unsigned char> percent_hex = intToBytes(percent);
    // Append Command and Percent Speed
    params.insert(params.end(), SET_SPEED_M1.begin(), SET_SPEED_M1.end());
    params.insert(params.end(), percent_hex.begin(), percent_hex.begin() + 2);
    sendCmd("RT",params);
}
void SerialComTlt::setPercentSpeedM2(unsigned int percent){
    if(percent > 100) percent = 100;
    if(percent < 0) percent = 0;
    std::vector<unsigned char> params;
    std::vector<unsigned char> percent_hex = intToBytes(percent);
    // Append Command and Percent Speed
    params.insert(params.end(), SET_SPEED_M2.begin(), SET_SPEED_M2.end());
    params.insert(params.end(), percent_hex.begin(), percent_hex.begin() + 2);
    sendCmd("RT",params);
}

/*
Move Commands
 - down: retract until stopped
 - up: extend until stopped
 - pose: extract or extend to match set pose
*/
void SerialComTlt::moveM1Down(){
    sendCmd("RE",MOVE_M1_DOWN);
}
void SerialComTlt::moveM1Up(){
    sendCmd("RE",MOVE_M1_UP);
}
void SerialComTlt::moveM1Pose(){
    sendCmd("RE",MOVE_M1_POSE);
}
void SerialComTlt::moveM2Down(){
    sendCmd("RE",MOVE_M2_DOWN);
}
void SerialComTlt::moveM2Up(){
    sendCmd("RE",MOVE_M2_UP);
}
void SerialComTlt::moveM2Pose(){
    sendCmd("RE",MOVE_M2_POSE);
}
void SerialComTlt::moveAllDown(){
    sendCmd("RE",MOVE_ALL_DOWN);
}
void SerialComTlt::moveAllUp(){
    sendCmd("RE",MOVE_ALL_UP);
}
void SerialComTlt::moveAllPose(){
    sendCmd("RE",MOVE_ALL_POSE);
}

/*
Stop Commands:
*/
void SerialComTlt::stopM1(){
    sendCmd("RS",STOP_M1_FAST);
    if (debug_) cout << "SerialComTlt:stopMot1" << endl;
}
void SerialComTlt::stopM2(){
    sendCmd("RS",STOP_M2_FAST);
    if (debug_) cout << "SerialComTlt:stopMot2" << endl;
}
void SerialComTlt::stopAll(){
    sendCmd("RS",STOP_ALL_FAST);
    if (debug_)cout << "SerialComTlt::stopMotAll" << endl;
}

float SerialComTlt::getColumnSize(){
    getPoseM1();
    getPoseM2();

    int mot1_norm_pose = mot1_pose_ - MOTOR1_TICK_OFFSET;
    if(mot1_norm_pose < 0) mot1_norm_pose = 0;

    int mot2_norm_pose = mot2_pose_ - MOTOR2_TICK_OFFSET;
    if(mot2_norm_pose < 0) mot2_norm_pose = 0;

    float mot1_meters = static_cast<float>(mot1_norm_pose * MOTOR1_TICKS_TO_METERS);
    float mot2_meters = static_cast<float>(mot2_norm_pose * MOTOR2_TICKS_TO_METERS);

    current_pose_ = mot1_meters + mot2_meters;
    return current_pose_;
}

/*
Feedback and Checksum
*/

// compute checksum
unsigned short SerialComTlt::calculateChecksum(vector<unsigned char> *cmd){
    unsigned short crc = 0;
    for(vector<unsigned char>::iterator it=cmd->begin(); it!=cmd->end(); ++it){
        crc = static_cast<unsigned short>(CRC_TABLE[((crc >> 8) ^ *it) & 0xFF] ^ (crc << 8));
    }
    return crc;
}

// compare response checksum with calculated
bool SerialComTlt::checkResponseChecksum(vector<unsigned char> *response){
    unsigned short response_checksum_lsb = *(response->end()-2);
    unsigned short response_checksum_msb = *(response->end()-1);

    if (debug_){
        unsigned short checksum = (response_checksum_lsb<<8) | response_checksum_msb;
        stringstream checksum_hex;
        checksum_hex << hex << checksum;
        cout << "SerialComTlt::checkResponseChecksum - Response Checksum = "<< checksum_hex.str() << endl;
    }

    vector<unsigned char> response_msg = *response;
    response_msg.resize(response_msg.size()-2);

    unsigned short checksum = calculateChecksum(&response_msg);
    unsigned short computed_lsb = checksum &0x00FF;
    unsigned short computed_msb = checksum >> 8;

    if (debug_){
        unsigned short checksum2 = (computed_lsb<<8) | computed_msb;
        stringstream checksum_hex;
        checksum_hex << hex << checksum2;
        cout << "SerialComTlt::checkResponseChecksum - Computed Checksum = "<< checksum_hex.str() << endl;
    }

    if(response_checksum_lsb == computed_lsb && response_checksum_msb == computed_msb){
        return true;
    }
    else{
        return false;
    }

    return false;
}

bool SerialComTlt::checkResponseAck(vector<unsigned char> *response){
    if(response->size()>4){
        unsigned short cmd_status = *(response->begin()+2);
        if(cmd_status == 0x06 ){
            return true;
        }

        else if (cmd_status == 0x81 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 81 Parameter data error "<< endl;
        }
        else if (cmd_status == 0x82 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 82 Parameter count error "<< endl;
        }
        else if (cmd_status == 0x83 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 83 Command error "<< endl;
        }
        else if (cmd_status == 0x84 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 84 Permission error "<< endl;
        }
    }
    else{
        return false;
    }
    return false;
}

vector<unsigned char> SerialComTlt::feedback(){
    int i = 0;
    int timeout =0;
    vector<unsigned char> received_data;
    string last_data;
    string command_type="";
    int msg_size = -1;
    bool success = false;
    bool first_byte = false;

    while (!stop_loop_)
    {
        // Check if serial available
        if (serial_tlt_.available()){

            last_data = serial_tlt_.read();

            if(!first_byte && last_data =="R"){  // Beginning of the message
                first_byte = true;
                for (const auto &item : last_data) {
                    received_data.push_back(int(item));  // convert cmd string in hex value
                }
            }
            if( first_byte){
                i++;
                if(i>1) {
                    for (const auto &item : last_data) {
                        received_data.push_back(int(item));  // convert cmd string in hex value
                    }
                }

                // command detector
                if(i==2){
                    command_type = last_data;
                    if (debug_) cout << "Response Command type : R"<< command_type << endl;
                }

                // success detector
                if( i==3 && last_data==""){ // ACK
                    success = true;
                }


                if(command_type =="G")  {
                    if (i == 4 && success) {

                        for (const auto &item : last_data) {
                            msg_size = 7 + int(item);
                        }
                    }
                    else if(i == 4 && !success) {
                        msg_size = 5;
                    }
                }
                else if(command_type =="T" ||command_type =="C"||command_type =="E"||command_type =="S"||command_type =="O"||command_type =="A")  {
                    msg_size = 5;
                }
                else{
                    msg_size = 5;
                }

                if(msg_size > 0 && i == msg_size){
                    stop_loop_ = true;
                    break;
                }
            }
        }
        else{
            usleep(1);
            timeout++;
            if( timeout > 5000){
                stop_loop_ = true;
                return received_data;
            }
        }
    }
    return received_data;
}


/*
Extract:
 - pose
 - speed
 - status
*/
bool SerialComTlt::extractPose(vector<unsigned char>* response, int mot){
    int position = (unsigned char)(*(response->end()-5)) << 8 | (unsigned char)(*(response->end() - 6));

    if(mot == 1) mot1_pose_ = position;
    else if(mot == 2) mot2_pose_ = position;
    else return false;

    return true;
}
bool SerialComTlt::extractPercentSpeed(vector<unsigned char>* response, int mot){
    int speed = (unsigned char)(*(response->end()-5)) << 8 | (unsigned char)(*(response->end() - 6));
    
    if(mot == 1) mot1_percent_speed_ = speed;
    else if(mot == 2) mot2_percent_speed_ = speed;
    else return false;
    
    return true;
}
bool SerialComTlt::extractStatus(vector<unsigned char>* response, int mot){
    unsigned char status = *(response->end()-6);

    if(mot == 1){
        mot1_initialized_ = bool(status | 0x01);
        mot1_retracting_ = bool(status | 0x02);
        mot1_extending_ = bool(status | 0x04);
    }
    else if(mot == 2){
        mot2_initialized_ = bool(status | 0x01);
        mot2_retracting_ = bool(status | 0x02);
        mot2_extending_ = bool(status | 0x04);
    }
    else{
        return false;
    }
    return true;
}

/*
* Converter
*/
vector<unsigned char> SerialComTlt::intToBytes(unsigned int paramInt){
    vector<unsigned char> arrayOfByte(4);
    for (int i = 0; i < 4; i++)
        arrayOfByte[i] = static_cast<unsigned char>(paramInt >> (i * 8));
    return arrayOfByte;
}

/*
Send Command:
 - convert command to bytes
 - compute checksum
 - write to serial com
 - extract results 
*/
bool SerialComTlt::sendCmd(string cmd, vector<unsigned char> param){
    // Lock
    lock_.lock();

    // Convert String Command to Hex
    vector<unsigned char> final_cmd;
    if (debug_) cout <<"----------------------"<< endl << "Input Cmd:  ";
    for (const auto &item : cmd) {
        if (debug_) cout << item;
        final_cmd.push_back(int(item));  // convert cmd string in hex value
    }

    // Add Parameters to Command
    if (debug_) cout <<" [";
    if (!param.empty()){
        for(vector<unsigned char>::iterator it=param.begin(); it!=param.end(); ++it){
            if (debug_) cout <<' ' <<  int(*it);
            final_cmd.push_back(*it);       // add params to the cmd
        }
    }
    if (debug_) cout <<" ]"<< endl;
    
    // Compute checksum
    unsigned short checksum = calculateChecksum(&final_cmd);
    unsigned short lsb = checksum &0x00FF;
    unsigned short msb = checksum >> 8;

    final_cmd.push_back(lsb);
    final_cmd.push_back(msb);

    if (debug_){
        stringstream final_cmd_hex;
        for (unsigned short j: final_cmd){
            final_cmd_hex <<  hex << j << ' ';
        }
        cout << "SerialComTlt::sendCmd - Output Cmd: " << final_cmd_hex.str()<< endl;
    }

    // Send Command
    if ( serial_tlt_.isOpen() ){
        try{
            serial_tlt_.write(final_cmd);
            usleep( 1 );
            serial_tlt_.flush();
            stop_loop_= false;
        }
        catch (serial::IOException e){
            cout << "SerialComTlt::sendCmd - serial::IOException: " << e.what() << endl;
        }
    }
    usleep(10); // wait for response

    // Receive Response
    vector<unsigned char> output = feedback();
     if(output.size() == 0){
        cout << "SerialComTlt::sendCmd - Response empty" << endl;
        lock_.unlock();
        return false;
    }
    if (debug_){
        stringstream output_hex;
        for (unsigned short j: output){
            output_hex <<  hex << j << ' ';
        }
        cout << "SerialComTlt::sendCmd - Response: " << output_hex.str()<< endl;
    }

    // Checksum Failed, Try Again
    if(!checkResponseChecksum(&output) || !checkResponseAck(&output)){
        cout << "SerialComTlt::sendCmd - Cmd failed, retrying..." << endl;
        serial_tlt_.flush();
        usleep(300);
        serial_tlt_.write(final_cmd);

        output = feedback();
        if (debug_){
            stringstream output_hex;
            for (unsigned short j: output){
                output_hex <<  hex << j << ' ';
            }
            cout << "SerialComTlt::sendCmd - Response: " << output_hex.str()<< endl;
        }
        if(output.size() == 0 || !checkResponseChecksum(&output) || !checkResponseAck(&output)){
            cout << "SerialComTlt::sendCmd - Command FAILED !!!" << endl;
            lock_.unlock();
            startRs232Com();
            return false;
        }
    }

    // Extract Information
    if(cmd =="RG"){
        // get pose
        if(param == GET_POSE_M1) extractPose(&output,1);
        else if (param == GET_POSE_M2) extractPose(&output,2);
        // get speed
        else if (param == GET_SPEED_M1) extractPercentSpeed(&output,1);
        else if (param == GET_SPEED_M2) extractPercentSpeed(&output,2);
        // get status
        else if (param == GET_STATUS_M1) extractPercentSpeed(&output,1);
        else if (param == GET_STATUS_M2) extractPercentSpeed(&output,2);
    }
    if (debug_) cout <<"----------------------" << endl;

    // Unlock
    lock_.unlock();
    return true;
}

SerialComTlt::State SerialComTlt::initState(){
    return SerialComTlt::State::INIT;
}

SerialComTlt::State SerialComTlt::idleState(){
    return SerialComTlt::State::IDLE;
}

SerialComTlt::State SerialComTlt::motionState(){
    return SerialComTlt::State::MOTION;
}

SerialComTlt::State SerialComTlt::failureState(){
    return SerialComTlt::State::FAILURE;
}


/*
 * Loop to maintain the remote function with RC command
*/
void SerialComTlt::comLoop(){
    vector<unsigned char> params = {0x01, 0x00, 0xff};
    while(run_){
        while(com_started_){
            // Start/Maintain Communication
            sendCmd("RC",params);

            // Update Data
            getPoseM1();
            getPoseM2();
            getPercentSpeedM1();
            getPercentSpeedM2();
            getStatusM1();
            getStatusM2();

            // State Machine
            switch(state_)
            {
                case SerialComTlt::State::INIT:
                    next_state_ = initState();
                case SerialComTlt::State::IDLE:
                    next_state_ = idleState();
                case SerialComTlt::State::MOTION:
                    next_state_ = motionState();
                case SerialComTlt::State::FAILURE:
                    next_state_ = failureState();
            }
            state_ = next_state_;


            // if(!manual_target_ && (go_up_ || go_down_)){
            //     manual_target_ = true;
            //     if(go_up_) moveUp();
            //     else moveDown();

            // }
            // else{
            //     getColumnSize();
            //     if(current_target_ !=last_target_){
            //         if (process_target_) stop();
            //         setColumnSize(current_target_);
            //         last_target_ = current_target_;
            //         process_target_= true;
            // cout << "SerialComTtl::comLoop - start processing target" << endl;
            //     }

            //     if(process_target_ && (mot1_pose_ ==  mot_ticks_)){
            //     stop();
            // cout << "SerialComTlt::comLoop - stop processing target" << endl;
            //         process_target_= false;
            //     }

            // if(((mot1_pose_target_ + margin_) >= mot1_pose_) && ((mot1_pose_target_ - margin_) <= mot1_pose_)){
            //     stopM1();
            // }
            // if(((mot2_pose_target_ + margin_) >= mot2_pose_) && ((mot2_pose_target_ - margin_) <= mot2_pose_)){
            //     stopM2();
            // }

            // }
            // if(manual_target_ && !go_up_ && !go_down_){
            //     stop();
            //     manual_target_= false;
            // }

            usleep(10);
        }
        usleep(1);
    }
}

