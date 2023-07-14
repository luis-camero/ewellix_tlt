#ifndef TLTCOM_H
#define TLTCOM_H

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>
#include "serial/serial.h"

using namespace std;

const int TICK_ERROR_MARGIN = 2;

const int MOTOR1_TICKS = 850;
const int MOTOR1_TICK_OFFSET = 10;
const float MOTOR1_METERS = 0.265;
const float MOTOR1_METERS_TO_TICKS = MOTOR1_TICKS / MOTOR1_METERS;
const float MOTOR1_TICKS_TO_METERS = MOTOR1_METERS / MOTOR1_TICKS;

const int MOTOR2_TICKS = 850;
const int MOTOR2_TICK_OFFSET = 10;
const float MOTOR2_METERS = 0.265;
const float MOTOR2_METERS_TO_TICKS = MOTOR2_TICKS / MOTOR2_METERS;
const float MOTOR2_TICKS_TO_METERS = MOTOR2_METERS / MOTOR2_TICKS;

const float ALL_MOTOR_METERS = MOTOR1_METERS + MOTOR2_METERS;
const float ALL_MOTOR_TICKS = MOTOR1_TICKS + MOTOR2_TICKS;

const float MOTOR1_METER_RATIO = MOTOR1_METERS / (ALL_MOTOR_METERS);
const float MOTOR2_METER_RATIO = MOTOR2_METERS / (ALL_MOTOR_METERS);

/*
Class representing Serial communication with the TLT Columns
*/
class SerialComTlt
{
    public:
        SerialComTlt();
        ~SerialComTlt();

        bool startSerialCom(string port, int baud_rate);

        bool startRs232Com();
        bool stopRs232Com();

        void moveUp(int s);
        void moveDown(int s);
        void moveUp();
        void moveDown();
        void stop();

        void comLoop();

        bool run_;
        float current_target_;
        float current_pose_;
        bool go_up_;
        bool go_down_;

        void moveMot1(int);
        void moveMot2(int);

        int margin_ = 1;

        // High Level Commands
        void set_position(float position);
        float get_position();

        // States
        enum class State {
            INIT,
            CALIB,
            IDLE,
            MOTION,
            COMPLETE,
            FAILURE,
        };
        State state_;
        State next_state_;
        State initState();
        State idleState();

        // Calibration State
        enum class CalibSubState {
            INIT,
            RETRACT,
            EXTEND_LOWER,
            EXTEND_UPPER,
            RETRACT_UPPER,
            RETRACT_LOWER,
        };
        unsigned int calib_start_ticks_;
        unsigned int calib_end_ticks_;
        chrono::steady_clock::time_point calib_start_time_;
        chrono::steady_clock::time_point calib_end_time_;
        CalibSubState calib_sub_state_;
        CalibSubState next_calib_sub_state_;
        State calibState();
        State motionState();
        State failureState();

    private:

        serial::Serial serial_tlt_;
        bool debug_;
        bool stop_loop_;
        bool com_started_; // req
        //int mot1_pose_;JTA_SERVER_H
        //int mot2_pose_;
        float last_target_;
        bool process_target_;
        bool manual_target_;
        mutex lock_;


        vector<unsigned char> intToBytes(unsigned int paramInt);
        bool sendCmd(string, vector<unsigned char>);
        vector<unsigned char> feedback(vector<unsigned char>);
        unsigned short calculateChecksum (vector<unsigned char>*);
        bool checkResponseChecksum(vector<unsigned char>*);
        bool checkResponseAck(vector<unsigned char>*);

        // Motors control with pose
        void moveMotAll(int, int);


        /*
        Motor Encoder Pose:
         - in encoder ticks.
         - range: 10 - 850 ticks
        */
        // Variables
        unsigned int mot1_ticks_;
        unsigned int mot2_ticks_;
        unsigned int mot1_ticks_goal_;
        unsigned int mot2_ticks_goal_;
        // Commands: Get Pose (RG)
        const std::vector<unsigned char> GET_POSE_M1 = {0x11, 0x00};
        const std::vector<unsigned char> GET_POSE_M2 = {0x12, 0x00};
        bool extractPose(vector<unsigned char>, int);
        void getPoseM1();
        void getPoseM2();
        // Commands: Set Pose (RT)
        const std::vector<unsigned char> SET_POSE_M1 = {0x06, 0x00, 0x21, 0x30};
        const std::vector<unsigned char> SET_POSE_M2 = {0x06, 0x00, 0x22, 0x30};
        void setPoseM1(unsigned int pose);
        void setPoseM2(unsigned int pose);
        // Checks
        bool isRetracted();
        bool isExtended();


        /*
        Motor Percentage Speed
         - in percentage.
         - range: 0 - 100 percent
        */
        // Variables
        unsigned int mot1_percent_speed_;
        unsigned int mot2_percent_speed_;
        // Commands: Get Speed (RG)
        const std::vector<unsigned char> GET_SPEED_M1 = {0x11, 0X30};
        const std::vector<unsigned char> GET_SPEED_M2 = {0x12, 0X30};
        // const std::vector<unsigned char> GET_SPEED_M1 = {0xF1, 0X00};
        // const std::vector<unsigned char> GET_SPEED_M2 = {0xF2, 0X00};
        bool extractPercentSpeed(vector<unsigned char>, int);
        void getPercentSpeedM1();
        void getPercentSpeedM2();
        // Commands: Set Speed (RT)
        const std::vector<unsigned char> SET_SPEED_M1 = {0x04, 0x00, 0x11, 0x30};
        const std::vector<unsigned char> SET_SPEED_M2 = {0x04, 0x00, 0x12, 0x30};
        void setPercentSpeedM1(unsigned int percent);
        void setPercentSpeedM2(unsigned int percent);

        /*
        Motor Status
         - three boolean variables
         - bit0: initialized
         - bit1: retracting
         - bit2: extending
        */
        // Variables
        bool mot1_initialized_;
        bool mot1_retracting_;
        bool mot1_extending_;
        bool mot2_initialized_;
        bool mot2_retracting_;
        bool mot2_extending_;
        // Commands: Get Status (RG)
        const std::vector<unsigned char> GET_STATUS_M1 = {0xE1, 0X00};
        const std::vector<unsigned char> GET_STATUS_M2 = {0xE2, 0X00};
        bool extractStatus(vector<unsigned char>, int);
        void getStatusM1();
        void getStatusM2();

        /*
        Trigger Motor Movement or Stop
        */
        // Commands: Move Motors (RE)
        const std::vector<unsigned char> MOVE_M1_DOWN = {0x00, 0x01, 0xff};
        const std::vector<unsigned char> MOVE_M1_POSE = {0x00, 0x09, 0xff};
        const std::vector<unsigned char> MOVE_M1_UP = {0x00, 0x02, 0xff};
        void moveM1Down();
        void moveM1Pose();
        void moveM1Up();

        const std::vector<unsigned char> MOVE_M2_DOWN = {0x01, 0x01, 0xff};
        const std::vector<unsigned char> MOVE_M2_POSE = {0x01, 0x09, 0xff};
        const std::vector<unsigned char> MOVE_M2_UP = {0x01, 0x02, 0xff};
        void moveM2Down();
        void moveM2Pose();
        void moveM2Up();

        const std::vector<unsigned char> MOVE_ALL_DOWN = {0x07, 0x01, 0xff};
        const std::vector<unsigned char> MOVE_ALL_POSE = {0x07, 0x09, 0xff};
        const std::vector<unsigned char> MOVE_ALL_UP = {0x07, 0x02, 0xff};
        void moveAllDown();
        void moveAllPose();
        void moveAllUp();

        // Commands: Stop Motors (RS)
        const std::vector<unsigned char> STOP_M1_FAST = {0x00, 0x00};
        const std::vector<unsigned char> STOP_M1_SLOW = {0x00, 0x01};
        const std::vector<unsigned char> STOP_M2_FAST = {0x01, 0x00};
        const std::vector<unsigned char> STOP_M2_SLOW = {0x01, 0x01};
        const std::vector<unsigned char> STOP_ALL_FAST = {0x07, 0x00};
        const std::vector<unsigned char> STOP_ALL_SLOW = {0x07, 0x01};
        void stopM1();
        void stopM2();
        void stopAll();


        // For the Checksum
        unsigned short CRC_TABLE[256] = {
            0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
            0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
            0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
            0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
            0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
            0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
            0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
            0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
            0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
            0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
            0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
            0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
            0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
            0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
            0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
            0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
            0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
            0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
            0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
            0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
            0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
            0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
            0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
            0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
            0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
            0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
            0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
            0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
            0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
            0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
            0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
            0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
        };
};

#endif //TLTCOM_H
