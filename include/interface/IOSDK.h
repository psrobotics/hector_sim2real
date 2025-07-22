#ifndef IOSDK_H
#define IOSDK_H

#include "IOInterface.h"
#include "../sdk/include/unitree_legged_sdk.h"
// #include "nanopb/pb_encode.h"
// #include "nanopb/pb_decode.h"
#include <fstream>
#include "../Custom_SDK/include/CustomCommunication.h"

// extern "C"{
//     #include "Full_encode.pb.h"
// }

using namespace UNITREE_LEGGED_SDK;

class IOSDK : public IOInterface{
public:
IOSDK(LeggedType robot, int cmd_panel_id);
~IOSDK(){}
// void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
void sendUDP(const LowlevelCmd *cmd);
void ReceiveUDP(LowlevelState *state);

// std::ofstream tau_command;
// std::ofstream kp_command;
// std::ofstream kd_command;

UDP _udp;
Safety _control;
LowCmd _lowCmd = {0};
LowState _lowState = {0};
LowLevelCmd message_to_encode;
CustomCommunication customcommunication;
// static int count;
std::chrono::high_resolution_clock::time_point previoustime;

bool bias_calibrated = false;
int sample_count_ = 0;
const int calibration_samples_ = 500;

float gyro_bias_x_ = 0.0f;
float gyro_bias_y_ = 0.0f;
float gyro_bias_z_ = 0.0f;

float acc_sum_x_ = 0.0f;
float acc_sum_y_ = 0.0f;
float acc_sum_z_ = 0.0f;

float pitch_ = 0.0f;
float roll_ = 0.0f;
float yaw_ = 0.0f;

// Sampling time
const float dt_ = 1.0f / 1000.0f;

float avg_acc_x = 0.0f;
float avg_acc_y = 0.0f;
float avg_acc_z = 0.0f;

float corrected_gyro_x = 0.0f;
float corrected_gyro_y = 0.0f;
float corrected_gyro_z = 0.0f;

};

#endif  // IOSDK_H