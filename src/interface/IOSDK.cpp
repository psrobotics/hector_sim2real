
#include "../../include/interface/IOSDK.h"
#include "../../include/interface/WirelessHandle.h"
#include "../../include/interface/KeyBoard.h"
#include "../../include/sdk/include/unitree_legged_sdk.h"

#include <stdio.h>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;
static int count;
IOSDK::IOSDK(LeggedType robot, int cmd_panel_id) : _control(robot),
                                                   _udp(LOWLEVEL),
                                                   customcommunication(8070),
                                                   imu_filter(1000.0f, 0.05f)
{
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);
    if (cmd_panel_id == 1)
    {
        cmdPanel = new WirelessHandle();
    }
    else if (cmd_panel_id == 2)
    {
        cmdPanel = new KeyBoard();
    }

    count = 0;

}


void IOSDK::sendUDP(const LowlevelCmd *cmd)
{
    // std::cout << "Running Custom Communication" << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(start - previoustime);
    // std::cout << "Time taken bw message is " << duration.count() << "micro s\n";

    previoustime = start;

    message_to_encode = LowLevelCmd_init_zero;

    message_to_encode.arm_motor1.mode = 0;

    // Arm Motor Assigning
    message_to_encode.leg_motor1.mode = 10;
    message_to_encode.leg_motor1.q = cmd->motorCmd[0].q;
    message_to_encode.leg_motor1.dq = cmd->motorCmd[0].dq;
    message_to_encode.leg_motor1.kp = cmd->motorCmd[0].Kp;
    message_to_encode.leg_motor1.kd = cmd->motorCmd[0].Kd;
    message_to_encode.leg_motor1.tau = cmd->motorCmd[0].tau;

    message_to_encode.leg_motor2.mode = 10;
    message_to_encode.leg_motor2.q = cmd->motorCmd[1].q;
    message_to_encode.leg_motor2.dq = cmd->motorCmd[1].dq;
    message_to_encode.leg_motor2.kp = cmd->motorCmd[1].Kp;
    message_to_encode.leg_motor2.kd = cmd->motorCmd[1].Kd;
    message_to_encode.leg_motor2.tau = cmd->motorCmd[1].tau;

    message_to_encode.leg_motor3.mode = 10;
    message_to_encode.leg_motor3.q = cmd->motorCmd[2].q;
    message_to_encode.leg_motor3.dq = cmd->motorCmd[2].dq;
    message_to_encode.leg_motor3.kp = cmd->motorCmd[2].Kp;
    message_to_encode.leg_motor3.kd = cmd->motorCmd[2].Kd;
    message_to_encode.leg_motor3.tau = cmd->motorCmd[2].tau;

    message_to_encode.leg_motor4.mode = 10;
    message_to_encode.leg_motor4.q = cmd->motorCmd[3].q;
    message_to_encode.leg_motor4.dq = cmd->motorCmd[3].dq;
    message_to_encode.leg_motor4.kp = cmd->motorCmd[3].Kp;
    message_to_encode.leg_motor4.kd = cmd->motorCmd[3].Kd;
    message_to_encode.leg_motor4.tau = cmd->motorCmd[3].tau;

    message_to_encode.leg_motor5.mode = 10;
    message_to_encode.leg_motor5.q = cmd->motorCmd[4].q;
    message_to_encode.leg_motor5.dq = cmd->motorCmd[4].dq;
    message_to_encode.leg_motor5.kp = cmd->motorCmd[4].Kp;
    message_to_encode.leg_motor5.kd = cmd->motorCmd[4].Kd;
    message_to_encode.leg_motor5.tau = cmd->motorCmd[4].tau;

    message_to_encode.leg_motor6.mode = 10;
    message_to_encode.leg_motor6.q = cmd->motorCmd[5].q;
    message_to_encode.leg_motor6.dq = cmd->motorCmd[5].dq;
    message_to_encode.leg_motor6.kp = cmd->motorCmd[5].Kp;
    message_to_encode.leg_motor6.kd = cmd->motorCmd[5].Kd;
    message_to_encode.leg_motor6.tau = cmd->motorCmd[5].tau;

    message_to_encode.leg_motor7.mode = 10;
    message_to_encode.leg_motor7.q = cmd->motorCmd[6].q;
    message_to_encode.leg_motor7.dq = cmd->motorCmd[6].dq;
    message_to_encode.leg_motor7.kp = cmd->motorCmd[6].Kp;
    message_to_encode.leg_motor7.kd = cmd->motorCmd[6].Kd;
    message_to_encode.leg_motor7.tau = cmd->motorCmd[6].tau;

    // std::cout << "q command is " << cmd->motorCmd[6].q << std::endl;

    message_to_encode.leg_motor8.mode = 10;
    message_to_encode.leg_motor8.q = cmd->motorCmd[7].q;
    message_to_encode.leg_motor8.dq = cmd->motorCmd[7].dq;
    message_to_encode.leg_motor8.kp = cmd->motorCmd[7].Kp;
    message_to_encode.leg_motor8.kd = cmd->motorCmd[7].Kd;
    message_to_encode.leg_motor8.tau = cmd->motorCmd[7].tau;

    message_to_encode.leg_motor9.mode = 10;
    message_to_encode.leg_motor9.q = cmd->motorCmd[8].q;
    message_to_encode.leg_motor9.dq = cmd->motorCmd[8].dq;
    message_to_encode.leg_motor9.kp = cmd->motorCmd[8].Kp;
    message_to_encode.leg_motor9.kd = cmd->motorCmd[8].Kd;
    message_to_encode.leg_motor9.tau = cmd->motorCmd[8].tau;

    message_to_encode.leg_motor10.mode = 10;
    message_to_encode.leg_motor10.q = cmd->motorCmd[9].q;
    message_to_encode.leg_motor10.dq = cmd->motorCmd[9].dq;
    message_to_encode.leg_motor10.kp = cmd->motorCmd[9].Kp;
    message_to_encode.leg_motor10.kd = cmd->motorCmd[9].Kd;
    message_to_encode.leg_motor10.tau = cmd->motorCmd[9].tau;

    message_to_encode.arm_motor1.q = cmd->motorCmd[10].q;
    message_to_encode.arm_motor1.dq = cmd->motorCmd[10].dq;
    message_to_encode.arm_motor1.kp = cmd->motorCmd[10].Kp;
    message_to_encode.arm_motor1.kd = cmd->motorCmd[10].Kd;
    message_to_encode.arm_motor1.tau = cmd->motorCmd[10].tau;
    // std::cout << "Arm1 kd is " << cmd->motorCmd[11].Kd;

    message_to_encode.arm_motor2.q = cmd->motorCmd[11].q;
    message_to_encode.arm_motor2.dq = cmd->motorCmd[11].dq;
    message_to_encode.arm_motor2.kp = cmd->motorCmd[11].Kp;
    message_to_encode.arm_motor2.kd = cmd->motorCmd[11].Kd;
    message_to_encode.arm_motor2.tau = cmd->motorCmd[11].tau;
    // std::cout << "Arm2 kd is " << cmd->motorCmd[12].Kd;

    message_to_encode.arm_motor3.q = cmd->motorCmd[12].q;
    message_to_encode.arm_motor3.dq = cmd->motorCmd[12].dq;
    message_to_encode.arm_motor3.kp = cmd->motorCmd[12].Kp;
    message_to_encode.arm_motor3.kd = cmd->motorCmd[12].Kd;
    message_to_encode.arm_motor3.tau = cmd->motorCmd[12].tau;

    message_to_encode.arm_motor4.q = cmd->motorCmd[13].q;
    message_to_encode.arm_motor4.dq = cmd->motorCmd[13].dq;
    message_to_encode.arm_motor4.kp = cmd->motorCmd[13].Kp;
    message_to_encode.arm_motor4.kd = cmd->motorCmd[13].Kd;
    message_to_encode.arm_motor4.tau = cmd->motorCmd[13].tau;

    message_to_encode.arm_motor5.q = cmd->motorCmd[14].q;
    message_to_encode.arm_motor5.dq = cmd->motorCmd[14].dq;
    message_to_encode.arm_motor5.kp = cmd->motorCmd[14].Kp;
    message_to_encode.arm_motor5.kd = cmd->motorCmd[14].Kd;
    message_to_encode.arm_motor5.tau = cmd->motorCmd[14].tau;

    message_to_encode.arm_motor6.q = cmd->motorCmd[15].q;
    message_to_encode.arm_motor6.dq = cmd->motorCmd[15].dq;
    message_to_encode.arm_motor6.kp = cmd->motorCmd[15].Kp;
    // std::cout << "arm6 Kp: " << message_to_encode.arm_motor6.kp << std::endl;
    message_to_encode.arm_motor6.kd = cmd->motorCmd[15].Kd;
    // std::cout << "arm6 Kd: " << message_to_encode.arm_motor6.kd << std::endl;
    message_to_encode.arm_motor6.tau = cmd->motorCmd[15].tau;

    message_to_encode.arm_motor7.q = cmd->motorCmd[16].q;
    message_to_encode.arm_motor7.dq = cmd->motorCmd[16].dq;
    message_to_encode.arm_motor7.kp = cmd->motorCmd[16].Kp;
    message_to_encode.arm_motor7.kd = cmd->motorCmd[16].Kd;
    message_to_encode.arm_motor7.tau = cmd->motorCmd[16].tau;

    message_to_encode.arm_motor8.q = cmd->motorCmd[17].q;
    message_to_encode.arm_motor8.dq = cmd->motorCmd[17].dq;
    message_to_encode.arm_motor8.kp = cmd->motorCmd[17].Kp;
    message_to_encode.arm_motor8.kd = cmd->motorCmd[17].Kd;
    message_to_encode.arm_motor8.tau = cmd->motorCmd[17].tau;

    message_to_encode.Calibration = cmd->Calibration;

    message_to_encode.arm_motor1.mode = 1.0;

    auto encoded_message = customcommunication.encodeLowLevelCmd(message_to_encode);

    customcommunication.sendUdpMessage(encoded_message, 8080);
    // std::cout << "Sent" << std::endl;
}

void IOSDK::ReceiveUDP(LowlevelState *state)
{
    count++;
    // std::cout << "Custom Receive" << std::endl;
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    const auto _start_recv = std::chrono::high_resolution_clock::now();

    // // Receive UDP message and decode it here
    auto receivedBuffer = customcommunication.receiveUdpMessage();

    LowLevelState decodedState = customcommunication.decodeLowLevelState(receivedBuffer);

    state->motorState[0].q = decodedState.leg_motor1.q;
    state->motorState[0].dq = decodedState.leg_motor1.dq;
    state->motorState[0].tauEst = decodedState.leg_motor1.tauEst;
    state->motorState[0].temperature = decodedState.leg_motor1.temperature;
    state->motorState[0].error = decodedState.leg_motor1.error;

    // std::cout << "Leg1 motor q is " << decodedState.leg_motor1.q << std::endl;

    state->motorState[1].q = decodedState.leg_motor2.q;
    state->motorState[1].dq = decodedState.leg_motor2.dq;
    state->motorState[1].tauEst = decodedState.leg_motor2.tauEst;
    state->motorState[1].temperature = decodedState.leg_motor2.temperature;
    state->motorState[1].error = decodedState.leg_motor2.error;

    // std::cout << "Leg2 motor q is " << decodedState.leg_motor2.q << std::endl;

    state->motorState[2].q = decodedState.leg_motor3.q;
    state->motorState[2].dq = decodedState.leg_motor3.dq;
    state->motorState[2].tauEst = decodedState.leg_motor3.tauEst;
    state->motorState[2].temperature = decodedState.leg_motor3.temperature;
    state->motorState[2].error = decodedState.leg_motor3.error;

    // std::cout << "Leg3 motor q is " << decodedState.leg_motor3.q << std::endl;

    state->motorState[3].q = decodedState.leg_motor4.q;
    state->motorState[3].dq = decodedState.leg_motor4.dq;
    state->motorState[3].tauEst = decodedState.leg_motor4.tauEst;
    state->motorState[3].temperature = decodedState.leg_motor4.temperature;
    state->motorState[3].error = decodedState.leg_motor4.error;
    // std::cout << "Leg3 motor q is " << decodedState.leg_motor4.q << std::endl;

    state->motorState[4].q = decodedState.leg_motor5.q;
    state->motorState[4].dq = decodedState.leg_motor5.dq;
    state->motorState[4].tauEst = decodedState.leg_motor5.tauEst;
    state->motorState[4].temperature = decodedState.leg_motor5.temperature;
    state->motorState[4].error = decodedState.leg_motor5.error;
    // std::cout << "Leg5 motor q is " << decodedState.leg_motor5.q << std::endl;
    // std::cout << "\n\n";

    state->motorState[5].q = decodedState.leg_motor6.q;
    state->motorState[5].dq = decodedState.leg_motor6.dq;
    state->motorState[5].tauEst = decodedState.leg_motor6.tauEst;
    state->motorState[5].temperature = decodedState.leg_motor6.temperature;
    state->motorState[5].error = decodedState.leg_motor6.error;

    // std::cout << "Leg6 motor q is " << decodedState.leg_motor6.q << std::endl;

    state->motorState[6].q = decodedState.leg_motor7.q;
    state->motorState[6].dq = decodedState.leg_motor7.dq;
    state->motorState[6].tauEst = decodedState.leg_motor7.tauEst;
    state->motorState[6].temperature = decodedState.leg_motor7.temperature;
    state->motorState[6].error = decodedState.leg_motor7.error;
    // std::cout << "Leg7 motor q is " << decodedState.leg_motor7.q << std::endl;

    state->motorState[7].q = decodedState.leg_motor8.q;
    state->motorState[7].dq = decodedState.leg_motor8.dq;
    state->motorState[7].tauEst = decodedState.leg_motor8.tauEst;
    state->motorState[7].temperature = decodedState.leg_motor8.temperature;
    state->motorState[7].error = decodedState.leg_motor8.error;
    // std::cout << "Leg8 motor q is " << decodedState.leg_motor8.q<< std::endl;;

    state->motorState[8].q = decodedState.leg_motor9.q;
    state->motorState[8].dq = decodedState.leg_motor9.dq;
    state->motorState[8].tauEst = decodedState.leg_motor9.tauEst;
    state->motorState[8].temperature = decodedState.leg_motor9.temperature;
    state->motorState[8].error = decodedState.leg_motor9.error;
    // std::cout << "Leg9 motor q is " << decodedState.leg_motor9.q << std::endl;;

    state->motorState[9].q = decodedState.leg_motor10.q;
    state->motorState[9].dq = decodedState.leg_motor10.dq;
    state->motorState[9].tauEst = decodedState.leg_motor10.tauEst;
    state->motorState[9].temperature = decodedState.leg_motor10.temperature;
    state->motorState[9].error = decodedState.leg_motor10.error;
    // std::cout << "Leg10 motor q is " << decodedState.leg_motor10.q << std::endl;;

    // if(count % 10 ==0){
    // for (int m=0; m < 10; m++){
    //     if(m==5){
    //         std::cout<< "\n\n";}

    //     std::cout << "motor # " << m << "q is =  " << state->motorState[m].q <<std::endl;
    // }
    // }
    state->motorState[10].q = decodedState.arm_motor1.q;
    state->motorState[10].dq = decodedState.arm_motor1.dq;
    state->motorState[10].tauEst = decodedState.arm_motor1.tauEst;

    // std::cout << "Arm motor 1 state q is" << state->motorState[10].q << std::endl;

    state->motorState[11].q = decodedState.arm_motor2.q;
    state->motorState[11].dq = decodedState.arm_motor2.dq;
    state->motorState[11].tauEst = decodedState.arm_motor2.tauEst;
    // std::cout << "Arm motor 2 state q is" << state->motorState[11].q << std::endl;

    state->motorState[12].q = decodedState.arm_motor3.q;
    state->motorState[12].dq = decodedState.arm_motor3.dq;
    state->motorState[12].tauEst = decodedState.arm_motor3.tauEst;
    // std::cout << "Arm motor 3 state q is" << state->motorState[12].q << std::endl;

    state->motorState[13].q = decodedState.arm_motor4.q;
    state->motorState[13].dq = decodedState.arm_motor4.dq;
    state->motorState[13].tauEst = decodedState.arm_motor4.tauEst;
    // std::cout << "Arm motor 4 state q is" << state->motorState[13].q << std::endl;

    state->motorState[14].q = decodedState.arm_motor5.q;
    state->motorState[14].dq = decodedState.arm_motor5.dq;
    state->motorState[14].tauEst = decodedState.arm_motor5.tauEst;
    // std::cout << "Arm motor 5 state q is" << state->motorState[14].q << std::endl;

    state->motorState[15].q = decodedState.arm_motor6.q;
    state->motorState[15].dq = decodedState.arm_motor6.dq;
    state->motorState[15].tauEst = decodedState.arm_motor6.tauEst;
    // std::cout << "Ar/m motor 6 state q is" << state->motorState[15].q << std::endl;

    state->motorState[16].q = decodedState.arm_motor7.q;
    state->motorState[16].dq = decodedState.arm_motor7.dq;
    state->motorState[16].tauEst = decodedState.arm_motor7.tauEst;
    // std::cout << "Arm motor 7 state q is" << state->motorState[16].q << std::endl;

    state->motorState[17].q = decodedState.arm_motor8.q;
    state->motorState[17].dq = decodedState.arm_motor8.dq;
    state->motorState[17].tauEst = decodedState.arm_motor8.tauEst;
    // std::cout << "Arm motor 8 state q is" << state->motorState[17].q << std::endl;

    state->imu.gyroscope[0] = decodedState.imu_1.GyroX;
    state->imu.gyroscope[1] = decodedState.imu_1.GyroY;
    state->imu.gyroscope[2] = decodedState.imu_1.GyroZ;
    state->imu.accelerometer[0] = decodedState.imu_1.AccX;
    state->imu.accelerometer[1] = decodedState.imu_1.AccY;
    state->imu.accelerometer[2] = decodedState.imu_1.AccZ;

    if (!bias_calibrated)
    {
        gyro_bias_x_ += state->imu.gyroscope[0];
        gyro_bias_y_ += state->imu.gyroscope[1];
        gyro_bias_z_ += state->imu.gyroscope[2];

        acc_sum_x_ += state->imu.accelerometer[0];
        acc_sum_y_ += state->imu.accelerometer[1];
        acc_sum_z_ += state->imu.accelerometer[2];

        sample_count_++;

        if (sample_count_ == calibration_samples_)
        {
            gyro_bias_x_ /= calibration_samples_;
            gyro_bias_y_ /= calibration_samples_;
            gyro_bias_z_ /= calibration_samples_;

            avg_acc_x = acc_sum_x_ / calibration_samples_;
            avg_acc_y = acc_sum_y_ / calibration_samples_;
            avg_acc_z = acc_sum_z_ / calibration_samples_;

            // Calculate initial roll and pitch
            //roll_ = atan2(avg_acc_y, avg_acc_z);
            //pitch_ = atan2(-avg_acc_x, sqrt(avg_acc_y * avg_acc_y + avg_acc_z * avg_acc_z));

            // Assuming we start laying the robot down all the time
            //roll_ = 0.0;
            //pitch_ = +M_PI/2 + M_PI/80;
            bias_calibrated = true;
        }
    }
    else
    {
        corrected_gyro_x = state->imu.gyroscope[0]- gyro_bias_x_;
        corrected_gyro_y = state->imu.gyroscope[1]- gyro_bias_y_;
        corrected_gyro_z = state->imu.gyroscope[2]- gyro_bias_z_;

        // Integrate gyro to get orientation
        //roll_ += corrected_gyro_x * dt_;
        //pitch_ += corrected_gyro_y * dt_;
        //yaw_ += corrected_gyro_z * dt_;

        // Use new imu filter
        float gx = state->imu.gyroscope[0] - gyro_bias_x_;
        float gy = state->imu.gyroscope[1] - gyro_bias_y_;
        float gz = state->imu.gyroscope[2] - gyro_bias_z_;
        // The filter uses g scale
        float ax = state->imu.accelerometer[0]/9.81f;
        float ay = state->imu.accelerometer[1]/9.81f;
        float az = state->imu.accelerometer[2]/9.81f;
        imu_filter.update(gx,gy,gz, ax,ay,az);
        auto [rr, pp, yy] = imu_filter.get_euler();

        roll_ = (float)rr;
        pitch_ = (float)pp;
        yaw_ = (float)yy;
    }

    state->imu.rpy[0] = roll_;
    state->imu.rpy[1] = pitch_;
    state->imu.rpy[2] = yaw_;

    const auto _end_recv = std::chrono::high_resolution_clock::now();
    auto _duration_recv = std::chrono::duration_cast<std::chrono::microseconds>(_end_recv - _start_recv);
    // std::cout << "Time taken by recvState() message is " << _duration_recv.count() << "micro s\n";
}