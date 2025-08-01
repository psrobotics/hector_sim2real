#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include <lcm/lcm-cpp.hpp>
#include "../include/interface/IOSDK.h"
#include "../include/Custom_SDK/include/low_cmd_t.hpp"
#include "../include/Custom_SDK/include/low_state_t.hpp"

using namespace UNITREE_LEGGED_SDK;

bool running = true;


class custom_wrapper
{
public:
    custom_wrapper();
    ~custom_wrapper();
    void hw_send();
    void hw_recv();
    void ctrl_loop();
    void lcm_send();
    void lcm_recv();
    void handle_low_cmd(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const exlcm::low_cmd_t* msg);
    void hw_ik(); // convert mujoco cmd to motor cmd with leg ik
    void hw_fk(); // convert motor state to mujoco state with leg fk

    Safety *safe;
    LowlevelCmd *low_cmd; // cmd sent to motor
    LowlevelState *low_state; // state recv from motor

    LowlevelCmd *mujoco_cmd; // cmd recv form policy, beforce ik
    LowlevelState *mujoco_state; // state ready to sent to policy, after fk

    IOInterface *io_interface;

    lcm::LCM lcm;
    // Prepared state sent to upperstream python
    exlcm::low_state_t lcm_mujoco_state;
    std::string state_channel_name;
    std::string cmd_channel_name;

private:
    // HW related params
    double gear_ratio;
    double knee_offset;
};

custom_wrapper::custom_wrapper()
{
    low_cmd = new LowlevelCmd();
    low_state = new LowlevelState();

    mujoco_cmd = new LowlevelCmd();
    mujoco_state = new LowlevelState();

    safe = new Safety(LeggedType::A1);
    // Init with keyboard ctrl
    io_interface = new IOSDK(LeggedType::A1, 2);

    if(!lcm.good())
        std::cerr<<"LCM initial failed"<<std::endl;
    state_channel_name = "HECTOR_HW_LOW_STATE";
    cmd_channel_name = "HECTOR_HW_LOW_CMD";

    lcm.subscribe(cmd_channel_name, &custom_wrapper::handle_low_cmd, this);

    // Init hw related params
    gear_ratio = 2.0;
    knee_offset = -2.38;
}

custom_wrapper::~custom_wrapper()
{
    delete low_cmd;
    delete low_state;
    delete mujoco_cmd;
    delete mujoco_state;
    delete safe;
    delete io_interface;
}

void custom_wrapper::hw_send()
{
    hw_ik();
    io_interface->sendUDP(low_cmd);
    //std::cout<<"Low level send loop"<<std::endl;
}

void custom_wrapper::hw_recv()
{
    io_interface->ReceiveUDP(low_state);
    hw_fk();
    //std::cout<<"Low level recv loop"<<std::endl;
}

// Send mujoco state to upper-stream python script
void custom_wrapper::lcm_send()
{
    // Copy low-level states first
    // Notice that some arm motor does not come with joint temp and err
    for(int _id=0; _id<18; _id++)
    {
        lcm_mujoco_state.q[_id] = mujoco_state->motorState[_id].q;
        lcm_mujoco_state.dq[_id] = mujoco_state->motorState[_id].dq;
        lcm_mujoco_state.ddq[_id] = 0.0;
        lcm_mujoco_state.tau_est[_id] = mujoco_state->motorState[_id].tauEst;
        lcm_mujoco_state.temperature[_id] = mujoco_state->motorState[_id].temperature;
        lcm_mujoco_state.error[_id] = mujoco_state->motorState[_id].error;
    }
    // IMU data
    for(int _i=0; _i<3; _i++)
    {
        lcm_mujoco_state.rpy[_i] = mujoco_state->imu.rpy[_i];
        lcm_mujoco_state.gyroscope[_i] = mujoco_state->imu.gyroscope[_i];
        lcm_mujoco_state.accelerometer[_i] = mujoco_state->imu.accelerometer[_i];
    }   

    const auto _t = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()
             ).count();
    lcm_mujoco_state.timestamp = _t;

    lcm.publish(state_channel_name, &lcm_mujoco_state);
}


// Handle to recv mujoco cmd from upper stream python script
void custom_wrapper::handle_low_cmd(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const exlcm::low_cmd_t* msg)
{
    std::cout<<"Received cmd channel - "<<chan<<std::endl;
    // Parse all low cmd
    // Only one for all calibrate at low-cmd
    low_cmd->Calibration = msg->calibrate[0];
    for(int _i=0; _i<18; _i++)
    {
        mujoco_cmd->motorCmd[_i].q = msg->q[_i];
        mujoco_cmd->motorCmd[_i].dq = msg->dq[_i];
        mujoco_cmd->motorCmd[_i].Kp = msg->kp[_i];
        mujoco_cmd->motorCmd[_i].Kd = msg->kd[_i];
        mujoco_cmd->motorCmd[_i].tau = msg->tau[_i];
    }
}

// Spin lcm handle to recv mujoco cmd
void custom_wrapper::lcm_recv()
{
    lcm.handle();
    std::cout<<"Recv mujoco cmd"<<std::endl;
}

// Test control loop to interact with hw
void custom_wrapper::ctrl_loop()
{
    mujoco_cmd->Calibration = 0;

    // Legs
    for (int i = 0; i < 10; i++)
    {
        mujoco_cmd->motorCmd[i].q = 0;
        mujoco_cmd->motorCmd[i].dq = 0;
        mujoco_cmd->motorCmd[i].Kp = 0.0;
        mujoco_cmd->motorCmd[i].Kd = 0.0;
        mujoco_cmd->motorCmd[i].tau = 0;
    }
    // Just arms stat
    for (int i = 10; i < 18; i++)
    {
        mujoco_cmd->motorCmd[i].q = 0;
        mujoco_cmd->motorCmd[i].dq = 0;
        mujoco_cmd->motorCmd[i].Kp = 1.0;
        mujoco_cmd->motorCmd[i].Kd = 0.05;
        mujoco_cmd->motorCmd[i].tau = 0;
    }

    // Hip r
    mujoco_cmd->motorCmd[2].q = 1.0;
    mujoco_cmd->motorCmd[2].dq = 0;
    mujoco_cmd->motorCmd[2].Kp = 3.5;
    mujoco_cmd->motorCmd[2].Kd = 0.1;
    mujoco_cmd->motorCmd[2].tau = 0;
    // Hip l
    mujoco_cmd->motorCmd[7].q = 1.0;
    mujoco_cmd->motorCmd[7].dq = 0;
    mujoco_cmd->motorCmd[7].Kp = 3.5;
    mujoco_cmd->motorCmd[7].Kd = 0.1;
    mujoco_cmd->motorCmd[7].tau = 0;

    std::cout<<"Main control loop ctrl"<<std::endl;

    // also print imu data to dubug
    std::cout << "IMU deubug - x "<<low_state->imu.rpy[0] 
              << " y - "<<low_state->imu.rpy[1] 
              << " z - "<<low_state->imu.rpy[2]
              <<std::endl;

}

// convert mujoco cmd to motor cmd with leg ik, mujoco_cmd->motor_cmd
void custom_wrapper::hw_ik()
{
    std::cout<<"hw ik"<<std::endl;

    // Copy calibration
    low_cmd->Calibration = mujoco_cmd->Calibration;

    for(int leg=0; leg<2; leg++)
    {
        int base = leg*5;
        // j0~j2 hip joints, direct copy
        for(int _j=0; _j<3; _j++)
        {
            int _i = base+_j;
            low_cmd->motorCmd[_i] = mujoco_cmd->motorCmd[_i];
        }

        // j3 knee
        int _i3 = base+3;
        double q3_des = mujoco_cmd->motorCmd[_i3].q;
        double dq3_des = mujoco_cmd->motorCmd[_i3].dq;
        double tau3_des = mujoco_cmd->motorCmd[_i3].tau;
        // inverse: raw = (q_des – offset)*ratio + offset
        double q3_raw  = (q3_des  - knee_offset) * gear_ratio + knee_offset;
        double dq3_raw = dq3_des * gear_ratio;
        double tau3_raw = tau3_des / gear_ratio;
        low_cmd->motorCmd[_i3].q = q3_raw;
        low_cmd->motorCmd[_i3].dq = dq3_raw;
        low_cmd->motorCmd[_i3].tau = tau3_raw;
        low_cmd->motorCmd[_i3].Kp = mujoco_cmd->motorCmd[_i3].Kp / (gear_ratio*gear_ratio);
        low_cmd->motorCmd[_i3].Kd = mujoco_cmd->motorCmd[_i3].Kd / (gear_ratio*gear_ratio);

        // j4 ankle
        int _i4 = base+4;
        double q4_des = mujoco_cmd->motorCmd[_i4].q;
        double dq4_des = mujoco_cmd->motorCmd[_i4].dq;
        double tau4_des = mujoco_cmd->motorCmd[_i4].tau;
        // inverse: raw_toe = q4_des + q3_des – offset
        double q4_raw = q4_des + q3_des - knee_offset;
        // and dq4_raw = dq4_des + dq3_des
        double dq4_raw = dq4_des + dq3_des;
        low_cmd->motorCmd[_i4].q  = q4_raw;
        low_cmd->motorCmd[_i4].dq = dq4_raw;
        low_cmd->motorCmd[_i4].tau = mujoco_cmd->motorCmd[_i4].tau;
        low_cmd->motorCmd[_i4].Kp  = mujoco_cmd->motorCmd[_i4].Kp;
        low_cmd->motorCmd[_i4].Kd  = mujoco_cmd->motorCmd[_i4].Kd;
    }
    // Copy arms j10-17 directly
    for (int _i = 10; _i < 18; _i++) {
        low_cmd->motorCmd[_i] = mujoco_cmd->motorCmd[_i];
    }
}
// convert motor state to mujoco state with leg fk, low_state->mujoco_state
void custom_wrapper::hw_fk()
{
    std::cout<<"hw fk"<<std::endl;
    for(int leg=0; leg<2; leg++)
    {
        int base = leg*5;
        // j0~j2 hip joints, direct copy
        for(int _j=0; _j<3; _j++)
        {
            int _i = base+_j;
            mujoco_state->motorState[_i] = low_state->motorState[_i];
        }

        // j3 knee
        int _i3 = base+3;
        double q3_raw = low_state->motorState[_i3].q;
        double dq3_raw = low_state->motorState[_i3].dq;
        double tau3_raw = low_state->motorState[_i3].tauEst;
        // forward: data_q3 = offset + (raw_q3 – offset)/ratio
        double q3 = (q3_raw  - knee_offset) / gear_ratio + knee_offset;
        double dq3 = dq3_raw / gear_ratio;
        double tau3 = tau3_raw * gear_ratio;
        mujoco_state->motorState[_i3].q = q3;
        mujoco_state->motorState[_i3].dq = dq3;
        mujoco_state->motorState[_i3].tauEst = tau3;
        mujoco_state->motorState[_i3].temperature = low_state->motorState[_i3].temperature;
        mujoco_state->motorState[_i3].error = low_state->motorState[_i3].error;

        // j4 ankle
        int _i4 = base+4;
        double q4_raw = low_state->motorState[_i4].q;
        double dq4_raw = low_state->motorState[_i4].dq;
        double tau4_raw = low_state->motorState[_i4].tauEst;
        // forward: data_q4 = raw_q4 – data_q3 + offset
        double q4 = q4_raw - q3 + knee_offset;
        // and dq4_raw = dq4_des + dq3_des
        double dq4 = dq4_raw - dq3;
        mujoco_state->motorState[_i4].q = q4;
        mujoco_state->motorState[_i4].dq = dq4;
        mujoco_state->motorState[_i4].tauEst = tau4_raw;
        mujoco_state->motorState[_i4].temperature = low_state->motorState[_i4].temperature;
        mujoco_state->motorState[_i4].error = low_state->motorState[_i4].error;
    }
    // Copy arms j10-17 directly
    for (int _i = 10; _i < 18; _i++) {
        mujoco_state->motorState[_i] = low_state->motorState[_i];
    }
    // Copy imu data
    for(int _i=10; _i<3; _i++)
    {
        mujoco_state->imu.rpy[_i] = low_state->imu.rpy[_i];
        mujoco_state->imu.gyroscope[_i] = low_state->imu.gyroscope[_i];
        mujoco_state->imu.accelerometer[_i] = low_state->imu.accelerometer[_i];
    }
}

void set_process_scheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] function setprocessscheduler failed \n ";
    }
}

int main()
{
    set_process_scheduler();

    double ctrl_dt = 0.02;
    double lowlevel_dt = 0.002;

    int robot_id = 4;     // AlienGo=1, A1=2, Biped=4
    int cmd_panel_id = 2; // Wireless=1, keyboard=2

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // (since C++11) use brace‐initialization
    custom_wrapper wrapper{};

    LoopFunc loop_ctrl("control_loop", ctrl_dt, boost::bind(&custom_wrapper::ctrl_loop, &wrapper));
    LoopFunc loop_hw_send("udp_send", lowlevel_dt, boost::bind(&custom_wrapper::hw_send, &wrapper));
    LoopFunc loop_hw_recv("udp_recv", lowlevel_dt, boost::bind(&custom_wrapper::hw_recv, &wrapper));
    LoopFunc loop_lcm_send("lcm_send", lowlevel_dt, boost::bind(&custom_wrapper::lcm_send, &wrapper));
    LoopFunc loop_lcm_recv("lcm_recv", lowlevel_dt, boost::bind(&custom_wrapper::lcm_recv, &wrapper));

    loop_hw_send.start();
    loop_hw_recv.start();
    //loop_lcm_send.start();
    //loop_lcm_recv.start();
    loop_ctrl.start();

    while(true)
    {
        sleep(10);
    }

    return 0;
}
