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
    double gear_ratio_knee;
    double gear_ratio_arm;
    double knee_offset;
    int joint_dir[18];
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
    gear_ratio_knee = 2.0;
    gear_ratio_arm = 1.417;
    knee_offset = -2.38;
    int j_dir_init_vals[18] = {
        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,
        1, 1,-1, 1,
        1, 1,-1, 1
    };
    std::copy(j_dir_init_vals, j_dir_init_vals+18, joint_dir);
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
    //std::cout<<"lcm published"<<std::endl;
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

    // Test, Write a pd ctr with initial pos
    double init_q[18] = {0.00, 0.00, 0.785, -1.57, 0.785,
                          0.00, 0.00, 0.785, -1.57, 0.785,
                          0.00, 0.785, 0.000,  -1.57,
                          0.00, 0.785, 0.000,  -1.57};
    double kp_leg = 5.0;
    double kd_leg = 0.4;
    double kp_arm = 3.0;
    double kd_arm = 0.3;

    // Assign values for leg
    for(int i=0; i<10; i++)
    {
        mujoco_cmd->motorCmd[i].q = init_q[i];
        mujoco_cmd->motorCmd[i].dq = 0.0;
        mujoco_cmd->motorCmd[i].Kp = kp_leg;
        mujoco_cmd->motorCmd[i].Kd = kd_leg;
        mujoco_cmd->motorCmd[i].tau = 0.0;  
    }
    // Assign valuse for arm
    for(int i=10; i<18; i++)
    {
        mujoco_cmd->motorCmd[i].q = init_q[i];
        mujoco_cmd->motorCmd[i].dq = 0.0;
        mujoco_cmd->motorCmd[i].Kp = kp_arm;
        mujoco_cmd->motorCmd[i].Kd = kd_arm;
        mujoco_cmd->motorCmd[i].tau = 0.0;  
    }

    std::cout<<"Main control loop ctrl"<<std::endl;

    // also print imu data to dubug
    std::cout << "IMU deubug - x "<<mujoco_state->imu.rpy[0] 
              << " y - "<<mujoco_state->imu.rpy[1] 
              << " z - "<<mujoco_state->imu.rpy[2]
              <<std::endl;

}

// convert mujoco cmd to motor cmd with leg ik, mujoco_cmd->low_cmd
void custom_wrapper::hw_ik() {
    // Copy calibration straight through
    low_cmd->Calibration = mujoco_cmd->Calibration;

    // Legs 0–9 (2 legs × 5 joints each)
    for (int leg = 0; leg < 2; ++leg) {
        int base = leg * 5;

        // j0–j2: hips (direct copy + sign flip)
        for (int j = 0; j < 3; ++j) {
            int i = base + j;
            low_cmd->motorCmd[i].q   = joint_dir[i] * mujoco_cmd->motorCmd[i].q;
            low_cmd->motorCmd[i].dq  = joint_dir[i] * mujoco_cmd->motorCmd[i].dq;
            low_cmd->motorCmd[i].tau = joint_dir[i] * mujoco_cmd->motorCmd[i].tau;
            low_cmd->motorCmd[i].Kp  =               mujoco_cmd->motorCmd[i].Kp;
            low_cmd->motorCmd[i].Kd  =               mujoco_cmd->motorCmd[i].Kd;
        }

        // j3: knee (with offset+ratio transform)
        int i3 = base + 3;
        {
            double qd = mujoco_cmd->motorCmd[i3].q,
                   dd = mujoco_cmd->motorCmd[i3].dq,
                   tt = mujoco_cmd->motorCmd[i3].tau;
            double q_raw  = (qd - knee_offset) * gear_ratio_knee + knee_offset;
            double dq_raw = dd  * gear_ratio_knee;
            double t_raw  = tt  / gear_ratio_knee;

            low_cmd->motorCmd[i3].q   = joint_dir[i3] * q_raw;
            low_cmd->motorCmd[i3].dq  = joint_dir[i3] * dq_raw;
            low_cmd->motorCmd[i3].tau = joint_dir[i3] * t_raw;
            low_cmd->motorCmd[i3].Kp  = mujoco_cmd->motorCmd[i3].Kp / (gear_ratio_knee * gear_ratio_knee);
            low_cmd->motorCmd[i3].Kd  = mujoco_cmd->motorCmd[i3].Kd / (gear_ratio_knee * gear_ratio_knee);
        }

        // j4: ankle (dependent on both ankle & knee desired)
        int i4 = base + 4;
        {
            double qd3 = mujoco_cmd->motorCmd[i3].q,
                   dd3 = mujoco_cmd->motorCmd[i3].dq,
                   qd4 = mujoco_cmd->motorCmd[i4].q,
                   dd4 = mujoco_cmd->motorCmd[i4].dq;

            double q_raw  = qd4 + qd3 - knee_offset;
            double dq_raw = dd4 + dd3;

            low_cmd->motorCmd[i4].q   = joint_dir[i4] * q_raw;
            low_cmd->motorCmd[i4].dq  = joint_dir[i4] * dq_raw;
            low_cmd->motorCmd[i4].tau = joint_dir[i4] * mujoco_cmd->motorCmd[i4].tau;
            low_cmd->motorCmd[i4].Kp  =               mujoco_cmd->motorCmd[i4].Kp;
            low_cmd->motorCmd[i4].Kd  =               mujoco_cmd->motorCmd[i4].Kd;
        }
    }

    // Arms j10–j17 (direct copy + sign flip)
    for (int i = 10; i < 18; ++i) {
        low_cmd->motorCmd[i].q   = joint_dir[i] * mujoco_cmd->motorCmd[i].q;
        low_cmd->motorCmd[i].dq  = joint_dir[i] * mujoco_cmd->motorCmd[i].dq;
        low_cmd->motorCmd[i].tau = joint_dir[i] * mujoco_cmd->motorCmd[i].tau;
        low_cmd->motorCmd[i].Kp  =               mujoco_cmd->motorCmd[i].Kp;
        low_cmd->motorCmd[i].Kd  =               mujoco_cmd->motorCmd[i].Kd;
    }
    // Overwrite last arm joint with reduction ratio
    low_cmd->motorCmd[13].q   = joint_dir[13] * mujoco_cmd->motorCmd[13].q * gear_ratio_arm;
    low_cmd->motorCmd[13].dq  = joint_dir[13] * mujoco_cmd->motorCmd[13].dq * gear_ratio_arm;
    low_cmd->motorCmd[13].tau = joint_dir[13] * mujoco_cmd->motorCmd[13].tau / gear_ratio_arm;

    low_cmd->motorCmd[17].q   = joint_dir[17] * mujoco_cmd->motorCmd[17].q * gear_ratio_arm;
    low_cmd->motorCmd[17].dq  = joint_dir[17] * mujoco_cmd->motorCmd[17].dq * gear_ratio_arm;
    low_cmd->motorCmd[17].tau = joint_dir[17] * mujoco_cmd->motorCmd[17].tau / gear_ratio_arm;
}


// convert motor state to mujoco state with leg fk, low_state->mujoco_state
void custom_wrapper::hw_fk() {
    // Legs 0–9 (2 legs × 5 joints each)
    for (int leg = 0; leg < 2; ++leg) {
        int base = leg * 5;

        // j0–j2: hips (direct copy + sign flip)
        for (int j = 0; j < 3; ++j) {
            int i = base + j;
            mujoco_state->motorState[i].q       = joint_dir[i] * low_state->motorState[i].q;
            mujoco_state->motorState[i].dq      = joint_dir[i] * low_state->motorState[i].dq;
            mujoco_state->motorState[i].tauEst  = joint_dir[i] * low_state->motorState[i].tauEst;
            mujoco_state->motorState[i].temperature = low_state->motorState[i].temperature;
            mujoco_state->motorState[i].error       = low_state->motorState[i].error;
        }

        // j3: knee (invert offset+ratio)
        int i3 = base + 3;
        {
            double q_raw  = low_state->motorState[i3].q,
                   dq_raw = low_state->motorState[i3].dq,
                   tt_raw = low_state->motorState[i3].tauEst;
            double q = (q_raw - knee_offset) / gear_ratio_knee + knee_offset;
            double dq = dq_raw / gear_ratio_knee;
            double t = tt_raw * gear_ratio_knee;

            mujoco_state->motorState[i3].q      = joint_dir[i3] * q;
            mujoco_state->motorState[i3].dq     = joint_dir[i3] * dq;
            mujoco_state->motorState[i3].tauEst = joint_dir[i3] * t;
            mujoco_state->motorState[i3].temperature = low_state->motorState[i3].temperature;
            mujoco_state->motorState[i3].error       = low_state->motorState[i3].error;
        }

        // j4: ankle (depends on raw ankle minus knee-angle)
        int i4 = base + 4;
        {
            double q3 = (low_state->motorState[i3].q - knee_offset) / gear_ratio_knee + knee_offset;
            double q_raw  = low_state->motorState[i4].q,
                   dq_raw = low_state->motorState[i4].dq,
                   tt_raw = low_state->motorState[i4].tauEst;
            double q = q_raw - q3 + knee_offset;
            double dq = dq_raw - ((low_state->motorState[i3].dq) / gear_ratio_knee);

            mujoco_state->motorState[i4].q      = joint_dir[i4] * q;
            mujoco_state->motorState[i4].dq     = joint_dir[i4] * dq;
            mujoco_state->motorState[i4].tauEst = joint_dir[i4] * tt_raw;
            mujoco_state->motorState[i4].temperature = low_state->motorState[i4].temperature;
            mujoco_state->motorState[i4].error       = low_state->motorState[i4].error;
        }
    }

    // Arms j10–j17 (direct copy + sign flip)
    for (int i = 10; i < 18; ++i) {
        mujoco_state->motorState[i].q      = joint_dir[i] * low_state->motorState[i].q;
        mujoco_state->motorState[i].dq     = joint_dir[i] * low_state->motorState[i].dq;
        mujoco_state->motorState[i].tauEst = joint_dir[i] * low_state->motorState[i].tauEst;
        mujoco_state->motorState[i].temperature = low_state->motorState[i].temperature;
        mujoco_state->motorState[i].error       = low_state->motorState[i].error;
    }

    // Overwrite last arm joint with reduction ratio
    mujoco_state->motorState[13].q   = joint_dir[13] * low_state->motorState[13].q / gear_ratio_arm;
    mujoco_state->motorState[13].dq  = joint_dir[13] * low_state->motorState[13].dq / gear_ratio_arm;
    mujoco_state->motorState[13].tauEst = joint_dir[13] * low_state->motorState[13].tauEst * gear_ratio_arm;

    mujoco_state->motorState[17].q   = joint_dir[17] * low_state->motorState[17].q / gear_ratio_arm;
    mujoco_state->motorState[17].dq  = joint_dir[17] * low_state->motorState[17].dq / gear_ratio_arm;
    mujoco_state->motorState[17].tauEst = joint_dir[17] * low_state->motorState[17].tauEst * gear_ratio_arm;

    // IMU (unchanged)
    for (int i = 0; i < 3; ++i) {
        mujoco_state->imu.rpy[i]           = low_state->imu.rpy[i];
        mujoco_state->imu.gyroscope[i]     = low_state->imu.gyroscope[i];
        mujoco_state->imu.accelerometer[i] = low_state->imu.accelerometer[i];
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

    double ctrl_dt = 0.02; //50hz
    double lowlevel_dt = 0.0005; //2000hz

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
    loop_lcm_send.start();
    //loop_lcm_recv.start();
    loop_ctrl.start();

    while(true)
    {
        sleep(10);
    }

    return 0;
}
