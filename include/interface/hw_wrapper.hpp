#ifndef HW_WRAPPER_H
#define HW_WRAPPER_H

#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <map>
#include <algorithm>

#include <lcm/lcm-cpp.hpp>
#include "../Custom_SDK/include/low_cmd_t.hpp"
#include "../Custom_SDK/include/low_state_t.hpp"

#include "IOSDK.h"
#include "t265.hpp"

#include <onnxruntime_cxx_api.h>

// --- Constants and Configuration ---
constexpr int NUM_JOINTS = 18;
constexpr int OBS_DIM = 67;
constexpr int OBS_HIST = 10;
const double INTERPOLATION_DURATION = 2.0;

// Use inline variables for C++17 to define vectors in a header
inline const Eigen::VectorXd DEFAULT_Q = (Eigen::VectorXd(NUM_JOINTS) 
                                       << 0.00, 0.00, 0.785+0.22, -1.57, 0.785-0.02,
                                          0.00, 0.00, 0.785+0.22, -1.57, 0.785-0.02, //-0.05
                                          0.00, 0.785, 0.000, -1.57,
                                          0.00, 0.785, 0.000, -1.57)
                                             .finished();

inline const Eigen::VectorXd STANDING_Q = DEFAULT_Q;

inline const Eigen::VectorXd STANDING_KP = 1.0 * (Eigen::VectorXd(NUM_JOINTS) 
                                         << 35.0, 35.0, 35.0, 35.0, 35.0,
                                            35.0, 35.0, 35.0, 35.0, 35.0,
                                            3.0, 3.0, 3.0, 3.0,
                                            3.0, 3.0, 3.0, 3.0)
                                               .finished(); // 60 40

inline const Eigen::VectorXd STANDING_KD = 1.2 * (Eigen::VectorXd(NUM_JOINTS) 
                                             << 1.0, 1.0, 1.0, 1.0, 1.0,
                                                1.0, 1.0, 1.0, 1.0, 1.0,
                                                0.1, 0.1, 0.1, 0.1,
                                                0.1, 0.1, 0.1, 0.1)
                                                   .finished();

inline const Eigen::VectorXd POLICY_KP = STANDING_KP;
inline const Eigen::VectorXd POLICY_KD = STANDING_KD;

// --- Enums and Dataclasses ---
enum class ControlState
{
    DAMPING,
    STANDING,
    POLICY
};

class hw_wrapper
{
public:
    hw_wrapper();//(t265_wrapper *_t265);
    ~hw_wrapper();
    void hw_send();
    void hw_recv();
    void ctrl_loop();
    void uni_ctrl_loop();

    void hw_ik(); // convert mujoco cmd to motor cmd with leg ik
    void hw_ik_dir_only();
    void hw_ik_shaping();
    void hw_fk(); // convert motor state to mujoco state with leg fk

    Eigen::VectorXf get_obs();
    void switch_state(ControlState new_state);
    void update_gains();

    Safety *safe;
    LowlevelCmd *low_cmd;     // cmd sent to motor
    LowlevelState *low_state; // state recv from motor

    LowlevelCmd *policy_cmd;     // cmd recv form policy, beforce ik
    LowlevelState *policy_state; // state ready to sent to policy, after fk

    IOInterface *io_interface;

    lcm::LCM lcm;
    // Prepared state sent to upperstream python
    exlcm::low_state_t lcm_mujoco_state;
    std::string state_channel_name;
    std::string cmd_channel_name;

    // State machine
    ControlState state;
    ControlState next_state;

    // Interpolation kp kd
    bool is_interpolating = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> interpolation_start_time;
    Eigen::VectorXd kp_start, kd_start;
    Eigen::VectorXd kp_target, kd_target;
    std::map<ControlState, std::pair<Eigen::VectorXd, Eigen::VectorXd>> target_gains;

    // ONNX Policy
    Ort::Env ort_env;
    Ort::Session ort_session;
    Ort::MemoryInfo memory_info;
    std::vector<const char *> input_node_names;
    std::vector<const char *> output_node_names;

    // Observation history & gait
    Eigen::VectorXf last_action;
    Eigen::VectorXf obs_buffer;
    Eigen::Vector2d phase;

    double gait_freq = 1.8;
    double action_scale = 0.5;
    double ctrl_dt = 0.02;
    double phase_dt;
    long _counter = 0;
    int _sub_count = 20;

    // User commands
    Eigen::Vector3d twist_command = Eigen::Vector3d::Zero();
    Eigen::VectorXd body_command = Eigen::VectorXd::Zero(12);

    // Buffer for filters
    Eigen::Vector3d gyro_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_last = Eigen::Vector3d::Zero();

    Eigen::VectorXd q_n = Eigen::VectorXd::Zero(NUM_JOINTS);
    Eigen::VectorXd dq_n = Eigen::VectorXd::Zero(NUM_JOINTS);
    Eigen::VectorXd q_last = Eigen::VectorXd::Zero(NUM_JOINTS);
    Eigen::VectorXd dq_last = Eigen::VectorXd::Zero(NUM_JOINTS);

    t265_wrapper *t265;

private:
    // HW related params
    double gear_ratio_knee;
    double gear_ratio_arm;
    double knee_offset;
    int joint_dir[18];
    int imu_dir[3];
};

#endif
