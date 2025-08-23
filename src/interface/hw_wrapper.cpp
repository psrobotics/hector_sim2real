#include "../../include/interface/hw_wrapper.hpp"

hw_wrapper::hw_wrapper()
    : ort_env(ORT_LOGGING_LEVEL_WARNING, "hector_policy"),
      ort_session(ort_env, "../src/onnx/kp35/hector_wbc_s2_0820_1.onnx", Ort::SessionOptions{nullptr}),
      memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      kp_start(NUM_JOINTS),
      kd_start(NUM_JOINTS)
{
    // Init buffers
    low_cmd = new LowlevelCmd();
    low_state = new LowlevelState();
    policy_cmd = new LowlevelCmd();
    policy_state = new LowlevelState();

    safe = new Safety(LeggedType::A1);
    io_interface = new IOSDK(LeggedType::A1, 2);

    // Gait and command setup
    phase << 0.0, M_PI;
    phase_dt = 2 * M_PI * gait_freq * ctrl_dt;
    body_command(0) = 0.55;

    // Gains map
    target_gains[ControlState::DAMPING] = {Eigen::VectorXd::Constant(NUM_JOINTS, 3.0), Eigen::VectorXd::Constant(NUM_JOINTS, 5.0)};
    target_gains[ControlState::STANDING] = {STANDING_KP, STANDING_KD};
    target_gains[ControlState::POLICY] = {POLICY_KP, POLICY_KD};

    for (int i = 0; i < 18; i++)
    {
        policy_cmd->motorCmd[i].Kp = target_gains[ControlState::DAMPING].first(i);
        policy_cmd->motorCmd[i].Kd = target_gains[ControlState::DAMPING].second(i);
    }

    // ONNX setup
    input_node_names.push_back("obs");
    output_node_names.push_back("continuous_actions");

    // Observation buffers
    int obs_size_n = 82;
    last_action.setZero(NUM_JOINTS);
    last_obs_1.setZero(obs_size_n);
    last_obs_2.setZero(obs_size_n);
    last_obs_3.setZero(obs_size_n);

    std::cout << "Starting in DAMPING state." << std::endl;

    // Init hw related params
    gear_ratio_knee = 2.0;
    gear_ratio_arm = 1.417;
    knee_offset = -2.38;
    int j_dir_init_vals[18] = {
        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,
        1, 1, -1, 1,
        1, 1, -1, 1};
    std::copy(j_dir_init_vals, j_dir_init_vals + 18, joint_dir);
    int imu_dir_vals[3] = {1, 1, 1};
    std::copy(imu_dir_vals, imu_dir_vals + 3, imu_dir);
}

hw_wrapper::~hw_wrapper()
{
    delete low_cmd;
    delete low_state;
    delete policy_cmd;
    delete policy_state;
    delete safe;
    delete io_interface;
}

void hw_wrapper::hw_send()
{
    hw_ik();
    io_interface->sendUDP(low_cmd);
    // std::cout<<"Low level send loop"<<std::endl;
}

void hw_wrapper::hw_recv()
{
    io_interface->ReceiveUDP(low_state);
    hw_fk();
    // std::cout<<"Low level recv loop"<<std::endl;
}

void hw_wrapper::switch_state(ControlState new_state)
{
    if (state == new_state && !is_interpolating)
        return;

    std::cout << "\n---> Requesting transition to new state..." << std::endl;
    is_interpolating = true;
    interpolation_start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        kp_start(i) = policy_cmd->motorCmd[i].Kp;
        kd_start(i) = policy_cmd->motorCmd[i].Kd;
    }

    kp_target = target_gains[new_state].first;
    kd_target = target_gains[new_state].second;
    next_state = new_state;
}

void hw_wrapper::update_gains()
{
    if (!is_interpolating)
        return;

    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(now - interpolation_start_time).count();
    double progress = std::min(elapsed_time / INTERPOLATION_DURATION, 1.0);

    Eigen::VectorXd interpolated_kp = kp_start + (kp_target - kp_start) * progress;
    Eigen::VectorXd interpolated_kd = kd_start + (kd_target - kd_start) * progress;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        policy_cmd->motorCmd[i].Kp = interpolated_kp(i);
        policy_cmd->motorCmd[i].Kd = interpolated_kd(i);
    }

    if (progress >= 1.0)
    {
        is_interpolating = false;
        state = next_state;
        std::cout << "Transition complete. Now in new state." << std::endl;
    }
}

Eigen::VectorXf hw_wrapper::get_obs()
{
    // IMU and acc
    auto rpy = policy_state->imu.rpy;
    Eigen::AngleAxisd roll_t(rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_t(rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_t(rpy[2], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_t * pitch_t * roll_t;
    Eigen::Matrix3d imu_xmat = q.matrix();
    Eigen::Vector3d gravity_vec = imu_xmat.transpose() * Eigen::Vector3d(0, 0, -1);

    Eigen::Vector4d phase_obs;
    phase_obs << cos(phase(0)), cos(phase(1)), sin(phase(0)), sin(phase(1));

    // Simple low-pass filters
    Eigen::Vector3d gyro, acc;
    for (int i = 0; i < 3; i++)
    {
        gyro(i) = policy_state->imu.gyroscope[i];
        acc(i) = policy_state->imu.accelerometer[i];
    }
    Eigen::Vector3d gyro_f = 0.0 * gyro_last + 1.0 * gyro;
    Eigen::Vector3d acc_f = 0.0 * acc_last + 1.0 * acc;

    // Copy over q, dq
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        q_n(i) = policy_state->motorState[i].q;
        dq_n(i) = policy_state->motorState[i].dq;
    }
    Eigen::VectorXd q_f = 0.0 * q_last + 1.0 * q_n;
    Eigen::VectorXd dq_f = 0.2 * dq_last + 0.8 * dq_n;

    // Concatenate observation parts
    Eigen::VectorXf obs_n(OBS_DIM);
    obs_n << gyro_f.cast<float>(),       // 3
        acc_f.cast<float>(),             // 3
        gravity_vec.cast<float>(),       // 3
        (q_f - DEFAULT_Q).cast<float>(), // 18
        dq_f.cast<float>(),              // 18
        last_action,                     // 18
        phase_obs.cast<float>(),         // 4
        twist_command.cast<float>(),     // 3
        body_command.cast<float>();      // 12

    // Take in past 3 history obs
    Eigen::VectorXf obs_full(obs_n.size() * 4);
    obs_full << obs_n, last_obs_1, last_obs_2, last_obs_3;

    // Update buffers
    gyro_last = gyro;
    acc_last = acc;
    q_last = q_n;
    dq_last = dq_n;

    last_obs_3 = last_obs_2;
    last_obs_2 = last_obs_1;
    last_obs_1 = obs_n;

    return obs_full;
}

// Test control loop to interact with hw
void hw_wrapper::ctrl_loop()
{
    update_gains();

    std::cout<<twist_command[0]<<std::endl;

    switch (state)
    {
    case ControlState::DAMPING:
    {
        std::cout << "DAMPING" << std::endl;
        std::cout << policy_cmd->motorCmd[0].Kp << " - " << policy_cmd->motorCmd[0].Kd << std::endl;

        for (int i = 0; i < 18; i++)
            policy_cmd->motorCmd[i].q = policy_state->motorState[i].q;

        break;
    }
    case ControlState::STANDING:
    {
        std::cout << "STANDING" << std::endl;
        std::cout << policy_cmd->motorCmd[0].Kp << " - " << policy_cmd->motorCmd[0].Kd << std::endl;

        for (int i = 0; i < 18; i++)
            policy_cmd->motorCmd[i].q = STANDING_Q[i];
        break;
    }
    case ControlState::POLICY:
    {
        std::cout << "POLICY" << std::endl;
        Eigen::VectorXf obs = get_obs();
        std::vector<int64_t> input_shape = {1, obs.size()};

        // Get onnx output
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info,
                                                                  obs.data(),
                                                                  obs.size(),
                                                                  input_shape.data(),
                                                                  input_shape.size());
        auto output_tensors = ort_session.Run(Ort::RunOptions{nullptr},
                                              input_node_names.data(),
                                              &input_tensor,
                                              1,
                                              output_node_names.data(),
                                              1);
        float *action_ptr = output_tensors.front().GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action(action_ptr, NUM_JOINTS);

        // Low pass filter for action
        Eigen::VectorXf action_f = 0.2 * last_action + 0.8 * action;
        // Update action buffer
        last_action = action;
        Eigen::VectorXf q_tar_n = action_f * static_cast<float>(action_scale) + DEFAULT_Q.cast<float>();

        std::cout << q_tar_n << std::endl;

        // Update phase
        phase(0) = fmod(phase(0) + phase_dt + M_PI, 2 * M_PI) - M_PI;
        phase(1) = fmod(phase(1) + phase_dt + M_PI, 2 * M_PI) - M_PI;

        // Push target j angles
        for (int i = 0; i < 18; i++)
        {
            policy_cmd->motorCmd[i].q = q_tar_n[i];
        }

        break;
    }
    }

    /*
    policy_cmd->Calibration = 0;

    // Test, Write a pd ctr with initial pos
    double init_q[18] = {0.00, 0.00, 0.785, -1.57, 0.785,
                         0.00, 0.00, 0.785, -1.57, 0.785,
                         0.00, 0.785, 0.000, -1.57,
                         0.00, 0.785, 0.000, -1.57};
    double kp_leg = 0.0; // 5.0;
    double kd_leg = 0.4;
    double kp_arm = 0.0; // 3.0;
    double kd_arm = 0.3;

    // Assign values for leg
    for (int i = 0; i < 10; i++)
    {
        policy_cmd->motorCmd[i].q = init_q[i];
        policy_cmd->motorCmd[i].dq = 0.0;
        policy_cmd->motorCmd[i].Kp = kp_leg;
        policy_cmd->motorCmd[i].Kd = kd_leg;
        policy_cmd->motorCmd[i].tau = 0.0;
    }
    // Assign valuse for arm
    for (int i = 10; i < 18; i++)
    {
        policy_cmd->motorCmd[i].q = init_q[i];
        policy_cmd->motorCmd[i].dq = 0.0;
        policy_cmd->motorCmd[i].Kp = kp_arm;
        policy_cmd->motorCmd[i].Kd = kd_arm;
        policy_cmd->motorCmd[i].tau = 0.0;
    }

    std::cout << "Main control loop ctrl" << std::endl;
    */

    // also print imu data to dubug
    std::cout << "IMU deubug - x " << policy_state->imu.rpy[0]
              << " y - " << policy_state->imu.rpy[1]
              << " z - " << policy_state->imu.rpy[2]
              << std::endl;
}

// convert mujoco cmd to motor cmd with leg ik, policy_cmd->low_cmd
void hw_wrapper::hw_ik()
{
    // Copy calibration straight through
    low_cmd->Calibration = policy_cmd->Calibration;

    // Legs 0–9 (2 legs × 5 joints each)
    for (int leg = 0; leg < 2; ++leg)
    {
        int base = leg * 5;

        // j0–j2: hips (direct copy + sign flip)
        for (int j = 0; j < 3; ++j)
        {
            int i = base + j;
            low_cmd->motorCmd[i].q = joint_dir[i] * policy_cmd->motorCmd[i].q;
            low_cmd->motorCmd[i].dq = joint_dir[i] * policy_cmd->motorCmd[i].dq;
            low_cmd->motorCmd[i].tau = joint_dir[i] * policy_cmd->motorCmd[i].tau;
            low_cmd->motorCmd[i].Kp = policy_cmd->motorCmd[i].Kp;
            low_cmd->motorCmd[i].Kd = policy_cmd->motorCmd[i].Kd;
        }

        // j3: knee (with offset+ratio transform)
        int i3 = base + 3;
        {
            double qd = policy_cmd->motorCmd[i3].q,
                   dd = policy_cmd->motorCmd[i3].dq,
                   tt = policy_cmd->motorCmd[i3].tau;
            double q_raw = (qd - knee_offset) * gear_ratio_knee + knee_offset;
            double dq_raw = dd * gear_ratio_knee;
            double t_raw = tt / gear_ratio_knee;

            low_cmd->motorCmd[i3].q = joint_dir[i3] * q_raw;
            low_cmd->motorCmd[i3].dq = joint_dir[i3] * dq_raw;
            low_cmd->motorCmd[i3].tau = joint_dir[i3] * t_raw;
            low_cmd->motorCmd[i3].Kp = policy_cmd->motorCmd[i3].Kp / (gear_ratio_knee * gear_ratio_knee);
            low_cmd->motorCmd[i3].Kd = policy_cmd->motorCmd[i3].Kd / (gear_ratio_knee * gear_ratio_knee);
        }

        // j4: ankle (dependent on both ankle & knee desired)
        int i4 = base + 4;
        {
            double qd3 = policy_cmd->motorCmd[i3].q,
                   dd3 = policy_cmd->motorCmd[i3].dq,
                   qd4 = policy_cmd->motorCmd[i4].q,
                   dd4 = policy_cmd->motorCmd[i4].dq;

            double q_raw = qd4 + qd3 - knee_offset;
            double dq_raw = dd4 + dd3;

            low_cmd->motorCmd[i4].q = joint_dir[i4] * q_raw;
            low_cmd->motorCmd[i4].dq = joint_dir[i4] * dq_raw;
            low_cmd->motorCmd[i4].tau = joint_dir[i4] * policy_cmd->motorCmd[i4].tau;
            low_cmd->motorCmd[i4].Kp = policy_cmd->motorCmd[i4].Kp;
            low_cmd->motorCmd[i4].Kd = policy_cmd->motorCmd[i4].Kd;
        }
    }

    // Arms j10–j17 (direct copy + sign flip)
    for (int i = 10; i < 18; ++i)
    {
        low_cmd->motorCmd[i].q = joint_dir[i] * policy_cmd->motorCmd[i].q;
        low_cmd->motorCmd[i].dq = joint_dir[i] * policy_cmd->motorCmd[i].dq;
        low_cmd->motorCmd[i].tau = joint_dir[i] * policy_cmd->motorCmd[i].tau;
        low_cmd->motorCmd[i].Kp = policy_cmd->motorCmd[i].Kp;
        low_cmd->motorCmd[i].Kd = policy_cmd->motorCmd[i].Kd;
    }
    // Overwrite last arm joint with reduction ratio
    low_cmd->motorCmd[13].q = joint_dir[13] * policy_cmd->motorCmd[13].q * gear_ratio_arm;
    low_cmd->motorCmd[13].dq = joint_dir[13] * policy_cmd->motorCmd[13].dq * gear_ratio_arm;
    low_cmd->motorCmd[13].tau = joint_dir[13] * policy_cmd->motorCmd[13].tau / gear_ratio_arm;

    low_cmd->motorCmd[17].q = joint_dir[17] * policy_cmd->motorCmd[17].q * gear_ratio_arm;
    low_cmd->motorCmd[17].dq = joint_dir[17] * policy_cmd->motorCmd[17].dq * gear_ratio_arm;
    low_cmd->motorCmd[17].tau = joint_dir[17] * policy_cmd->motorCmd[17].tau / gear_ratio_arm;
}

// convert motor state to mujoco state with leg fk, low_state->policy_state
void hw_wrapper::hw_fk()
{
    // Legs 0–9 (2 legs × 5 joints each)
    for (int leg = 0; leg < 2; ++leg)
    {
        int base = leg * 5;

        // j0–j2: hips (direct copy + sign flip)
        for (int j = 0; j < 3; ++j)
        {
            int i = base + j;
            policy_state->motorState[i].q = joint_dir[i] * low_state->motorState[i].q;
            policy_state->motorState[i].dq = joint_dir[i] * low_state->motorState[i].dq;
            policy_state->motorState[i].tauEst = joint_dir[i] * low_state->motorState[i].tauEst;
            policy_state->motorState[i].temperature = low_state->motorState[i].temperature;
            policy_state->motorState[i].error = low_state->motorState[i].error;
        }

        // j3: knee (invert offset+ratio)
        int i3 = base + 3;
        {
            double q_raw = low_state->motorState[i3].q,
                   dq_raw = low_state->motorState[i3].dq,
                   tt_raw = low_state->motorState[i3].tauEst;
            double q = (q_raw - knee_offset) / gear_ratio_knee + knee_offset;
            double dq = dq_raw / gear_ratio_knee;
            double t = tt_raw * gear_ratio_knee;

            policy_state->motorState[i3].q = joint_dir[i3] * q;
            policy_state->motorState[i3].dq = joint_dir[i3] * dq;
            policy_state->motorState[i3].tauEst = joint_dir[i3] * t;
            policy_state->motorState[i3].temperature = low_state->motorState[i3].temperature;
            policy_state->motorState[i3].error = low_state->motorState[i3].error;
        }

        // j4: ankle (depends on raw ankle minus knee-angle)
        int i4 = base + 4;
        {
            double q3 = (low_state->motorState[i3].q - knee_offset) / gear_ratio_knee + knee_offset;
            double q_raw = low_state->motorState[i4].q,
                   dq_raw = low_state->motorState[i4].dq,
                   tt_raw = low_state->motorState[i4].tauEst;
            double q = q_raw - q3 + knee_offset;
            double dq = dq_raw - ((low_state->motorState[i3].dq) / gear_ratio_knee);

            policy_state->motorState[i4].q = joint_dir[i4] * q;
            policy_state->motorState[i4].dq = joint_dir[i4] * dq;
            policy_state->motorState[i4].tauEst = joint_dir[i4] * tt_raw;
            policy_state->motorState[i4].temperature = low_state->motorState[i4].temperature;
            policy_state->motorState[i4].error = low_state->motorState[i4].error;
        }
    }

    // Arms j10–j17 (direct copy + sign flip)
    for (int i = 10; i < 18; ++i)
    {
        policy_state->motorState[i].q = joint_dir[i] * low_state->motorState[i].q;
        policy_state->motorState[i].dq = joint_dir[i] * low_state->motorState[i].dq;
        policy_state->motorState[i].tauEst = joint_dir[i] * low_state->motorState[i].tauEst;
        policy_state->motorState[i].temperature = low_state->motorState[i].temperature;
        policy_state->motorState[i].error = low_state->motorState[i].error;
    }

    // Overwrite last arm joint with reduction ratio
    policy_state->motorState[13].q = joint_dir[13] * low_state->motorState[13].q / gear_ratio_arm;
    policy_state->motorState[13].dq = joint_dir[13] * low_state->motorState[13].dq / gear_ratio_arm;
    policy_state->motorState[13].tauEst = joint_dir[13] * low_state->motorState[13].tauEst * gear_ratio_arm;

    policy_state->motorState[17].q = joint_dir[17] * low_state->motorState[17].q / gear_ratio_arm;
    policy_state->motorState[17].dq = joint_dir[17] * low_state->motorState[17].dq / gear_ratio_arm;
    policy_state->motorState[17].tauEst = joint_dir[17] * low_state->motorState[17].tauEst * gear_ratio_arm;

    // IMU (change some axis dir)
    for (int i = 0; i < 3; ++i)
    {
        policy_state->imu.rpy[i] = low_state->imu.rpy[i] * imu_dir[i];
        policy_state->imu.gyroscope[i] = low_state->imu.gyroscope[i] * imu_dir[i];
        policy_state->imu.accelerometer[i] = low_state->imu.accelerometer[i] * imu_dir[i];
    }
}