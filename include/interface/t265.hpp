// t265_pose_gyro.hpp
#pragma once
#include <librealsense2/rs.hpp>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <atomic>
#include <optional>
#include <stdexcept>
#include <iostream>

inline Eigen::Matrix3d make_R_UC()
{
    Eigen::Matrix3d R;
    R.col(0) = Eigen::Vector3d(0, 0, -1); 
    R.col(1) = Eigen::Vector3d(-1, 0, 0);
    R.col(2) = Eigen::Vector3d(0, 1, 0);

    return R;
}

inline Eigen::Vector3d remap_vector_C_to_U(const Eigen::Vector3d &v_C)
{
    static const Eigen::Matrix3d R_UC = make_R_UC();
    return R_UC * v_C;
}

class t265_wrapper
{
public:
    t265_wrapper() : running_(false) {}

    // Start pipeline (opens device + streams)
    void start()
    {
        if (running_)
            return;

        rs2::config cfg;
        // T265: 6DoF pose stream
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        // IMU gyro stream (angular velocity, rad/s)
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        //  Accel is available too:
        // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ);

        pipe_ = rs2::pipeline{};
        profile_ = pipe_.start(cfg);
        running_ = true;
    }

    // Blocking update with timeout_ms; returns true if updated.
    bool wait_and_update(unsigned timeout_ms = 1000)
    {
        if (!running_)
            return false;
        auto fs = pipe_.wait_for_frames(timeout_ms);
        bool any = false;
        for (auto &&f : fs)
        {
            if (auto pf = f.as<rs2::pose_frame>())
            {
                auto p = pf.get_pose_data();
                auto quat_c = Eigen::Quaterniond(p.rotation.w, -p.rotation.z, -p.rotation.x, p.rotation.y);
                std::cout<<quat_c<<std::endl;
                //quat_wxyz_ = remap_quaternion_WC_to_WU(quat_c);
                quat_wxyz_ = quat_c;

                quate2euler();

                quat_ts_ = pf.get_timestamp() * 1e-3; // ms->s
                any = true;
            }
            if (auto mf = f.as<rs2::motion_frame>())
            {
                if (mf.get_profile().stream_type() == RS2_STREAM_GYRO)
                {
                    rs2_vector v = mf.get_motion_data();
                    auto gyro_c = Eigen::Vector3d(v.x, v.y, v.z);
                    gyro_xyz_ = remap_vector_C_to_U(gyro_c);
                    gyro_ts_ = mf.get_timestamp() * 1e-3;
                    any = true;
                }
            }
        }
        return any;
    }

    void quate2euler()
    {
        double w = quat_wxyz_->w();
        double x = quat_wxyz_->x();
        double y = quat_wxyz_->y();
        double z = quat_wxyz_->z();

        double pitch = std::asin(2.0 * (x*z - w*y));
        double roll = std::atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z);
        double yaw = std::atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z);

        euler_xyz_ << -1.0*roll, -1.0*pitch, yaw;
    }

    // Getters (std::optional to indicate availability)
    std::optional<Eigen::Quaterniond> quaternion_wxyz() const
    {
        if (quat_wxyz_.has_value())
            return quat_wxyz_;
        return std::nullopt;
    }

    std::optional<Eigen::Vector3d> gyro_xyz() const
    {
        if (gyro_xyz_.has_value())
            return gyro_xyz_;
        return std::nullopt;
    }

    Eigen::Vector3d euler_xyz()
    {
        return euler_xyz_;
    }

        // Stop pipeline
    void stop()
    {
        if (!running_)
            return;
        pipe_.stop();
        running_ = false;
    }

    ~t265_wrapper()
    {
        if (running_)
        {
            try
            {
                pipe_.stop();
            }
            catch (...)
            {
            }
        }
    }

    std::optional<double> quaternion_timestamp_s() const { return quat_ts_; }
    std::optional<double> gyro_timestamp_s() const { return gyro_ts_; }

private:
    rs2::pipeline pipe_;
    rs2::pipeline_profile profile_;
    std::atomic<bool> running_;

    std::optional<Eigen::Quaterniond> quat_wxyz_;
    std::optional<Eigen::Vector3d> gyro_xyz_;
    Eigen::Vector3d euler_xyz_;
    std::optional<double> quat_ts_;
    std::optional<double> gyro_ts_;
};
