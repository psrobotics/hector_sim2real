#include "../include/interface/t265.hpp"
#include <iostream>
#include <cmath>
#include "eigen3/Eigen/Dense"

int main()
{
    try
    {
        t265_wrapper t265;
        t265.start();

        for (int i = 0; i < 20000; ++i)
        {
            // Either poll() or wait_and_update()
            t265.wait_and_update();

            if (auto q = t265.quaternion_wxyz())
            {
                auto euler = t265.euler_xyz();
                std::cout << "roll: " << euler[0] << "\n";
                std::cout << "pitch: " << euler[1] << "\n";
                std::cout << "yaw: " << euler[2] << "\n";

                std::cout<<"quant ori - "<<q->w()<< q->x()<< q->y()<< q->z()<<std::endl;

                Eigen::Quaterniond qq(q->w(), q->x(), q->y(), q->z());
                Eigen::Matrix3d imu_xmat = qq.matrix();
                Eigen::Vector3d gravity_vec = imu_xmat.transpose() * Eigen::Vector3d(0, 0, -1);
                std::cout << "Gravity vec: " << gravity_vec << "\n";
            }
            if (auto g = t265.gyro_xyz())
            {
                std::cout << "Gyro (rad/s):   " << g->transpose() << "\n";
            }
        }

        t265.stop();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
    }
    return 0;
}
