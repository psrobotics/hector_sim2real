#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

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

    Safety *safe;
    LowlevelCmd *low_cmd;
    LowlevelState *low_state;
    IOInterface *io_interface;
};

custom_wrapper::custom_wrapper()
{
    low_cmd = new LowlevelCmd();
    low_state = new LowlevelState();
    safe = new Safety(LeggedType::A1);
    // Init with keyboard ctrl
    io_interface = new IOSDK(LeggedType::A1, 2);
}

custom_wrapper::~custom_wrapper()
{
    delete low_cmd;
    delete low_state;
    delete safe;
    delete io_interface;
}

void custom_wrapper::hw_send()
{
    io_interface->sendUDP(low_cmd);
    std::cout<<"Low level send loop"<<std::endl;
}

void custom_wrapper::hw_recv()
{
    io_interface->ReceiveUDP(low_state);
    std::cout<<"Low level recv loop"<<std::endl;
}

void custom_wrapper::ctrl_loop()
{
    low_cmd->Calibration = 0;

    // Legs
    for (int i = 0; i < 10; i++)
    {
        low_cmd->motorCmd[i].q = 0;
        low_cmd->motorCmd[i].dq = 0;
        low_cmd->motorCmd[i].Kp = 0.0;
        low_cmd->motorCmd[i].Kd = 0.0;
        low_cmd->motorCmd[i].tau = 0;
    }
    // Just arms
    for (int i = 10; i < 18; i++)
    {
        low_cmd->motorCmd[i].q = 0;
        low_cmd->motorCmd[i].dq = 0;
        low_cmd->motorCmd[i].Kp = 1.0;
        low_cmd->motorCmd[i].Kd = 0.05;
        low_cmd->motorCmd[i].tau = 0;
    }

    // Hip r
    low_cmd->motorCmd[2].q = 1.0;
    low_cmd->motorCmd[2].dq = 0;
    low_cmd->motorCmd[2].Kp = 3.5;
    low_cmd->motorCmd[2].Kd = 0.1;
    low_cmd->motorCmd[2].tau = 0;
    // Hip l
    low_cmd->motorCmd[7].q = 1.0;
    low_cmd->motorCmd[7].dq = 0;
    low_cmd->motorCmd[7].Kp = 3.5;
    low_cmd->motorCmd[7].Kd = 0.1;
    low_cmd->motorCmd[7].tau = 0;

    std::cout<<"Main control loop ctrl"<<std::endl;
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

    loop_hw_send.start();
    loop_hw_recv.start();
    loop_ctrl.start();

    while(true)
    {
        sleep(10);
    }

    return 0;
}
