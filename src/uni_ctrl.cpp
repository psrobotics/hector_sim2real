#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include <thread>
#include <chrono>
#include <atomic>
#include <termios.h>

#include <lcm/lcm-cpp.hpp>
#include "../include/interface/IOSDK.h"
#include "../include/Custom_SDK/include/low_cmd_t.hpp"
#include "../include/Custom_SDK/include/low_state_t.hpp"

#include "../include/interface/hw_wrapper.hpp"
#include "../include/interface/t265.hpp"

using namespace UNITREE_LEGGED_SDK;

std::atomic<bool> running(true);

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
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

void keyboard_input_loop(hw_wrapper *wrapper)
{
    std::cout << "\nKeyboard Controls:" << std::endl;
    std::cout << "  'd': Damping state" << std::endl;
    std::cout << "  's': Standing state" << std::endl;
    std::cout << "  'p': Policy state" << std::endl;
    std::cout << "  'w'/'x': Adjust forward command" << std::endl;

    while (running)
    {
        int key = getch();
        switch (key)
        {
        case 'd':
            wrapper->switch_state(ControlState::DAMPING);
            break;
        case 's':
            wrapper->switch_state(ControlState::STANDING);
            break;
        case 'p':
            wrapper->switch_state(ControlState::POLICY);
            break;
        case 'w':
            wrapper->twist_command[0] += 0.5;
            break;
        case 'x':
            wrapper->twist_command[0] -= 0.5;
            break;
        }
    }
}

int main()
{
    set_process_scheduler();

    double ctrl_dt = 0.02;      // 50hz
    double lowlevel_dt = 0.001; // 1000hz

    int robot_id = 4;     // AlienGo=1, A1=2, Biped=4
    int cmd_panel_id = 2; // Wireless=1, keyboard=2

    //t265_wrapper t265;

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // (since C++11) use brace‐initialization
    hw_wrapper wrapper{};
    wrapper.state = ControlState::DAMPING;

    LoopFunc loop_ctrl("control_loop", ctrl_dt, boost::bind(&hw_wrapper::ctrl_loop, &wrapper));
    LoopFunc loop_hw_send("udp_send", lowlevel_dt, boost::bind(&hw_wrapper::hw_send, &wrapper));
    LoopFunc loop_hw_recv("udp_recv", lowlevel_dt, boost::bind(&hw_wrapper::hw_recv, &wrapper));
    LoopFunc keyboard("keyboard", ctrl_dt, boost::bind(&keyboard_input_loop, &wrapper));

    loop_hw_send.start();
    loop_hw_recv.start();
    loop_ctrl.start();
    keyboard.start();

    while (true)
    {
        sleep(10);
    }

    return 0;
}
