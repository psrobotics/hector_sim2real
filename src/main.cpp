#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "../include/interface/IOSDK.h"

using namespace UNITREE_LEGGED_SDK;

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

void setProcessScheduler()
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
    setProcessScheduler();

    double dt = 0.001;
    int robot_id = 4;     // AlienGo=1, A1=2, Biped=4
    int cmd_panel_id = 2; // Wireless=1, keyboard=2

    IOInterface *ioInter;
    if (robot_id == 4)
    {
        ioInter = new IOSDK(LeggedType::A1, cmd_panel_id);
        std::cout << "Init IOsdk for Hector" << std::endl;
    }
    else
    {
        std::cerr << "Unknown HW Type" << std::endl;
    }


    LowlevelCmd *lowCmd = new LowlevelCmd();
    LowlevelState *lowState = new LowlevelState();


    while (1)
    {

        usleep(100);
        lowCmd->Calibration = 0;

        // Legs
        for (int i = 0; i < 10; i++)
        {
            lowCmd->motorCmd[i].q = 0;
            lowCmd->motorCmd[i].dq = 0;
            lowCmd->motorCmd[i].Kp = 0.0;
            lowCmd->motorCmd[i].Kd = 0.0;
            lowCmd->motorCmd[i].tau = 0;
        }

        //just arms
        for (int i = 10; i < 18; i++)
        {
            lowCmd->motorCmd[i].q = 0;
            lowCmd->motorCmd[i].dq = 0;
            lowCmd->motorCmd[i].Kp = 1.0;
            lowCmd->motorCmd[i].Kd = 0.05;
            lowCmd->motorCmd[i].tau = 0;
        }

            // Hip r
            lowCmd->motorCmd[2].q = 1.0;
            lowCmd->motorCmd[2].dq = 0;
            lowCmd->motorCmd[2].Kp = 3.5;
            lowCmd->motorCmd[2].Kd = 0.1;
            lowCmd->motorCmd[2].tau = 0;

            // Hip l
            lowCmd->motorCmd[7].q = 1.0;
            lowCmd->motorCmd[7].dq = 0;
            lowCmd->motorCmd[7].Kp = 3.5;
            lowCmd->motorCmd[7].Kd = 0.1;
            lowCmd->motorCmd[7].tau = 0;

        ioInter->sendUDP(lowCmd);
        std::cout << "Sent Mot states" << std::endl;
        usleep(100);

        ioInter->ReceiveUDP(lowState);
        std::cout << "Rev states - " << lowState->motorState[2].q << std::endl;
    }

    delete lowState;
    delete ioInter;

    return 0;
}
