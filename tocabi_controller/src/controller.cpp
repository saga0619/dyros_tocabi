#include <ros/ros.h>
#include "tocabi_controller/tocabi_controller.h"
#include "tocabi_controller/terminal.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tocabi_controller");
    DataContainer dc;

    dc.nh.param<std::string>("/tocabi_controller/run_mode", dc.mode, "default");
    dc.nh.param("/tocabi_controller/ncurse", dc.ncurse_mode, true);
    dc.nh.param<std::string>("/tocabi_controller/ifname", dc.ifname, "enp0s31f6");
    dc.nh.param("/tocabi_controller/ctime", dc.ctime, 250);
    dc.nh.param("/tocabi_controller/pub_mode", dc.pubmode, false);

    Tui tui(dc);
    bool simulation = true;
    dc.dym_hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));

    std::thread thread[4];

    pthread_t pthr[4];

    if (dc.mode == "simulation")
    {
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        TocabiController rc(dc, stm, dym);

        thread[0] = std::thread(&TocabiController::stateThread, &rc);
        thread[1] = std::thread(&TocabiController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&TocabiController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&TocabiController::tuiThread, &rc);

        sched_param sch;
        int policy;
        for (int i = 0; i < 4; i++)
        {
            pthread_getschedparam(thread[i].native_handle(), &policy, &sch);
            sch.sched_priority = 49;
            if (pthread_setschedparam(thread[i].native_handle(), SCHED_FIFO, &sch))
            {
                std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
            }
        }

        for (int i = 0; i < 3; i++)
        {
            thread[i].join();
            rprint_sol(dc.ncurse_mode, 3 + 2 * i, 35, "Thread %d End", i);
        }
    }
    else if (dc.mode == "realrobot")
    {
        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        TocabiController rc(dc, rtm, dym);

        //EthercatElmo Management Thread
        osal_thread_create(&pthr[0], NULL, (void *)&RealRobotInterface::ethercatCheck, &rtm);
        osal_thread_create_rt(&pthr[1], NULL, (void *)&RealRobotInterface::ethercatThread, &rtm);


        //Sensor Data Management Thread 


        //Robot Controller Thread
        thread[0] = std::thread(&TocabiController::stateThread, &rc);
        thread[1] = std::thread(&TocabiController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&TocabiController::dynamicsThreadLow, &rc);

        //For Additional functions ..
        thread[3] = std::thread(&TocabiController::tuiThread, &rc);

        pthread_join(pthr[1], NULL);
        for (int i = 1; i < 3; i++)
        {
            thread[i].join();
            rprint_sol(dc.ncurse_mode, 3 + 2 * i, 35, "Thread %d End", i);
        }
    }
    return 0;
}