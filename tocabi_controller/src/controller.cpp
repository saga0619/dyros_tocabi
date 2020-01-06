#include <ros/ros.h>
#include "tocabi_controller/tocabi_controller.h"
#include "tocabi_controller/terminal.h"
void terminate_signal(int sig)
{
    shutdown_tocabi = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, terminate_signal);

    ros::init(argc, argv, "tocabi_controller");
    DataContainer dc;

    dc.nh.param<std::string>("/tocabi_controller/run_mode", dc.mode, "default");
    dc.nh.param("/tocabi_controller/ncurse", dc.ncurse_mode, false);
    dc.nh.param<std::string>("/tocabi_controller/ifname", dc.ifname, "enp0s31f6");
    dc.nh.param("/tocabi_controller/ctime", dc.ctime, 250);
    dc.nh.param("/tocabi_controller/pub_mode", dc.pubmode, false);

    bool simulation = true;
    dc.dym_hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));

    Tui tui(dc);

    if (!dc.ncurse_mode)
        endwin();
    std::cout << "Tocabi Controller : ";
    if (dc.mode == "simulation")
    {
        std::cout << "Simulation Mode " << std::endl;
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        TocabiController rc(dc, stm, dym);
/*
        std::thread thread[4];
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
        }*/
    }
    else if (dc.mode == "realrobot")
    {
        std::cout << "RealRobot Mode " << std::endl;
        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        TocabiController tc(dc, rtm, dym);

        std::thread thread[8];

        //EthercatElmo Management Thread
        thread[0] = std::thread(&RealRobotInterface::ethercatCheck, &rtm);
        thread[1] = std::thread(&RealRobotInterface::ethercatThread, &rtm);

        //Sensor Data Management Thread
        thread[2] = std::thread(&RealRobotInterface::imuThread, &rtm);
        thread[3] = std::thread(&RealRobotInterface::ftsensorThread, &rtm);

        //Robot Controller Thread
        thread[4] = std::thread(&TocabiController::stateThread, &tc);
        thread[5] = std::thread(&TocabiController::dynamicsThreadHigh, &tc);
        thread[6] = std::thread(&TocabiController::dynamicsThreadLow, &tc);

        //For Additional functions ..
        thread[7] = std::thread(&TocabiController::tuiThread, &tc);

        //For RealTime Thread

        sched_param sch;
        int policy;
        int priority = 40;

        int rt_thread_num = 4;
        int rt_thread_id[rt_thread_num] = {0, 2, 3, 4};
        std::string thread_name[] = {"ethercatThread", "ImuThread", "FTsensor", "StateThread"};

        for (int i = 0; i < rt_thread_num; i++)
        {

            pthread_getschedparam(thread[rt_thread_id[i]].native_handle(), &policy, &sch);
            sch.sched_priority = priority;
            if (pthread_setschedparam(thread[rt_thread_id[i]].native_handle(), SCHED_FIFO, &sch))
            {
                std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
            }
            else
            {
                std::cout << thread_name[i] << " setsched success !" << std::endl;
            }
        }

        for (int i = 1; i < 8; i++)
        {
            thread[i].join();
        }
    }

    else if (dc.mode == "ethercattest")
    {
        std::cout << "EtherCat Test Mode " << std::endl;

        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        TocabiController tc(dc, rtm, dym);

        //Total Number of Thread
        int thread_num = 3;

        //Total Number of Real-Time Thread
        int rt_thread_num = 1;

        std::thread thread[thread_num];
        int t_id = 0;

        //RT THREAD FIRST!
        thread[t_id++] = std::thread(&RealRobotInterface::ethercatThread, &rtm);

        //EthercatElmo Management Thread
        thread[t_id++] = std::thread(&RealRobotInterface::ethercatCheck, &rtm);

        //Sensor Data Management Thread
        //thread[2] = std::thread(&RealRobotInterface::imuThread, &rtm);
        //thread[3] = std::thread(&RealRobotInterface::ftsensorThread, &rtm);

        //Robot Controller Thread
        //thread[4] = std::thread(&TocabiController::stateThread, &tc);
        //thread[5] = std::thread(&TocabiController::dynamicsThreadHigh, &tc);
        //thread[6] = std::thread(&TocabiController::dynamicsThreadLow, &tc);

        //For Additional functions ..
        thread[t_id++] = std::thread(&TocabiController::tuiThread, &tc);

        //For RealTime Thread
        sched_param sch;
        int policy;
        int priority = 39;
        for (int i = 0; i < rt_thread_num; i++)
        {

            pthread_getschedparam(thread[i].native_handle(), &policy, &sch);

            sch.sched_priority = priority;
            if (pthread_setschedparam(thread[i].native_handle(), SCHED_FIFO, &sch))
            {
                std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
            }
            else
            {
                std::cout << "Thread #" << i << " : setsched success !" << std::endl;
            }
        }

        for (int i = 0; i < thread_num; i++)
        {
            thread[i].join();
        }
        std::cout << cgreen << "EthercatTest Mode :: All threads are completely terminated !" << creset << std::endl;
    }
    return 0;
}
