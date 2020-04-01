#include <ros/ros.h>
#include "tocabi_controller/tocabi_controller.h"
#include "tocabi_controller/terminal.h"
#include <std_msgs/String.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

volatile bool shutdown_tocabi_bool = false;

void terminate_signal(int sig)
{
    std::cout << "shutdown signal received " << std::endl;
    shutdown_tocabi = 1;
    shutdown_tocabi_bool = true;
}

int main(int argc, char **argv)
{
    signal(SIGINT, &terminate_signal);

    ros::init(argc, argv, "tocabi_controller");
    DataContainer dc;

    dc.nh.param<std::string>("/tocabi_controller/run_mode", dc.mode, "default");
    dc.nh.param<std::string>("/tocabi_controller/ifname", dc.ifname, "enp0s31f6");
    dc.nh.param("/tocabi_controller/ctime", dc.ctime, 500);
    dc.nh.param("/tocabi_controller/pub_mode", dc.pubmode, true);
    dc.nh.param<std::string>("/tocabi_controller/sim_mode", dc.sim_mode, "torque");
    dc.nh.getParam("/tocabi_controller/Kp",dc.tocabi_.vector_kp);
    dc.nh.getParam("/tocabi_controller/Kv",dc.tocabi_.vector_kv);
    
    dc.statusPub = dc.nh.advertise<std_msgs::String>("/tocabi/guilog", 1000);
    std::string strr("hello guilog");
    dc.statusPubMsg.data = strr;

    bool simulation = true;
    dc.dym_hz = 500;
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));

    Tui tui(dc);
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL)
    {
        homedir = getpwuid(getuid())->pw_dir;
    }

    //set Home directory.
    //    /home/{user_name}/tocabi_data
    dc.homedir = std::string(homedir) + "/tocabi_data";

    std::cout << "Tocabi Controller : ";

    if (dc.mode == "simulation")
    {
        std::cout << "Simulation Mode " << std::endl;

        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        TocabiController rc(dc, stm, dym);
        std::thread thread[4];
        thread[0] = std::thread(&TocabiController::stateThread, &rc);
        thread[1] = std::thread(&TocabiController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&TocabiController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&TocabiController::tuiThread, &rc);

        for (int i = 0; i < 4; i++)
        {
            thread[i].join();
        }
    }
    else if(dc.mode == "simulationposition")
    {
        std::cout << "Simulation Mode position" << std::endl;

        dc.simulationMode = true;
        
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        TocabiController rc(dc, stm, dym);
        std::thread thread[4];
        thread[0] = std::thread(&TocabiController::stateThread, &rc);
        thread[1] = std::thread(&TocabiController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&TocabiController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&TocabiController::tuiThread, &rc);

        for (int i = 0; i < 4; i++)
        {
            thread[i].join();
        }
    }
    else if (dc.mode == "realrobot")
    {
        std::cout << "RealRobot Mode" << std::endl;

        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        TocabiController tc(dc, rtm, dym);

        //dc.statusPubMsg.data=std::string("RealrobotMode");
        //dc.statusPub.publish(dc.statusPubMsg);

        //Total Number of Thread
        int thread_num = 8;

        //Total Number of Real-Time Thread
        int rt_thread_num = 2;

        std::thread thread[thread_num];
        int t_id = 0;

        //RT THREAD FIRST!
        thread[t_id++] = std::thread(&RealRobotInterface::ethercatThread, &rtm);
        thread[t_id++] = std::thread(&TocabiController::stateThread, &tc);

        //EthercatElmo Management Thread
        thread[t_id++] = std::thread(&RealRobotInterface::ethercatCheck, &rtm);

        //Sensor Data Management Thread
        thread[t_id++] = std::thread(&RealRobotInterface::imuThread, &rtm);
        thread[t_id++] = std::thread(&RealRobotInterface::ftsensorThread, &rtm);

        //Robot Controller Threadx
        thread[t_id++] = std::thread(&TocabiController::dynamicsThreadHigh, &tc);
        thread[t_id++] = std::thread(&TocabiController::dynamicsThreadLow, &tc);

        //For Additional functions ..
        thread[t_id++] = std::thread(&TocabiController::tuiThread, &tc);

        //For RealTime Thread
        sched_param sch;
        int policy;
        int priority[rt_thread_num] = {39, 38};
        for (int i = 0; i < rt_thread_num; i++)
        {

            pthread_getschedparam(thread[i].native_handle(), &policy, &sch);

            sch.sched_priority = priority[i];
            if (pthread_setschedparam(thread[i].native_handle(), SCHED_FIFO, &sch))
            {
                std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
            }
        }

        for (int i = 0; i < thread_num; i++)
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
        }

        for (int i = 0; i < thread_num; i++)
        {
            thread[i].join();
        }
    }
    else if (dc.mode == "testmode")
    {
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        TocabiController tc(dc, stm, dym);
        std::cout << "testmode" << std::endl;

        std::thread thread[2];
        thread[0] = std::thread(&StateManager::testThread, &stm);

        //thread[1] = std::thread(&DynamicsManager::testThread, &dym);

        thread[1] = std::thread(&TocabiController::tuiThread, &tc);

        for (int i = 0; i < 2; i++)
        {
            thread[i].join();
        }
    }
    else if (dc.mode == "ftsensor")
    {
        std::cout << "FT Test Mode " << std::endl;

        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        TocabiController tc(dc, rtm, dym);

        std::thread thread[1];
        //thread[0] = std::thread(&StateManager::testThread, &stm);

        //thread[1] = std::thread(&DynamicsManager::testThread, &dym);

        //thread[1] = std::thread(&TocabiController::tuiThread, &tc);

        thread[0] = std::thread(&RealRobotInterface::ftsensorThread, &rtm);

        for (int i = 0; i < 1; i++)
        {
            thread[i].join();
        }
    }
    std::cout << cgreen << "All threads are completely terminated !" << creset << std::endl;
    return 0;
}