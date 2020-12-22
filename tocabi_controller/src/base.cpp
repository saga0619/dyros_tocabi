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
    dc.nh.param<std::string>("/tocabi_controller/ifname2", dc.ifname2, "enp0s31f6");
    dc.nh.param("/tocabi_controller/ctime", dc.ctime, 500);
    dc.nh.param("/tocabi_controller/pub_mode", dc.pubmode, true);
    dc.nh.param<std::string>("/tocabi_controller/sim_mode", dc.sim_mode, "torque");
    dc.nh.getParam("/tocabi_controller/Kp", dc.tocabi_.vector_kp);
    dc.nh.getParam("/tocabi_controller/Kv", dc.tocabi_.vector_kv);
    dc.nh.getParam("/tocabi_controller/vellimit", dc.safety_limit);
    dc.nh.getParam("/tocabi_controller/NM2CNT", dc.tocabi_.vector_NM2CNT);
    dc.nh.getParam("/tocabi_controller/opvellimit", dc.tocabi_.com_vel_limit);
    dc.nh.getParam("/tocabi_controller/opacclimit", dc.tocabi_.com_acc_limit);

    dc.statusPub = dc.nh.advertise<std_msgs::String>("/tocabi/guilog", 1000);
    std::string strr("hello guilog");
    dc.statusPubMsg.data = strr;

    dc.rgbPub = dc.nh.advertise<std_msgs::Int32MultiArray>("/rgbled_topic", 1000);

    dc.rgbPubMsg.data.resize(18);
    dc.rgbPubMsg_before.data.resize(18);
    for (int i = 0; i < 18; i++)
    {
        dc.rgbPubMsg.data[i] = 0;
    }
    std::cout << "pub" << std::endl;

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

        std::thread thread[5];
        thread[0] = std::thread(&TocabiController::stateThread, &rc);
        thread[1] = std::thread(&TocabiController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&TocabiController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&TocabiController::trajectoryplannar, &rc);
        thread[4] = std::thread(&TocabiController::tuiThread, &rc);

        for (int i = 0; i < 5; i++)
        {
            thread[i].join();
        }
    }
    else if (dc.mode == "simulationposition")
    {
        std::cout << "Simulation Mode position" << std::endl;

        dc.simulationMode = true;

        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        TocabiController rc(dc, stm, dym);
        std::thread thread[5];
        thread[0] = std::thread(&TocabiController::stateThread, &rc);
        thread[1] = std::thread(&TocabiController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&TocabiController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&TocabiController::trajectoryplannar, &rc);
        thread[4] = std::thread(&TocabiController::tuiThread, &rc);

        for (int i = 0; i < 5; i++)
        {
            thread[i].join();
        }
    }
#ifdef COMPILE_REALROBOT
    else if (dc.mode == "realrobot")
    {
        std::cout << "RealRobot Mode" << std::endl;

        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        TocabiController tc(dc, rtm, dym);

        //dc.statusPubMsg.data=std::string("RealrobotMode");
        //dc.statusPub.publish(dc.statusPubMsg);

        //Total Number of Thread
        int thread_num = 9;

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
        //thread[t_id++] = std::thread(&RealRobotInterface::handftsensorThread, &rtm);

        //Robot Controller Threadx
        thread[t_id++] = std::thread(&TocabiController::dynamicsThreadHigh, &tc);
        thread[t_id++] = std::thread(&TocabiController::dynamicsThreadLow, &tc);
        thread[t_id++] = std::thread(&TocabiController::trajectoryplannar, &tc);

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

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(7, &cpuset);
        
        //Check Thread cpu usage
        cpu_set_t cst[7];
        for (int i = 0; i < 7; i++)
        {
            CPU_ZERO(&cst[i]);
            CPU_SET(i, &cst[i]);
        }
        /*
        for (int i = 0; i < 4; i++)
        {
            if (pthread_setaffinity_np(thread[i + 2].native_handle(), sizeof(cst[0]), &cst[0]))
            {
                std::cout << "Failed to setaffinity: " << std::strerror(errno) << std::endl;
            }
        }

        pthread_setaffinity_np(thread[1].native_handle(), sizeof(cst[1]), &cst[1]);
        pthread_setaffinity_np(thread[6].native_handle(), sizeof(cst[2]), &cst[2]);

        pthread_setaffinity_np(thread[7].native_handle(), sizeof(cst[3]), &cst[3]);
        pthread_setaffinity_np(thread[8].native_handle(), sizeof(cst[4]), &cst[4]); 
        */


        //sched_setaffinity(getpid(),sizeof(cpuset),&cpuset);
        if (pthread_setaffinity_np(thread[0].native_handle(), sizeof(cpuset), &cpuset))
        {
            std::cout << "Failed to setaffinity: " << std::strerror(errno) << std::endl;
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
        dc.connected = true;
        std::thread thread[3];

        thread[0] = std::thread(&StateManager::testThread, &stm);
        thread[1] = std::thread(&TocabiController::testThread, &tc);
        thread[2] = std::thread(&TocabiController::tuiThread, &tc);

        for (int i = 0; i < 3; i++)
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

        thread[0] = std::thread(&RealRobotInterface::ftsensorThread, &rtm);

        //  thread[1] = std::thread(&RealRobotInterface::handftsensorThread, &rtm);

        for (int i = 0; i < 1; i++)
        {
            thread[i].join();
        }
    }
#endif
    dc.rgbPubMsg.data = {0, 64, 0, 0, 64, 0, 0, 64, 0, 0, 64, 0, 0, 64, 0, 0, 64, 0};
    dc.rgbPub.publish(dc.rgbPubMsg);
    std::cout << cgreen << "All threads are completely terminated !" << creset << std::endl;
    return 0;
}