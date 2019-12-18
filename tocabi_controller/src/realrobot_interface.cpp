#include "tocabi_controller/realrobot_interface.h"

std::mutex mtx_torque_command;
std::mutex mtx_q;

double rising_time = 3.0;

RealRobotInterface::RealRobotInterface(DataContainer &dc_global) : dc(dc_global), StateManager(dc_global)
{
    imuSubscriber = dc.nh.subscribe("/imu/data", 1, &RealRobotInterface::ImuCallback, this);
    gainSubscriber = dc.nh.subscribe("/tocabi_controller/gain_command", 100, &RealRobotInterface::gainCallbak, this);

    printf("Starting red ethercat master\n");

    torque_desired.setZero();

    positionElmo.setZero();
    velocityElmo.setZero();
    torqueElmo.setZero();
    torqueDemandElmo.setZero();
    positionDesiredElmo.setZero();
    velocityDesiredElmo.setZero();
    torqueDesiredElmo.setZero();

    torqueDesiredController.setZero();
    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc.currentGain(i) = NM2CNT[i];
    }
}

void RealRobotInterface::ethercatThread_()
{
    char IOmap[4096];

    int wkc_count;
    boolean needlf = FALSE;
    boolean inOP = FALSE;

    struct timespec current, begin, time;
    double elapsed = 0.0, elapsed_sum = 0.0, elapsed_avg = 0.0, elapsed_var = 0.0, prev = 0.0, now = 0.0, current_time = 0.0, begin_time = 0.0;
    double elapsed_time[10000] = {0.0};
    static int elapsed_cnt = 0, max_cnt = 0, min_cnt = 0;
    double elapsed_min = 210000000.0, elapsed_max = 0.0;
    double time_mem[10000] = {0.0};

    bool reachedInitial[MODEL_DOF] = {false};

    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 49;

    pid_t pid = getpid();
    if (sched_setscheduler(pid, SCHED_FIFO, &schedp) == -1)
    {
        perror("sched_setscheduler(SCHED_FIFO)");
        exit(EXIT_FAILURE);
    }
    const char *ifname = dc.ifname.c_str();
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        /* network discovery */
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("%d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num

            /** CompleteAccess disabled for Elmo driver */
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                printf("Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                uint16 map_1c12[2] = {0x0001, 0x1605};
                uint16 map_1c13[4] = {0x0003, 0x1a04, 0x1a11, 0x1a12};
                int os;
                os = sizeof(map_1c12);
                ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, map_1c12, EC_TIMEOUTRXM);
                os = sizeof(map_1c13);
                ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, map_1c13, EC_TIMEOUTRXM);
            }

            /** if CA disable => automapping works */
            ec_config_map(&IOmap);

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            printf("Request operational state for all slaves\n");
            int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);

            int wait_cnt = 40;

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
            } while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                //printf("Operational state reached for all slaves.\n");
                rprint(dc, 15, 5, "Operational state reached for all slaves.");
                wkc_count = 0;
                inOP = TRUE;
                rprint(dc, 16, 5, "Enable red controller threads ... ");
                dc.connected = true;
                /* cyclic loop */
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                    rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                }

                struct timespec ts;
                int64 cycletime;

                cycletime = dc.ctime * 1000; /* cycletime in ns */

                clock_gettime(CLOCK_MONOTONIC, &ts);
                clock_gettime(CLOCK_MONOTONIC, &begin);
                prev = begin.tv_sec;
                prev += begin.tv_nsec / 1000000000.0;

                double to_ratio, to_calib;

                while (!dc.shutdown && ros::ok())
                {
                    /* wait to cycle start */
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
                    //std::cout<<"control time : " << control_time_<< std::endl;
                    /** PDO I/O refresh */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(250);

                    if (wkc >= expectedWKC)
                    {

                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                            {
                                reachedInitial[slave - 1] = true;
                            }
                        }
                        if (Walking_State == 0)
                        {
                            mtx_q.lock();
                            for (int slave = 1; slave <= ec_slavecount; slave++)
                            {
                                q_init_(slave - 1) = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * Dr[slave - 1];
                                positionElmo(slave - 1) = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * Dr[slave - 1];
                                velocityElmo(slave - 1) =
                                    (((int32_t)ec_slave[slave].inputs[14]) +
                                     ((int32_t)ec_slave[slave].inputs[15] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[16] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[17] << 24)) *
                                    CNT2RAD[slave - 1] * Dr[slave - 1];
                                dc.elmo_cnt = 0;
                            }
                            mtx_q.unlock();
                            ElmoConnected = true;
                            Walking_State = 1;
                        }
                        else if (Walking_State == 1)
                        {
                            torqueDesiredElmo = getCommand();
                            positionDesiredElmo = dc.positionDesired;
                            mtx_q.lock();
                            for (int slave = 1; slave <= ec_slavecount; slave++)
                            {
                                if (reachedInitial[slave - 1])
                                {
                                    positionElmo(slave - 1) = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * Dr[slave - 1];
                                    velocityElmo(slave - 1) =
                                        (((int32_t)ec_slave[slave].inputs[14]) +
                                         ((int32_t)ec_slave[slave].inputs[15] << 8) +
                                         ((int32_t)ec_slave[slave].inputs[16] << 16) +
                                         ((int32_t)ec_slave[slave].inputs[17] << 24)) *
                                        CNT2RAD[slave - 1] * Dr[slave - 1];

                                    torqueDemandElmo(slave - 1) =
                                        (int16_t)((ec_slave[slave].inputs[18]) +
                                                  (ec_slave[slave].inputs[19] << 8)) *
                                        Dr[slave - 1];
                                    torqueElmo(slave - 1) = rxPDO[slave - 1]->torqueActualValue * Dr[slave - 1];

                                    dc.torqueElmo = torqueElmo;
                                    dc.torqueDemandElmo = torqueDemandElmo;

                                    //_WalkingCtrl.Init_walking_pose(positionDesiredElmo, velocityDesiredElmo, slave-1);

                                    txPDO[slave - 1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[slave - 1]->targetPosition = (positionDesiredElmo(slave - 1)) * RAD2CNT[slave - 1] * Dr[slave - 1];
                                    //txPDO[slave - 1]->targetTorque = (int)(torqueDesiredElmo(slave - 1) * NM2CNT[slave - 1] * Dr[slave - 1]);

                                    std::cout << ec_slave[0].state << std::endl;
                                    if (dc.emergencyoff)
                                    {
                                        txPDO[slave - 1]->targetTorque = 0.0;
                                        if (Walking_State == 1)
                                        {
                                            //std::cout << "!!!!!! TORQUE TERMINATE !!!!!!" << std::endl;
                                        }
                                    }
                                    else
                                    {
                                        if (dc.torqueOn)
                                        {
                                            //If torqueOn command received, torque will increases slowly, for rising_time, which is currently 3 seconds.

                                            to_ratio = DyrosMath::minmax_cut((control_time_ - dc.torqueOnTime) / rising_time, 0.0, 1.0);
                                            dc.t_gain = to_ratio;
                                            //std::cout << "to_r : " << to_ratio << " control time : " << control_time_ << std::endl;
                                            if (dc.positionControl)
                                            {
                                                torqueDesiredElmo(slave - 1) = (Kp[slave - 1] * (positionDesiredElmo(slave - 1) - positionElmo(slave - 1))) + (Kv[slave - 1] * (0 - velocityElmo(slave - 1)));
                                                txPDO[slave - 1]->targetTorque = (int)(to_ratio * torqueDesiredElmo(slave - 1) * Dr[slave - 1]);
                                            }
                                            else
                                            {
                                                if (dc.customGain)
                                                {
                                                    txPDO[slave - 1]->targetTorque = (int)(to_ratio * torqueDesiredElmo(slave - 1) / CustomGain[slave - 1] * Dr[slave - 1]);
                                                }
                                                else
                                                {
                                                    txPDO[slave - 1]->targetTorque = (int)(to_ratio * torqueDesiredElmo(slave - 1) / NM2CNT[slave - 1] * Dr[slave - 1]);
                                                }
                                            }
                                        }
                                        else if (dc.torqueOff)
                                        {
                                            //If torqueOff command received, torque will decreases slowly, for rising_time(3 seconds. )

                                            if (dc.torqueOnTime + rising_time > dc.torqueOffTime)
                                            {
                                                to_calib = (dc.torqueOffTime - dc.torqueOnTime) / rising_time;
                                            }
                                            else
                                            {
                                                to_calib = 0.0;
                                            }
                                            to_ratio = DyrosMath::minmax_cut(1.0 - to_calib - (control_time_ - dc.torqueOffTime) / rising_time, 0.0, 1.0);

                                            dc.t_gain = to_ratio;
                                            //std::cout << "to_r : " << to_ratio << " control time : " << control_time_ << std::endl;
                                            if (dc.positionControl)
                                            {
                                                torqueDesiredElmo(slave - 1) = (Kp[slave - 1] * (positionDesiredElmo(slave - 1) - positionElmo(slave - 1))) + (Kv[slave - 1] * (0 - velocityElmo(slave - 1)));
                                                txPDO[slave - 1]->targetTorque = (int)(to_ratio * torqueDesiredElmo(slave - 1) * Dr[slave - 1]);
                                            }
                                            else
                                            {
                                                txPDO[slave - 1]->targetTorque = (int)(to_ratio * torqueDesiredElmo(slave - 1) / NM2CNT[slave - 1] * Dr[slave - 1]);
                                            }
                                        }
                                    }

                                    //txPDO[slave - 1]->targetTorque = (int)20;
                                    txPDO[slave - 1]->maxTorque = (uint16)500; // originaly 1000
                                }
                            }
                            mtx_q.unlock();
                        }
                        else if (Walking_State == 2)
                        {
                            //std::cout<<""
                        }
                        dc.elmo_cnt++;
                        needlf = TRUE;
                    }
                    clock_gettime(CLOCK_MONOTONIC, &time);
                    now = time.tv_sec;

                    control_time_ = (time.tv_sec - begin.tv_sec) + (time.tv_nsec - begin.tv_nsec) / 1000000000.0;

                    now += time.tv_nsec / 1000000000.0;
                    elapsed_time[elapsed_cnt] = now - prev;
                    prev = now;

                    elapsed_sum += elapsed_time[elapsed_cnt];
                    if (elapsed_min > elapsed_time[elapsed_cnt])
                        elapsed_min = elapsed_time[elapsed_cnt];
                    if (elapsed_max < elapsed_time[elapsed_cnt])
                        elapsed_max = elapsed_time[elapsed_cnt];

                    time_mem[elapsed_cnt] = (elapsed_time[elapsed_cnt] - (cycletime / 1000000000.0)) * 1000;

                    if (++elapsed_cnt >= 10000)
                    {
                        elapsed_avg = elapsed_sum / elapsed_cnt;
                        for (int i = 0; i < elapsed_cnt; i++)
                        {
                            elapsed_var += (elapsed_time[i] - elapsed_avg) * (elapsed_time[i] - elapsed_avg);
                            if (elapsed_time[i] > elapsed_avg + 0.00010)
                                max_cnt++;
                            if (elapsed_time[i] < elapsed_avg - 0.00010)
                                min_cnt++;
                        }

                        elapsed_var = elapsed_var / elapsed_cnt;
                        //printf("avg = %.3lf\tmin = %.3lf\tmax = %.3lf\tvar = %.6lf\tmax_cnt=%d\tmin_cnt=%d\tcnt = %d\n", elapsed_avg * 1000, elapsed_min * 1000, elapsed_max * 1000, elapsed_var * 1000000, max_cnt, min_cnt, elapsed_cnt);
                        max_cnt = 0;
                        min_cnt = 0;
                        elapsed_sum = 0;
                        elapsed_var = 0;
                        elapsed_cnt = 0;
                        elapsed_min = 210000000.0;
                        elapsed_max = 0.0;
                    }

                    add_timespec(&ts, cycletime);
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    if (ec_slave[slave - 1].state != EC_STATE_OPERATIONAL)
                    {
                        printf("EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               slave - 1, ec_slave[slave - 1].state, ec_slave[slave - 1].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave - 1].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            /** request INIT state for all slaves
             *  slave number = 0 -> write to all slaves
             */
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

void RealRobotInterface::updateState()
{
    //State is updated by main state loop of realrobot interface !
    ros::spinOnce();
    if (mtx_q.try_lock())
    {
        q_ = positionElmo;
        q_dot_ = velocityElmo;
        mtx_q.unlock();
        q_virtual_.segment(6, MODEL_DOF) = q_;
        q_dot_virtual_.segment(6, MODEL_DOF) = q_dot_;
    }
}

Eigen::VectorQd RealRobotInterface::getCommand()
{
    mtx_torque_command.lock();
    Eigen::VectorQd ttemp = torqueDesiredController;
    mtx_torque_command.unlock();
    return ttemp;
}

void RealRobotInterface::sendCommand(Eigen::VectorQd command, double sim_time)
{
    if (mtx_torque_command.try_lock())
    {
        torqueDesiredController = command;
        torque_desired = command;
        mtx_torque_command.unlock();
    }
}

void RealRobotInterface::ethercatCheck_()
{
    int expectedWKC;
    boolean needlf;
    volatile int wkc;
    boolean inOP;
    uint8 currentgroup = 0;
    printf("S\n");
    while (ros::ok())
    {
        std::cout << "test cout for ehtercatcheck ! " << std::endl;
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            printf("S\n");
            if (needlf)
            {
                printf("S\n");
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                printf("N %d\n", slave);
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("*");
        }

        std::cout << "test cout for ehtercatcheck ! : check end ! " << std::endl;

        for (int slave = 1; slave <= ec_slavecount; slave++)
            std::cout << "slave : " << slave << "\t" << ec_slave[slave].state << "\t";

        std::cout << std::endl;

        osal_usleep(250);
    }
    std::cout << "checking thread end !" << std::endl;
}

void RealRobotInterface::ethercatCheck()
{
}
void RealRobotInterface::ethercatThread()
{
}
void RealRobotInterface::imuThread()
{
}
void RealRobotInterface::ftsensorThread()
{
}



void RealRobotInterface::ImuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    q_virtual_(3) = msg->orientation.x;
    q_virtual_(4) = msg->orientation.y;
    q_virtual_(5) = msg->orientation.z;

    q_virtual_(MODEL_DOF_VIRTUAL) = msg->orientation.w;

    q_dot_virtual_(3) = msg->angular_velocity.x;
    q_dot_virtual_(4) = msg->angular_velocity.y;
    q_dot_virtual_(5) = msg->angular_velocity.z;

    //62.8hz lowpass velocity
}

bool RealRobotInterface::controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (statusWord & (1 << FAULT_BIT))
                {
                    controlWord = 0x80;
                    return false;
                }
                else
                {
                    controlWord = CW_SHUTDOWN;
                    return false;
                }
            }
            else
            {
                controlWord = CW_SWITCHON;
                return true;
            }
        }
        else
        {
            controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        controlWord = CW_ENABLEOP;
        return true;
    }
    controlWord = 0;
    return false;
}

void RealRobotInterface::add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

void RealRobotInterface::gainCallbak(const tocabi_controller::GainCommandConstPtr &msg)
{

    std::cout << "customgain Command received ! " << std::endl;
    for (int i = 0; i < MODEL_DOF; i++)
    {
        CustomGain[i] = msg->gain[i];
        std::cout << CustomGain[i] << "\t";
    }
    std::cout << std::endl;
    dc.customGain = true;
}