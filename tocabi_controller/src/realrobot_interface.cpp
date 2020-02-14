#include "tocabi_controller/realrobot_interface.h"
#include "sensoray826/sensoray826.h"
#include <errno.h>
#include <bitset>
#include <ros/package.h>

const char *homedir;

std::mutex mtx_elmo_command;
std::mutex mtx_q;

double rising_time = 3.0;
bool elmo_init = true;

RealRobotInterface::RealRobotInterface(DataContainer &dc_global) : dc(dc_global), StateManager(dc_global)
{
    imuSubscriber = dc.nh.subscribe("/imu/data", 1, &RealRobotInterface::ImuCallback, this);
    gainSubscriber = dc.nh.subscribe("/tocabi/gain_command", 100, &RealRobotInterface::gainCallbak, this);

    //pack_path = ros::package::getPath("tocabi_controller");
    zp_path = dc.homedir + "/zeropoint";
    zplog_path = dc.homedir + "/elmostart_log";

    torque_desired.setZero();

    positionElmo.setZero();
    positionExternalElmo.setZero();
    velocityElmo.setZero();
    torqueElmo.setZero();
    torqueDemandElmo.setZero();
    velocityDesiredElmo.setZero();
    torqueDesiredElmo.setZero();
    torqueDesiredController.setZero();
    positionDesiredElmo.setZero();
    positionDesiredElmo_Before.setZero();
    positionDesiredController.setZero();
    positionInitialElmo.setZero();
    positionZeroElmo.setZero();
    positionZeroModElmo.setZero();
    initTimeElmo.setZero();
    positionSafteyHoldElmo.setZero();

    rq_.setZero();
    rq_dot_.setZero();

    positionZeroModElmo(8) = 15.46875 * DEG2RAD;
    positionZeroModElmo(7) = 16.875 * DEG2RAD;
    positionZeroModElmo(TOCABI::Waist1_Joint) = -15.0 * DEG2RAD;
    positionZeroModElmo(TOCABI::Upperbody_Joint) = 0.0541;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc.currentGain(i) = NM2CNT[i];
    }

    file_homming.open(dc.homedir + "/hommingcheck.txt", ios_base::out);
    file_homming << "R7\tR8\tL8\tL7\tL3\tL4\tR4\tR3\tR5\tR6\tL6\tL5\tL1\tL2\tR2\tR1\tw1\tw2" << endl;

    fz_group1.resize(16);
    int i = 0;
    fz_group1[i++] = TOCABI::Neck_Joint;
    fz_group1[i++] = TOCABI::Head_Joint;

    fz_group1[i++] = TOCABI::R_Shoulder1_Joint;
    fz_group1[i++] = TOCABI::R_Shoulder2_Joint;
    fz_group1[i++] = TOCABI::R_Shoulder3_Joint;
    fz_group1[i++] = TOCABI::R_Armlink_Joint;
    fz_group1[i++] = TOCABI::R_Elbow_Joint;
    fz_group1[i++] = TOCABI::R_Forearm_Joint;
    fz_group1[i++] = TOCABI::R_Wrist1_Joint;

    fz_group1[i++] = TOCABI::L_Shoulder1_Joint;
    fz_group1[i++] = TOCABI::L_Shoulder2_Joint;
    fz_group1[i++] = TOCABI::L_Shoulder3_Joint;
    fz_group1[i++] = TOCABI::L_Armlink_Joint;
    fz_group1[i++] = TOCABI::L_Elbow_Joint;
    fz_group1[i++] = TOCABI::L_Forearm_Joint;
    fz_group1[i++] = TOCABI::L_Wrist1_Joint;

    elmofz[TOCABI::R_Armlink_Joint].init_direction = -1.0;
    elmofz[TOCABI::L_Armlink_Joint].init_direction = -1.0;
    elmofz[TOCABI::R_Elbow_Joint].init_direction = -1.0;
    elmofz[TOCABI::Upperbody_Joint].init_direction = -1.0;

    elmofz[TOCABI::R_Elbow_Joint].req_length = 0.06;
    elmofz[TOCABI::L_Elbow_Joint].req_length = 0.09;
    elmofz[TOCABI::L_Forearm_Joint].req_length = 0.09;
    elmofz[TOCABI::R_Forearm_Joint].req_length = 0.14;

    elmofz[TOCABI::L_Shoulder1_Joint].req_length = 0.18;
    elmofz[TOCABI::L_Shoulder2_Joint].req_length = 0.17;
    elmofz[TOCABI::R_Shoulder2_Joint].req_length = 0.08;

    elmofz[TOCABI::R_Shoulder3_Joint].req_length = 0.03;
    elmofz[TOCABI::L_Shoulder3_Joint].req_length = 0.07;

    elmofz[TOCABI::Waist2_Joint].req_length = 0.07;
    elmofz[TOCABI::Waist1_Joint].req_length = 0.07;

    fz_group2.resize(3);
    i = 0;
    fz_group2[i++] = TOCABI::Upperbody_Joint;
    fz_group2[i++] = TOCABI::Waist2_Joint;
    fz_group2[i++] = TOCABI::Waist1_Joint;
}

void RealRobotInterface::findZeroLeg()
{
    //std::cout << "lower leg check " << std::endl;
    for (int i = 0; i < 5; i++)
    {
        positionZeroElmo[i + TOCABI::R_HipRoll_Joint] = positionElmo[i + TOCABI::R_HipRoll_Joint] - positionExternalElmo[i + TOCABI::R_HipRoll_Joint];
        pub_to_gui(dc, "jointzp %d %d", i + TOCABI::R_HipRoll_Joint, 1);
        positionZeroElmo[i + TOCABI::L_HipRoll_Joint] = positionElmo[i + TOCABI::L_HipRoll_Joint] - positionExternalElmo[i + TOCABI::L_HipRoll_Joint];
        pub_to_gui(dc, "jointzp %d %d", i + TOCABI::L_HipRoll_Joint, 1);
        //std::cout << TOCABI::ELMO_NAME[i + TOCABI::R_HipRoll_Joint] << " pz IE P : " << positionElmo[i + TOCABI::R_HipRoll_Joint] << " pz EE P : " << positionExternalElmo[i + TOCABI::R_HipRoll_Joint] << std::endl;
        //std::cout << TOCABI::ELMO_NAME[i + TOCABI::L_HipRoll_Joint] << " pz ELMO : " << positionElmo[i + TOCABI::L_HipRoll_Joint] << " pz ELMO : " << positionExternalElmo[i + TOCABI::L_HipRoll_Joint] << std::endl;
    }
}

void RealRobotInterface::updateState()
{
    //State is updated by main state loop of realrobot interface !
    ros::spinOnce();
    if (mtx_q.try_lock())
    {
        q_ = rq_;
        q_dot_ = rq_dot_;
        mtx_q.unlock();
        q_virtual_.segment(6, MODEL_DOF) = q_;
        q_dot_virtual_.segment(6, MODEL_DOF) = q_dot_;
        mtx_q.unlock();
    }
}

Eigen::VectorQd RealRobotInterface::getCommand()
{
    Eigen::VectorQd t1_, t2_;

    mtx_elmo_command.lock();
    t1_ = torqueDesiredController;
    mtx_elmo_command.unlock();

    if (dc.positionControl)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            for (int j = 0; j < MODEL_DOF; j++)
            {
                if (TOCABI::ELMO_NAME[i] == TOCABI::JOINT_NAME[j])
                {
                    //ttemp(i) = positionDesiredController(j);
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            for (int j = 0; j < MODEL_DOF; j++)
            {
                if (TOCABI::ELMO_NAME[i] == TOCABI::JOINT_NAME[j])
                {
                    t2_(i) = t1_(j);
                }
            }
        }
    }
    return t2_;
}
int RealRobotInterface::checkTrajContinuity(int slv_number)
{
}

void RealRobotInterface::checkSafety(int slv_number, double max_vel, double max_dis)
{
    bool damping_mode = false;

    if (ElmoSafteyMode[slv_number] == 0)
    {
        if (checkPosSafety[slv_number])
        {
            if (abs(positionDesiredElmo(slv_number) - positionDesiredElmo_Before(slv_number)) > max_dis * 10.0)
            {
                std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " trajectory discontinuity : " << (positionDesiredElmo(slv_number) - positionDesiredElmo_Before(slv_number)) << creset << std::endl;
                pub_to_gui(dc, "%d %s locked, traj err", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
                ElmoSafteyMode[slv_number] = 1;
                positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
            }
        }
        if (ElmoMode[slv_number] == EM_POSITION)
        {
            if (abs(positionDesiredElmo(slv_number) - positionElmo(slv_number)) > max_dis * 10.0)
            {
                std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Position Command discontinuity : " << (positionDesiredElmo(slv_number) - positionElmo(slv_number)) << creset << std::endl;
                pub_to_gui(dc, "%d %s locked, command err", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
                ElmoSafteyMode[slv_number] = 1;
                positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
            }
        }

        if (abs(velocityElmo(slv_number)) > max_vel)
        {
            std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Velocity Over Limit" << creset << std::endl;
            ElmoSafteyMode[slv_number] = 1;
            positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
        }
    }

    if (ElmoSafteyMode[slv_number] == 1)
    {

        txPDO[slv_number]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
        txPDO[slv_number]->targetPosition = (int)(Dr[slv_number] * RAD2CNT[slv_number] * positionSafteyHoldElmo[slv_number]);
        if (abs(positionElmo[slv_number] - positionSafteyHoldElmo[slv_number]) > 0.15)
        {
            ElmoSafteyMode[slv_number] = 2;
            std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Holding Malfunction! Maybe current off? Torque zero!" << creset << std::endl;
        }
    }

    if (ElmoSafteyMode[slv_number] == 2)
    {
        txPDO[slv_number]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        txPDO[slv_number]->targetTorque = 0;
    }
}

void RealRobotInterface::findZeroPoint(int slv_number)
{
    double fztime = 3.0;
    double fztime_manual = 300.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        pub_to_gui(dc, "jointzp %d %d", slv_number, 0);
        if (hommingElmo[slv_number])
        {
            //std::cout << "motor " << slv_number << " init state : homming on" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
            elmofz[slv_number].firstPos = positionElmo[slv_number];
        }
        else
        {
            //std::cout << "motor " << slv_number << " init state : homming off" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMING;
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
            elmofz[slv_number].firstPos = positionElmo[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {
        //go to + 0.3rad until homming sensor turn off
        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos, 0.3, elmofz[slv_number].initTime, fztime);

        if ((hommingElmo[slv_number] == 0) && (hommingElmo_before[slv_number] == 0))
        {
            //std::cout << "motor " << slv_number << " seq 1 complete, wait 1 sec" << std::endl;
            hommingElmo_before[slv_number] = hommingElmo[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].posStart = positionElmo[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGEND)
    {
        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].posStart, -0.3, elmofz[slv_number].initTime, fztime);

        //go to -20deg until homming turn on, and turn off
        if ((hommingElmo_before[slv_number] == 1) && (hommingElmo[slv_number] == 0))
        {
            if (abs(elmofz[slv_number].posStart - positionElmo[slv_number]) > elmofz[slv_number].req_length)
            {
                elmofz[slv_number].posEnd = positionElmo[slv_number];
                elmofz[slv_number].endFound = 1;
            }
            else
            {
                printf("Motor %d : Not enough length start point : %f, Current Point %f \n", slv_number, elmofz[slv_number].posStart, positionElmo[slv_number]);

                //std::cout << "off : homming turned off, but not enough length start point : " << elmofz[slv_number].posStart << " Current off point : " << positionElmo[slv_number] << std::endl;
            }
        }
        else if ((hommingElmo_before[slv_number] == 0) && (hommingElmo[slv_number] == 0))
        {
            if (elmofz[slv_number].endFound == 1)
            {
                //std::cout << "motor " << slv_number << " seq 2 complete" << std::endl;
                elmofz[slv_number].findZeroSequence = 4;
                positionZeroElmo[slv_number] = (elmofz[slv_number].posEnd + elmofz[slv_number].posStart) * 0.5 + positionZeroModElmo[slv_number];
                elmofz[slv_number].initTime = control_time_;
                //std::cout << "on : Motor " << slv_number << " zero point found : " << positionZeroElmo[slv_number] << std::endl;
            }
        }

        if (control_time_ > elmofz[slv_number].initTime + fztime)
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMING)
    { //start from unknown

        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.3, elmofz[slv_number].initTime, fztime);
        if (control_time_ > (elmofz[slv_number].initTime + fztime))
        {
            positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos + 0.3 * elmofz[slv_number].init_direction, -0.6 * elmofz[slv_number].init_direction, elmofz[slv_number].initTime + 2.0, fztime * 2.0);
        }

        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            //std::cout << "homming found ! to sequence 1 ! " << std::endl;
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }

        if (control_time_ > (elmofz[slv_number].initTime + fztime * 3.0))
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_GOTOZEROPOINT)
    {
        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].posEnd, positionZeroElmo(slv_number) - elmofz[slv_number].posEnd, elmofz[slv_number].initTime, fztime * (abs(positionZeroElmo(slv_number) - elmofz[slv_number].posEnd) / 0.3));
        //go to zero position
        if (control_time_ > (elmofz[slv_number].initTime + fztime / 2))
        {
            //std::cout << "go to zero complete !" << std::endl;
            printf("Motor %d %s : Zero Point Found : %8.6f, homming length : %8.6f ! \n", slv_number, TOCABI::ELMO_NAME[slv_number].c_str(), positionZeroElmo[slv_number], abs(elmofz[slv_number].posStart - elmofz[slv_number].posEnd));
            pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
            //std::cout << slv_number << "Start : " << elmofz[slv_number].posStart << "End:" << elmofz[slv_number].posEnd << std::endl;
            positionDesiredElmo[slv_number] = positionZeroElmo(slv_number);
            elmofz[slv_number].findZeroSequence = 8; // torque to zero -> 8 position hold -> 5
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 5)
    {
        //find zero complete, hold zero position.
        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = positionZeroElmo(slv_number);
    }
    else if (elmofz[slv_number].findZeroSequence == 6)
    {
        //find zero point failed
        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].firstPos - elmofz[slv_number].initPos, elmofz[slv_number].initTime, fztime);
        if (control_time_ > (elmofz[slv_number].initTime + fztime))
        {
            elmofz[slv_number].findZeroSequence = 7;
            printf("Motor %d %s : Zero point detection Failed. Manual Detection Required. \n", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
            pub_to_gui(dc, "jointzp %d %d", slv_number, 2);
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            elmofz[slv_number].initTime = control_time_;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 7)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torqueDemandElmo[slv_number] = 0.0;
        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }
        if (control_time_ > (elmofz[slv_number].initTime + fztime_manual))
        {
            printf("Motor %d %s :  Manual Detection Failed. \n", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
            pub_to_gui(dc, "jointzp %d %d", slv_number, 3);
            elmofz[slv_number].findZeroSequence = 8;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 8)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torqueDemandElmo[slv_number] = 0.0;
    }
}

void RealRobotInterface::sendCommand(Eigen::VectorQd command, double sim_time)
{
    if (mtx_elmo_command.try_lock())
    {
        torqueDesiredController = command;
        torque_desired = command;
        mtx_elmo_command.unlock();
    }
}

void RealRobotInterface::ethercatCheck()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ethercatCheck Thread Start" << std::endl;
    while (ros::ok() && (!shutdown_tocabi_bool))
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                printf("S\n");
                needlf = FALSE;
                printf("\n");
            }
            // one ore more slaves are not responding
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("%sERROR : slave %d is in SAFE_OP + ERROR, attempting ack.%s\n", cred.c_str(), slave - 1, creset.c_str());
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("%sWARNING : slave %d is in SAFE_OP, change to OPERATIONAL.%s\n", cred.c_str(), slave - 1, creset.c_str());
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("%sMESSAGE : slave %d reconfigured%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        // re-check state
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            ConnectionUnstableBeforeStart = true;
                            printf("%sERROR : slave %d lost%s\n", cred.c_str(), slave - 1, creset.c_str());
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
                            printf("%sMESSAGE : slave %d recovered%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("%sMESSAGE : slave %d found%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                    }
                }
            }
            //if (!ec_group[currentgroup].docheckstate)
            //    printf("*");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << cyellow << "checking thread end !" << creset << std::endl;
}

void RealRobotInterface::ethercatThread()
{
    cpu_set_t cpuset;
    pthread_t thread;
    thread = pthread_self();
    CPU_ZERO(&cpuset);
    CPU_SET(7, &cpuset);
    //sched_setaffinity(getpid(),sizeof(cpuset),&cpuset);
    if (pthread_setaffinity_np(thread, sizeof(cpuset), &cpuset))
    {
        std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ethercatThread Start" << std::endl;

    char IOmap[4096];
    bool reachedInitial[MODEL_DOF] = {false};

    bool exit_middle = false;

    const char *ifname = dc.ifname.c_str();

    if (ec_init(ifname))
    {
        printf("ELMO : ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        /* network discovery */
        //ec_config_init()
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("ELMO : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num

            /** CompleteAccess disabled for Elmo driver */
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //printf("ELMO : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    printf("ELMO : slave[%d] CA? : false , shutdown request \n ", slave);
                    exit_middle = true;
                    shutdown_tocabi_bool = true;
                }
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            if (!exit_middle)
            {
                if (MODEL_DOF == ec_slavecount)
                {

                    for (int slave = 1; slave <= ec_slavecount; slave++)
                    {
                        //0x1605 :  Target Position             32bit
                        //          Target Velocity             32bit
                        //          Max Torque                  16bit
                        //          Control word                16bit
                        //          Modes of Operation          16bit
                        uint16 map_1c12[2] = {0x0001, 0x1605};

                        //0x1a00 :  position actual value       32bit
                        //          Digital Inputs              32bit
                        //          Status word                 16bit
                        //0x1a11 :  velocity actual value       32bit
                        //0x1a13 :  Torque actual value         16bit
                        //0x1a1e :  Auxiliart position value    32bit
                        uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                        //uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};
                        int os;
                        os = sizeof(map_1c12);
                        ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, map_1c12, EC_TIMEOUTRXM);
                        os = sizeof(map_1c13);
                        ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, map_1c13, EC_TIMEOUTRXM);
                    }

                    /** if CA disable => automapping works */
                    ec_config_map(&IOmap);

                    /* wait for all slaves to reach SAFE_OP state */
                    printf("ELMO : EC WAITING STATE TO SAFE_OP\n");
                    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

                    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                    printf("ELMO : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);

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
                        //dc.connected = true;
                        //printf("Operational state reached for all slaves.\n");
                        pub_to_gui(dc, "All slaves Status GREEN");
                        pub_to_gui(dc, "STARTING IN 3 ... ");
                        printf("ELMO : Operational state reached for all slaves! Starting in ... 3... ");
                        fflush(stdout);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        printf("2... ");
                        pub_to_gui(dc, "2 ... ");
                        fflush(stdout);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        printf("1... ");
                        pub_to_gui(dc, "1 ... ");
                        fflush(stdout);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        printf("0... Start! \n");
                        pub_to_gui(dc, "GO! ");

                        inOP = TRUE;

                        if (ConnectionUnstableBeforeStart)
                        {
                            shutdown_tocabi_bool = true;
                        }

                        /* cyclic loop */
                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                            rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                        }

                        //std::chrono::steady_clock::time_point t_begin = std::chrono::steady_clock::now();

                        std::chrono::duration<double> time_from_begin;
                        std::chrono::duration<double> time_err_reset = std::chrono::seconds(0);

                        std::chrono::microseconds cycletime(dc.ctime);
                        int cycle_count = 0;

                        std::chrono::steady_clock::time_point tp[6];
                        std::chrono::steady_clock::time_point time_until;

                        std::chrono::duration<double> td[7];
                        double to_ratio, to_calib;

                        double pwait_time = 1.0;
                        int c_count = 0;

                        double d_min = 1000;
                        double d_max = 0;
                        double d_mean = 0;

                        double d1_min = 1000;
                        double d1_max = 0;
                        double d1_mean = 0;

                        int oc_cnt = 0;
                        int oct_cnt = 0;

                        for (int i = 0; i < MODEL_DOF; i++)
                        {
                            ElmoSafteyMode[i] = 0;
                        }

                        dc.connected = true;
                        st_start_time = std::chrono::steady_clock::now();
                        dc.start_time_point = st_start_time;
                        while (!shutdown_tocabi_bool)
                        {
                            //std::cout<<"firstrealrobot"<<std::endl;
                            //Ethercat Loop begins :: RT Thread
                            tp[0] = std::chrono::steady_clock::now();

                            //std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
                            /*
                        while (std::chrono::steady_clock::now() < (t_begin + cycle_count * cycletime))
                        {
                            std::this_thread::sleep_for(std::chrono::nanoseconds(500));
                        }*/

                            std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);

                            tp[1] = std::chrono::steady_clock::now();
                            time_from_begin = std::chrono::steady_clock::now() - st_start_time;
                            control_time_ = time_from_begin.count();

                            /** PDO I/O refresh */
                            //ec_send_processdata();

                            tp[2] = std::chrono::steady_clock::now();
                            wkc = ec_receive_processdata(200);

                            tp[3] = std::chrono::steady_clock::now();

                            td[0] = st_start_time + cycle_count * cycletime - tp[0];
                            td[1] = tp[1] - (st_start_time + cycle_count * cycletime);

                            td[2] = tp[2] - tp[1];
                            td[3] = tp[3] - tp[2];

                            if (td[3] > std::chrono::microseconds(190))
                            {
                                oc_cnt++;
                            }

                            if (tp[3] > st_start_time + (cycle_count + 1) * cycletime)
                            {
                                std::cout << cred << " t_wait : " << td[0].count() * 1E+6 << " us, t_start : " << td[1].count() * 1E+6 << " us, ec_send : " << td[2].count() * 1E+6 << " us, ec_receive : " << td[3].count() * 1E+6 << " us" << creset << std::endl;
                            }

                            if (wkc >= expectedWKC)
                            {

                                for (int slave = 1; slave <= ec_slavecount; slave++)
                                {
                                    if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                                    {
                                        reachedInitial[slave - 1] = true;
                                    }
                                }

                                for (int slave = 1; slave <= ec_slavecount; slave++)
                                {
                                    if (reachedInitial[slave - 1])
                                    {
                                        //Get status
                                        positionElmo(slave - 1) = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * Dr[slave - 1];

                                        hommingElmo[slave - 1] =
                                            (((uint32_t)ec_slave[slave].inputs[4]) +
                                             ((uint32_t)ec_slave[slave].inputs[5] << 8) +
                                             ((uint32_t)ec_slave[slave].inputs[6] << 16) +
                                             ((uint32_t)ec_slave[slave].inputs[7] << 24));

                                        stateElmo[slave - 1] =
                                            (((uint16_t)ec_slave[slave].inputs[8]) +
                                             ((uint16_t)ec_slave[slave].inputs[9] << 8));

                                        velocityElmo(slave - 1) =
                                            (((int32_t)ec_slave[slave].inputs[10]) +
                                             ((int32_t)ec_slave[slave].inputs[11] << 8) +
                                             ((int32_t)ec_slave[slave].inputs[12] << 16) +
                                             ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                                            CNT2RAD[slave - 1] * Dr[slave - 1];

                                        torqueElmo(slave - 1) =
                                            (((int16_t)ec_slave[slave].inputs[14]) +
                                             ((int16_t)ec_slave[slave].inputs[15] << 8));

                                        positionExternalElmo(slave - 1) =
                                            (((int32_t)ec_slave[slave].inputs[16]) +
                                             ((int32_t)ec_slave[slave].inputs[17] << 8) +
                                             ((int32_t)ec_slave[slave].inputs[18] << 16) +
                                             ((int32_t)ec_slave[slave].inputs[19] << 24) - positionExternalModElmo[slave - 1]) *
                                            CNT2RAD[slave - 1] * -100.0 * Dr[slave - 1];

                                        ElmoConnected = true;

                                        if (slave == 1 || slave == 2 || slave == 19 || slave == 20)
                                        {
                                            hommingElmo[slave - 1] = !hommingElmo[slave - 1];
                                        }
                                        txPDO[slave - 1]->maxTorque = (uint16)1000; // originaly 1000
                                        ElmoMode[slave - 1] = EM_TORQUE;
                                        torqueDemandElmo[slave - 1] = 0.0;
                                    }
                                }
                            }

                            mtx_q.lock();
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                for (int j = 0; j < ec_slavecount; j++)
                                {
                                    if (TOCABI::JOINT_NAME[i] == TOCABI::ELMO_NAME[j].c_str())
                                    {
                                        {
                                            rq_[i] = positionElmo[j] - positionZeroElmo[j];
                                            rq_dot_[i] = velocityElmo[j];
                                        }
                                    }
                                }
                            }
                            mtx_q.unlock();

                            //Get State Seqence End, user controller start

                            dc.torqueElmo = torqueElmo;
                            checkfirst = (stateElmo[0] & al);

                            if (shutdown_tocabi_bool || !ros::ok() || shutdown_tocabi_bool)
                            {
                                ElmoTerminate = true;
                                //std::terminate();
                                break;
                            }

                            if (elmo_init)
                            {
                                if (checkfirst_before != checkfirst)
                                {
                                    //std::cout << "firstRun ON " << std::endl;

                                    if ((bootseq == 0) && (checkfirst == 0))
                                        bootseq++;
                                    if ((bootseq == 1) && (checkfirst == 33))
                                        bootseq++;
                                    if ((bootseq == 2) && (checkfirst == 35))
                                        bootseq++;
                                    if (bootseq == 3)
                                    {
                                        if (checkfirst == 39)
                                        {
                                            bootseq++;
                                        }
                                        else if (checkfirst == 8)
                                        {
                                            bootseq = 5;
                                        }
                                    }
                                    if ((bootseq == 4) && (checkfirst == 39))
                                    {
                                        std::cout << cgreen << "ELMO : READY, WARMSTART ! LOADING ZERO POINT ... " << creset << std::endl;
                                        pub_to_gui(dc, "ELMO WARM START, LOADING ZP ");

                                        bool waitop = true;
                                        for (int i = 0; i < MODEL_DOF; i++)
                                        {
                                            waitop = waitop && ((stateElmo[i] & al) == 39);
                                        }
                                        if (waitop)
                                        {
                                            pub_to_gui(dc, "ecatgood");
                                            bootseq++;
                                        }

                                        string tmp;
                                        double last_boot_time = 1;
                                        double last_save_time = 0;
                                        double zp[MODEL_DOF];
                                        bool loadfailed = false;

                                        elmo_zp_log.open(zplog_path, ios_base::in);

                                        if (elmo_zp_log.is_open())
                                        {
                                            getline(elmo_zp_log, tmp);
                                            last_boot_time = atof(tmp.c_str());
                                        }
                                        elmo_zp.open(zp_path, ios_base::in);

                                        if (elmo_zp.is_open())
                                        {
                                            getline(elmo_zp, tmp);
                                            last_save_time = atof(tmp.c_str());
                                        }

                                        if (last_save_time < last_boot_time)
                                        {
                                            cout << "ELMO : ERROR WITH LOADING ZERO POINT! " << std::endl;
                                            pub_to_gui(dc, "ZP LOADING FAILURE");
                                        }
                                        else
                                        {
                                            if (elmo_zp.is_open())
                                            {
                                                for (int i = 0; i < MODEL_DOF; i++)
                                                {
                                                    if (elmo_zp.eof())
                                                    {
                                                        std::cout << "ELMO : ERROR WITH LOADING ZERO POINT ! press i to check zeropoint !" << std::endl;
                                                        pub_to_gui(dc, "ZP LOADING FAILURE : prceed initialize");
                                                        loadfailed = true;
                                                        break;
                                                    }
                                                    getline(elmo_zp, tmp);
                                                    zp[i] = atof(tmp.c_str());
                                                }
                                                if (!loadfailed)
                                                {
                                                    std::cout << cgreen << "ELMO : ZERO POINT LOADED ! " << creset << std::endl;
                                                    pub_to_gui(dc, "ZP LOADING SUCCESS");
                                                    pub_to_gui(dc, "zpgood");
                                                    pub_to_gui(dc, "ecatgood");
                                                    elmo_init = false;
                                                    for (int i = 0; i < MODEL_DOF; i++)
                                                    {
                                                        positionZeroElmo[i] = zp[i];
                                                    }
                                                    positionInitialElmo = positionElmo;
                                                    dc.elmo_Ready = true;
                                                }
                                            }
                                        }
                                    }
                                    if ((bootseq == 5) && (checkfirst == 64))
                                    {
                                        std::cout << cyellow << "ELMO : COMMUTATION INITIALIZING ! " << creset << std::endl;
                                        pub_to_gui(dc, "ELMO : COMMUTATION INITIALIZING ! ");
                                        pub_to_gui(dc, "ecatcommutation");
                                        bootseq = 6;

                                        elmo_zp_log.open(zplog_path, ios_base::out);
                                        if (elmo_zp_log.is_open())
                                        {
                                            elmo_zp_log << setprecision(12) << ros::Time::now().toSec() << "\n";
                                            elmo_zp_log.close();
                                        }
                                        else
                                        {
                                            std::cout << cred << "ELMO : COMMUTATION LOGGING ERROR " << creset << std::endl;
                                            pub_to_gui(dc, "ELMO : COMMUTATION LOGGING ERROR ");
                                        }
                                    }
                                }
                                if ((bootseq == 6))
                                {
                                    bool waitop = true;
                                    for (int i = 0; i < MODEL_DOF; i++)
                                    {
                                        waitop = waitop && ((stateElmo[i] & al) == 39);
                                    }
                                    if (waitop)
                                    {
                                        std::cout << cgreen << "ELMO : COMMUTATION INITIALIZE COMPLETE ! Press i to check zeropoint ! " << creset << std::endl;
                                        pub_to_gui(dc, "ELMO : COMMUTATION INITIALIZE COMPLETE");
                                        pub_to_gui(dc, "ecatgood");

                                        bootseq++;
                                    }
                                }
                            }

                            checkfirst_before = checkfirst;
                            if (elmo_init && dc.start_initialize_sequence)
                            {
                                positionInitialElmo = positionElmo;
                                elmo_init = false;
                                std::cout << cred << "ELMO : Robot Initialize Process ! Finding Zero Point !" << creset << std::endl;
                                pub_to_gui(dc, "ELMO : SEARCHING ZP");
                                findZeroLeg();

                                elmofz[TOCABI::R_Shoulder3_Joint].findZeroSequence = 7;
                                elmofz[TOCABI::R_Shoulder3_Joint].initTime = control_time_;
                                pub_to_gui(dc, "jointzp %d %d", TOCABI::R_Shoulder3_Joint, 2);
                                elmofz[TOCABI::L_Shoulder3_Joint].findZeroSequence = 7;
                                elmofz[TOCABI::L_Shoulder3_Joint].initTime = control_time_;
                                pub_to_gui(dc, "jointzp %d %d", TOCABI::L_Shoulder3_Joint, 2);

                                for (int j = 0; j < MODEL_DOF; j++)
                                {
                                    hommingElmo_before[j] = hommingElmo[j];
                                }
                            }
                            fz_group1_check = true;

                            for (int i = 0; i < fz_group1.size(); i++)
                            {
                                fz_group1_check = fz_group1_check && (elmofz[fz_group1[i]].result == ElmoHommingStatus::SUCCESS);
                            }

                            fz_group2_check = true;
                            for (int i = 0; i < fz_group2.size(); i++)
                            {
                                fz_group2_check = fz_group2_check && (elmofz[fz_group2[i]].result == ElmoHommingStatus::SUCCESS);
                            }

                            if (fz_group1_check && (fz_group == 0))
                            {
                                fz_group++;
                                std::cout << cgreen << "ELMO : Upperbody zero point check complete " << creset << std::endl;
                            }
                            if (fz_group2_check && (fz_group == 1))
                            {
                                fz_group++;
                                elmo_zp.open(zp_path, ios_base::out);
                                elmo_zp << setprecision(12) << ros::Time::now().toSec() << "\n";
                                for (int i = 0; i < MODEL_DOF; i++)
                                {
                                    elmo_zp << positionZeroElmo[i] << "\n";
                                }
                                elmo_zp.close();

                                std::cout << cgreen << "ELMO : Waist zero point check complete and zero point saved. " << creset << std::endl;
                                pub_to_gui(dc, "ELMO : ZP SAVED");
                                pub_to_gui(dc, "zpgood");
                                dc.elmo_Ready = true;
                            }

                            //torqueDesiredController = getCommand();
                            if (dc.elmo_Ready)
                                torqueDesiredElmo = getCommand();

                            for (int i = 0; i < MODEL_DOF; i++)
                            {
                                if (!dc.elmo_Ready)
                                {
                                    if (!elmo_init)
                                    {
                                        if (fz_group == 0)
                                        {
                                            for (int j = 0; j < fz_group1.size(); j++)
                                            {
                                                if (i == fz_group1[j])
                                                {
                                                    findZeroPoint(i);
                                                }
                                            }
                                            //std::cout<<control_time_<<"\t"<<hommingElmo[TOCABI::R_Shoulder3_Joint]<<std::endl;
                                        }
                                        else if (fz_group == 1)
                                        {
                                            for (int j = 0; j < fz_group2.size(); j++)
                                            {
                                                if (i == fz_group2[j])
                                                {
                                                    findZeroPoint(i);
                                                }
                                            }
                                        }

                                        //if (slave == 1 || slave == 2 ||slave == 11 || slave == 14)
                                        //if (i == 1 || i == 0)
                                        //    findZeroPoint(i);

                                        hommingElmo_before[i] = hommingElmo[i];
                                    }
                                    else
                                    {
                                        torqueDesiredElmo[i] = 0.0;
                                    }
                                }
                                else
                                {
                                    if (dc.torqueOn)
                                    {
                                        //If torqueOn command received, torque will increases slowly, for rising_time, which is currently 3 seconds.
                                        to_ratio = DyrosMath::minmax_cut((control_time_ - dc.torqueOnTime) / rising_time, 0.0, 1.0);
                                        ElmoMode[i] = EM_TORQUE;
                                        dc.t_gain = to_ratio;
                                        if (dc.positionControl)
                                        {
                                            torqueDesiredElmo(i) = to_ratio * (Kp[i] * (positionDesiredElmo(i) - positionElmo(i))) + (Kv[i] * (0 - velocityElmo(i)));
                                        }
                                        else
                                        {
                                            torqueDesiredElmo(i) = to_ratio * torqueDesiredElmo(i);
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

                                        if (dc.positionControl)
                                        {
                                            torqueDesiredElmo(i) = to_ratio * (Kp[i] * (positionDesiredElmo(i) - positionElmo(i))) + (Kv[i] * (0 - velocityElmo(i)));
                                        }
                                        else
                                        {
                                            torqueDesiredElmo(i) = to_ratio * torqueDesiredElmo(i);
                                        }
                                    }
                                    else
                                    {
                                        ElmoMode[i] = EM_TORQUE;
                                        torqueDesiredElmo[i] = 0.0;
                                    }
                                }

                                if (dc.torquezeroByTerminal)
                                {
                                    ElmoMode[i] = EM_TORQUE;
                                    torqueDesiredElmo[i] = 0;
                                }

                                if (ElmoMode[i] == EM_POSITION)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                                    txPDO[i]->targetPosition = (int)(Dr[i] * RAD2CNT[i] * positionDesiredElmo[i]);
                                }
                                else if (ElmoMode[i] == EM_TORQUE)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;

                                    if (dc.customGain)
                                    {
                                        txPDO[i]->targetTorque = (int)(torqueDesiredElmo[i] * CustomGain[i] * Dr[i]);
                                    }
                                    else
                                    {
                                        txPDO[i]->targetTorque = (int)(torqueDesiredElmo[i] * NM2CNT[i] * Dr[i]);
                                    }
                                }
                                else
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }

                            //Hold position if safety limit breached
                            for (int i = 0; i < MODEL_DOF; i++)
                            {
                                if (ElmoMode[i] != EM_POSITION)
                                {
                                    checkPosSafety[i] = false;
                                }
                                checkSafety(i, 2.0, 10.0 * dc.ctime / 1E+6); //if angular velocity exceeds 0.5rad/s, Hold to current Position ///
                            }

                            //Torque off if emergency off received
                            if (dc.emergencyoff)
                            {
                                for (int i = 0; i < MODEL_DOF; i++)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }

                            //std::this_thread::sleep_until(st_start_time + cycle_count * cycletime+ std::chrono::microseconds(250));
                            ec_send_processdata();

                            positionDesiredElmo_Before = positionDesiredElmo;
                            if (dc.disableSafetyLock)
                            {
                                for (int i = 0; i < MODEL_DOF; i++)
                                {
                                    ElmoSafteyMode[i] = 0;
                                }
                                dc.disableSafetyLock = false;
                            }
                            for (int i = 0; i < MODEL_DOF; i++)
                            {
                                if (ElmoMode[i] == EM_POSITION)
                                {
                                    checkPosSafety[i] = true;
                                }
                            }
                            td[4] = std::chrono::steady_clock::now() - (st_start_time + cycle_count * cycletime); //timestamp for send time consumption.
                            d_mean = d_mean + td[4].count();
                            if (d_min > td[4].count())
                                d_min = td[4].count();
                            if (d_max < td[4].count())
                                d_max = td[4].count();

                            d1_mean = d1_mean + td[3].count();
                            if (d1_min > td[3].count())
                                d1_min = td[3].count();
                            if (d1_max < td[3].count())
                                d1_max = td[3].count();

                            c_count++;

                            if (control_time_ > pwait_time)
                            {
                                oct_cnt += oc_cnt;
                                if (dc.print_delay_info)
                                {
                                    printf("%3.0f, %d hz SEND min : %5.2f us, max : %5.2f us, avg : %5.2f us RECV min : %5.2f us, max : %5.2f us, avg %5.2f us, oc : %d, oct : %d \n", control_time_, c_count, d_min * 1.0E+6,
                                           d_max * 1.0E+6, d_mean / c_count * 1.0E+6, d1_min * 1.0E+6, d1_max * 1.0E+6, d1_mean * 1.0E+6 / c_count, oc_cnt, oct_cnt);

                                    pub_to_gui(dc, "%f : tout : %d, total : %d", control_time_, oc_cnt, oct_cnt);

                                    //int al = 63;
                                    std::bitset<16> stx(stateElmo[0]);
                                    std::bitset<16> stx2(stateElmo[0] & al);
                                    //std::cout << "current statusword : " << stx << std::endl;
                                    //std::cout << stx2 << std::endl;

                                    for (int i = 0; i < MODEL_DOF; i++)
                                    {
                                        if ((stateElmo[i] & al) != 39)
                                        {
                                            std::cout << "Joint " << i << "Not operational, " << (stateElmo[i] & al) << std::endl;
                                        }
                                    }
                                }
                                //std::cout << control_time_ << ", " << c_count << std::setprecision(4) << " hz, min : " << d_min * 1.0E+6 << " us , max : " << d_max * 1.0E+6 << " us, mean " << d_mean / c_count * 1.0E+6 << " us"
                                //          << "receive : mean :" << d1_mean / c_count * 1.0E+6 << " max : " << d1_max * 1.0E+6 << " min : " << d1_min * 1.0E+6 << std::endl;

                                d_min = 1000;
                                d_max = 0;
                                d_mean = 0;
                                c_count = 0;
                                oc_cnt = 0;

                                d1_min = 1000;
                                d1_max = 0;
                                d1_mean = 0;

                                pwait_time = pwait_time + 1.0;
                            }

                            if (dc.print_elmo_info_tofile)
                            {
                                file_homming << hommingElmo[2] << "\t" << hommingElmo[3] << "\t" << hommingElmo[4] << "\t" << hommingElmo[5] << "\t" << hommingElmo[6] << "\t" << hommingElmo[7] << "\t" << hommingElmo[8] << "\t" << hommingElmo[9] << "\t" << hommingElmo[10] << "\t" << hommingElmo[11] << "\t" << hommingElmo[12] << "\t" << hommingElmo[13] << "\t" << hommingElmo[14] << "\t" << hommingElmo[15] << "\t" << hommingElmo[16] << "\t" << hommingElmo[17] << "\t" << hommingElmo[18] << "\t" << hommingElmo[19] << endl;
                                file_homming << hommingElmo[18] << "\t" << positionElmo[18] << "\t" << hommingElmo[19] << "\t" << positionElmo[19] << "\t" << hommingElmo[26] << "\t" << positionElmo[26] << "\t" << endl;
                            }

                            cycle_count++;
                        }

                        inOP = FALSE;
                    }
                    else
                    {
                        printf("%sELMO : Not all slaves reached operational state.%s\n", cred.c_str(), creset.c_str());
                        ec_readstate();
                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            if (ec_slave[slave - 1].state != EC_STATE_OPERATIONAL)
                            {
                                printf("%sELMO : EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s%s\n", cred.c_str(), slave - 1, ec_slave[slave - 1].state, ec_slave[slave - 1].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave - 1].ALstatuscode), creset.c_str());
                            }
                        }
                    }
                    printf("\nELMO : Request init state for all slaves\n");
                    /** request INIT state for all slaves
             *  slave number = 0 -> write to all slaves
             */
                    ec_slave[0].state = EC_STATE_INIT;
                    ec_writestate(0);
                }
                else
                {
                    std::cout << cred << "ELMO : Ethercat Slave Count insufficient ! model_dof : " << MODEL_DOF << " , ec slave count : " << ec_slavecount << creset << std::endl;
                    ElmoTerminate = true;
                }
            }

            printf("ELMO : Checking EC STATE ... \n");
            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            printf("ELMO : Checking EC STATE Complete \n");
        }
        else
        {
            printf("%sELMO : No slaves found!%s\n", cred.c_str(), creset.c_str());
            ElmoTerminate = true;
        }
    }
    else
    {
        printf("ELMO : No socket connection on %s\nExcecute as root\n", ifname);
        ElmoTerminate = true;
    }

    std::cout << cyellow << "Ethercat Thread End !" << creset << std::endl;
    ElmoTerminate = true;
    shutdown_tocabi_bool = true;
}
double RealRobotInterface::elmoJointMove(double init, double angle, double start_time, double traj_time)
{
    double des_pos;

    if (control_time_ < start_time)
    {
        des_pos = init;
    }
    else if ((control_time_ >= start_time) && (control_time_ < (start_time + traj_time)))
    {
        des_pos = init + angle * (control_time_ - start_time) / traj_time;
    }
    // else if ((control_time_ >= (start_time + traj_time)) && (control_time_ < (start_time + 3 * traj_time)))
    //{
    //    des_pos = init + angle - 2 * angle * (control_time_ - (start_time + traj_time)) / traj_time;
    //}
    else if (control_time_ > (start_time + traj_time))
    {
        des_pos = init + angle;
    }

    return des_pos;
}

void RealRobotInterface::imuThread()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_from_begin;

    std::chrono::microseconds cycletime(1000);

    mscl::Connection con_;

    bool imu_connection_error = false;

    try
    {
        con_ = mscl::Connection::Serial("/dev/ttyACM0", 115200);
    }
    catch (mscl::Error &err)
    {
        std::cout << cred << "IMU CONNECTION ERROR : " << err.what() << creset << std::endl;
        imu_connection_error = true;
    }

    if (!imu_connection_error)
    {
        mscl::InertialNode node(con_);
        sensor_msgs::Imu imu_msg;
        MX5IMU mx5(dc, node);
        mx5.initIMU();
        int cycle_count = 0;

        std::cout << "IMU Thread Start ! " << std::endl;
        while (!shutdown_tocabi_bool)
        {
            //std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
            cycle_count++;
            //Code here
            //
            imu_msg = mx5.getIMU();
            q_virtual_(3) = imu_msg.orientation.x;
            q_virtual_(4) = imu_msg.orientation.y;
            q_virtual_(5) = imu_msg.orientation.z;
            q_virtual_(MODEL_DOF_VIRTUAL) = imu_msg.orientation.w;

            q_dot_virtual_(3) = imu_msg.angular_velocity.x;
            q_dot_virtual_(4) = imu_msg.angular_velocity.y;
            q_dot_virtual_(5) = imu_msg.angular_velocity.z;

            q_ddot_virtual_(0) = imu_msg.linear_acceleration.x;
            q_ddot_virtual_(1) = imu_msg.linear_acceleration.y;
            q_ddot_virtual_(2) = imu_msg.linear_acceleration.z;
        }
        mx5.endIMU();
    }

    std::cout << cyellow << "IMU Thread End!" << creset << std::endl;
}
void RealRobotInterface::ftsensorThread()
{
    //wait for
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_from_begin;

    std::chrono::microseconds cycletime(1000);

    int cycle_count = 0;

    sensoray826_dev ft = sensoray826_dev(1);
    ft.open();

    pub_to_gui(dc, "ftgood");
    ft.analogSingleSamplePrepare(slotAttrs, 16);
    //ft.initCalibration();

    while (!shutdown_tocabi_bool)
    {
        std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
        cycle_count++;

        ft.analogOversample();
        ft.computeFTData();

        //RF_FT(0) = ...
        //LF_FT(0) = ...

        if (dc.ftcalib) //enabled by gui
        {
            std::cout << "ft sensor : calibration ..."<<std::endl;
            pub_to_gui(dc, "ft sensor : calibration ... ");

            dc.ftcalib = false;
        }

        /*    if(cycle_count % 400 == 0)
        {
            printf("FTsensorL x : %f \t y : %f \t z : %f\n", ft.leftFootAxisData[0], ft.leftFootAxisData[1], ft.leftFootAxisData[2]);
            printf("FTsensorR x : %f \t y : %f \t z : %f\n", ft.rightFootAxisData[0], ft.rightFootAxisData[1], ft.rightFootAxisData[2]);
        }*/
    }

    std::cout << "FTsensor Thread End!" << std::endl;
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

void RealRobotInterface::gainCallbak(const std_msgs::Float32MultiArrayConstPtr &msg)
{

    std::cout << "customgain Command received ! " << std::endl;
    for (int i = 0; i < MODEL_DOF; i++)
    {
        CustomGain[i] = msg->data[i];
        std::cout << CustomGain[i] << "\t";
    }
    std::cout << std::endl;
    dc.customGain = true;
}
