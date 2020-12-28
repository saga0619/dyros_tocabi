#include "tocabi_controller/realrobot_interface.h"
#include "sensoray826/sensoray826.h"
#include "optoforce_can/optoforce_can.h"
#include <errno.h>
#include <bitset>
#include <ros/package.h>

const char *homedir;

std::atomic_bool atb_q = true;
std::atomic_bool atb_elmo = true;

double rising_time = 3.0;
bool elmo_init = true;
bool elmo_init_lower = false;
bool elmo_init_upper = false;
bool elmo_waiting_upperinit_commnad = false;
bool elmo_waiting_lowinit_command = false;

int roundtoint(double x)
{
    if (x >= 0)
        return (int)(x + 0.5);
    return (int)(x - 0.5);
}

RealRobotInterface::RealRobotInterface(DataContainer &dc_global) : dc(dc_global), StateManager(dc_global)
{
    gainSubscriber = dc.nh.subscribe("/tocabi/gain_command", 100, &RealRobotInterface::gainCallback, this);
    commandSubscriber = dc.nh.subscribe("/tocabi/torquemanual", 100, &RealRobotInterface::tcommandCallback, this);

    //pack_path = ros::package::getPath("tocabi_controller");
    zp_path = dc.homedir + "/zeropoint";
    zplog_path = dc.homedir + "/elmostart_log";
    ft_init_path = dc.homedir + "/ftinit_log";

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
    q_dot_before_.setZero();

    positionZeroModElmo(8) = 15.46875 * DEG2RAD;
    positionZeroModElmo(7) = 16.875 * DEG2RAD;
    positionZeroModElmo(TOCABI::Waist1_Joint) = -15.0 * DEG2RAD;
    positionZeroModElmo(TOCABI::Upperbody_Joint) = 0.0541;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc.currentGain(i) = NM2CNT[i];
    }

    //file_homming.open(dc.homedir + "/hommingcheck.txt", ios_base::out);
    //ft_sensor.open(dc.homedir + "/Ftsensorcheck.txt", ios_base::out);
    //file_homming << "R7\tR8\tL8\tL7\tL3\tL4\tR4\tR3\tR5\tR6\tL6\tL5\tL1\tL2\tR2\tR1\tw1\tw2" << endl;

    fz_group1.resize(18);
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
    fz_group1[i++] = TOCABI::R_Wrist2_Joint;

    fz_group1[i++] = TOCABI::L_Shoulder1_Joint;
    fz_group1[i++] = TOCABI::L_Shoulder2_Joint;
    fz_group1[i++] = TOCABI::L_Shoulder3_Joint;
    fz_group1[i++] = TOCABI::L_Armlink_Joint;
    fz_group1[i++] = TOCABI::L_Elbow_Joint;
    fz_group1[i++] = TOCABI::L_Forearm_Joint;
    fz_group1[i++] = TOCABI::L_Wrist1_Joint;
    fz_group1[i++] = TOCABI::L_Wrist2_Joint;

    elmofz[TOCABI::R_Armlink_Joint].init_direction = -1.0;
    elmofz[TOCABI::L_Armlink_Joint].init_direction = -1.0;
    elmofz[TOCABI::R_Elbow_Joint].init_direction = -1.0;
    elmofz[TOCABI::Upperbody_Joint].init_direction = -1.0;

    elmofz[TOCABI::Waist2_Joint].init_direction = -1.0;

    elmofz[TOCABI::R_Elbow_Joint].req_length = 0.06;
    elmofz[TOCABI::L_Elbow_Joint].req_length = 0.09;
    elmofz[TOCABI::L_Forearm_Joint].req_length = 0.09;
    elmofz[TOCABI::R_Forearm_Joint].req_length = 0.14;

    elmofz[TOCABI::L_Shoulder1_Joint].req_length = 0.18;
    elmofz[TOCABI::L_Shoulder2_Joint].req_length = 0.17;
    elmofz[TOCABI::R_Shoulder2_Joint].req_length = 0.08;

    elmofz[TOCABI::R_Shoulder3_Joint].req_length = 0.03;
    elmofz[TOCABI::L_Shoulder3_Joint].req_length = 0.04;

    elmofz[TOCABI::R_Wrist2_Joint].req_length = 0.05;
    elmofz[TOCABI::L_Wrist2_Joint].req_length = 0.05;

    elmofz[TOCABI::Waist2_Joint].req_length = 0.07;
    elmofz[TOCABI::Waist2_Joint].init_direction = -1.0;
    elmofz[TOCABI::Waist1_Joint].req_length = 0.07;

    fz_group2.resize(3);
    i = 0;
    fz_group2[i++] = TOCABI::Upperbody_Joint;
    fz_group2[i++] = TOCABI::Waist2_Joint;
    fz_group2[i++] = TOCABI::Waist1_Joint;

    fz_group3.resize(12);
    for (int i = 0; i < 6; i++)
    {
        fz_group3[i] = TOCABI::R_HipYaw_Joint + i;
        fz_group3[i + 6] = TOCABI::L_HipYaw_Joint + i;
    }
}

RealRobotInterface::~RealRobotInterface()
{
    std::cout << "RR Destructor" << std::endl;
}

void RealRobotInterface::updateState()
{
    //State is updated by main state loop of realrobot interface !

    control_time_ = control_time_real_;

    ros::spinOnce();

    if (atb_q)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            rq_[i] = req_[i];
            rq_dot_[i] = req_dot_[i];
            rq_ext_[i] = req_ext_[i];
            rq_elmo_[i] = req_elmo_[i];
        }

        for (int i = 0; i < ec_slavecount; i++)
        {
            q_[i] = rq_[TOCABI::JointMap[i]];
            q_dot_[i] = rq_dot_[TOCABI::JointMap[i]];
            q_ext_[i] = rq_ext_[TOCABI::JointMap[i]];
            torque_elmo_[i] = rq_elmo_[TOCABI::JointMap[i]];
        }

        q_ddot_ = q_dot_ - q_dot_before_;
        q_dot_before_ = q_dot_;

        q_virtual_local_.setZero();
        q_virtual_local_.segment(3, 3) = imu_quat.segment(0, 3);
        q_virtual_local_(MODEL_DOF_VIRTUAL) = imu_quat(3);
        q_virtual_local_.segment(6, MODEL_DOF) = q_;

        q_dot_virtual_local_.setZero();
        q_dot_virtual_local_.segment(3, 3) = imu_ang_vel;
        q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;

        q_ddot_virtual_local_.setZero();
        q_ddot_virtual_local_.segment(0, 3) = imu_lin_acc;
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5));
        if (atb_q)
        {
            for (int i = 0; i < MODEL_DOF; i++)
            {
                rq_[i] = req_[i];
                rq_dot_[i] = req_dot_[i];
                rq_ext_[i] = req_ext_[i];
                rq_elmo_[i] = req_elmo_[i];
            }

            for (int i = 0; i < ec_slavecount; i++)
            {
                q_[i] = rq_[TOCABI::JointMap[i]];
                q_dot_[i] = rq_dot_[TOCABI::JointMap[i]];
                q_ext_[i] = rq_ext_[TOCABI::JointMap[i]];
                torque_elmo_[i] = rq_elmo_[TOCABI::JointMap[i]];
            }

            q_ddot_ = q_dot_ - q_dot_before_;
            q_dot_before_ = q_dot_;

            q_virtual_local_.setZero();
            q_virtual_local_.segment(3, 3) = imu_quat.segment(0, 3);
            q_virtual_local_(MODEL_DOF_VIRTUAL) = imu_quat(3);
            q_virtual_local_.segment(6, MODEL_DOF) = q_;

            q_dot_virtual_local_.setZero();
            q_dot_virtual_local_.segment(3, 3) = imu_ang_vel;
            q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;

            q_ddot_virtual_local_.setZero();
            q_ddot_virtual_local_.segment(0, 3) = imu_lin_acc;
        }
        else
        {
            std::cout << "2nd try failed update state blocked since atb_q is locked " << std::endl;
            q_virtual_local_.segment(3, 3) = imu_quat.segment(0, 3);
            q_virtual_local_(MODEL_DOF_VIRTUAL) = imu_quat(3);
        }
    }
}

Eigen::VectorQd RealRobotInterface::getCommand()
{
    /*
    Eigen::VectorQd t1_, t2_;

    mtx_elmo_command.lock();

    t1_ = torqueDesiredController;
    mtx_elmo_command.unlock();

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
    return t2_;*/
}

int RealRobotInterface::checkTrajContinuity(int slv_number)
{
}

void RealRobotInterface::checkJointLimit(int slv_number)
{
}

void RealRobotInterface::checkSafety(int slv_number, double max_vel, double max_dis)
{
    bool damping_mode = false;
    if (ElmoSafteyMode[slv_number] == 0)
    {
        if (dc.zp_state == 2)
        {
            if (checkPosSafety[slv_number])
            {
                if (abs(positionDesiredElmo(slv_number) - positionDesiredElmo_Before(slv_number)) > max_dis * 10.0)
                {
                    std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " trajectory discontinuity : " << (positionDesiredElmo(slv_number) - positionDesiredElmo_Before(slv_number)) << creset << std::endl;
                    pub_to_gui(dc, "Lock %d %s , traj err", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
                    dc.safetyison = true;
                    ElmoSafteyMode[slv_number] = 1;
                    positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
                }
            }
            if (ElmoMode[slv_number] == EM_POSITION)
            {
                if (abs(positionDesiredElmo(slv_number) - positionElmo(slv_number)) > max_dis * 10.0)
                {
                    dc.safetyison = true;
                    std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Position Command discontinuity : " << (positionDesiredElmo(slv_number) - positionElmo(slv_number)) << creset << std::endl;
                    pub_to_gui(dc, "Lock %d %s , command err", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
                    ElmoSafteyMode[slv_number] = 1;
                    positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
                }
            }

            if (abs(velocityElmo(slv_number)) > max_vel)
            {
                dc.safetyison = true;
                std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Velocity Over Limit" << creset << std::endl;
                pub_to_gui(dc, "Lock %d %s , velocity err", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
                ElmoSafteyMode[slv_number] = 1;
                positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
            }
        }

        if (dc.zp_state == 2)
        {
            if (((positionElmo[slv_number] - positionZeroElmo[slv_number]) > jointLimitUp[slv_number]) || ((positionElmo[slv_number] - positionZeroElmo[slv_number]) < jointLimitLow[slv_number]))
            {
                dc.safetyison = true;
                std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Joint Limit exceeded, " << creset << std::endl;
                pub_to_gui(dc, "Lock %d %s , joint limit reached ", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
                ElmoSafteyMode[slv_number] = 1;
                positionSafteyHoldElmo[slv_number] = positionElmo[slv_number];
            }
        }
    }

    if (ElmoSafteyMode[slv_number] == 1)
    {
        if (!dc.safetycheckdisable)
        {
            txPDO[slv_number]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
            txPDO[slv_number]->targetPosition = (int)(Dr[slv_number] * RAD2CNT[slv_number] * positionSafteyHoldElmo[slv_number]);
            if (abs(positionElmo[slv_number] - positionSafteyHoldElmo[slv_number]) > 0.15)
            {
                ElmoSafteyMode[slv_number] = 2;
                std::cout << cred << "WARNING MOTOR " << slv_number << " , " << TOCABI::ELMO_NAME[slv_number] << " Holding Malfunction! Maybe current off? Torque zero!" << creset << std::endl;
            }
        }
    }

    if (ElmoSafteyMode[slv_number] == 2)
    {
        txPDO[slv_number]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        txPDO[slv_number]->targetTorque = 0;
    }
}

void RealRobotInterface::findZeroLeg()
{
    //std::cout << "lower leg check " << std::endl;
    for (int i = 0; i < 6; i++)
    {
        positionZeroElmo[i + TOCABI::R_HipYaw_Joint] = positionElmo[i + TOCABI::R_HipYaw_Joint] - positionExternalElmo[i + TOCABI::R_HipYaw_Joint];
        pub_to_gui(dc, "jointzp %d %d", i + TOCABI::R_HipYaw_Joint, 1);
        positionZeroElmo[i + TOCABI::L_HipYaw_Joint] = positionElmo[i + TOCABI::L_HipYaw_Joint] - positionExternalElmo[i + TOCABI::L_HipYaw_Joint];
        pub_to_gui(dc, "jointzp %d %d", i + TOCABI::L_HipYaw_Joint, 1);
        //std::cout << TOCABI::ELMO_NAME[i + TOCABI::R_HipRoll_Joint] << " pz IE P : " << positionElmo[i + TOCABI::R_HipRoll_Joint] << " pz EE P : " << positionExternalElmo[i + TOCABI::R_HipRoll_Joint] << std::endl;
        //std::cout << TOCABI::ELMO_NAME[i + TOCABI::L_HipRoll_Joint] << " pz ELMO : " << positionElmo[i + TOCABI::L_HipRoll_Joint] << " pz ELMO : " << positionExternalElmo[i + TOCABI::L_HipRoll_Joint] << std::endl;
    }

    //1. set init position of low

    //2.
}
void RealRobotInterface::findZeroPointlow(int slv_number)
{
    double velocity = 0.1;
    double fztime = 3.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        elmofz[slv_number].initTime = control_time_real_;
        elmofz[slv_number].initPos = positionElmo[slv_number];
        elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;

        if (positionExternalElmo[slv_number] > 0)
        {
            elmofz[slv_number].init_direction = -1;
        }
        else
        {
            elmofz[slv_number].init_direction = 1;
        }

        if ((positionExternalElmo[slv_number] > 3.14) || (positionExternalElmo[slv_number < -3.14]))
        {

            std::cout << cred << "elmo reboot required. joint " << slv_number << "external encoder error" << positionExternalElmo[slv_number] << std::endl;
        }
        else if (slv_number == 24)
        {
            std::cout << "positionExternal OK " << positionExternalElmo[slv_number] << std::endl;
        }
    }

    if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {

        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.6, elmofz[slv_number].initTime, fztime * 4.0);

        if (control_time_real_ == elmofz[slv_number].initTime)
        {
            //std::cout << "joint " << slv_number << "  init pos : " << elmofz[slv_number].initPos << "   goto " << elmofz[slv_number].initPos + elmofz[slv_number].init_direction * 0.6 << std::endl;
        }

        if (positionExternalElmo[slv_number] * elmofz[slv_number].init_direction > 0)
        {
            positionZeroElmo[slv_number] = positionElmo[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
            pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
        }
        if (control_time_real_ > elmofz[slv_number].initTime + fztime * 4.0)
        {
            elmofz[slv_number].result == ElmoHommingStatus::FAILURE;
        }
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
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
            elmofz[slv_number].firstPos = positionElmo[slv_number];
        }
        else
        {
            //std::cout << "motor " << slv_number << " init state : homming off" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMING;
            elmofz[slv_number].initTime = control_time_real_;
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
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].posStart = positionElmo[slv_number];
            elmofz[slv_number].initPos = positionElmo[slv_number];
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
                elmofz[slv_number].findZeroSequence = FZ_GOTOZEROPOINT;
                elmofz[slv_number].initPos = positionElmo[slv_number];
                positionZeroElmo[slv_number] = (elmofz[slv_number].posEnd + elmofz[slv_number].posStart) * 0.5 + positionZeroModElmo[slv_number];
                elmofz[slv_number].initTime = control_time_real_;
                //std::cout << "on : Motor " << slv_number << " zero point found : " << positionZeroElmo[slv_number] << std::endl;
            }
        }

        if (control_time_real_ > elmofz[slv_number].initTime + fztime)
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMING)
    { //start from unknown

        ElmoMode[slv_number] = EM_POSITION;
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.3, elmofz[slv_number].initTime, fztime);
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime))
        {
            positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos + 0.3 * elmofz[slv_number].init_direction, -0.6 * elmofz[slv_number].init_direction, elmofz[slv_number].initTime + fztime, fztime * 2.0);
        }

        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            //std::cout << "homming found ! to sequence 1 ! " << std::endl;
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }

        if (control_time_real_ > (elmofz[slv_number].initTime + fztime * 3.0))
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_GOTOZEROPOINT)
    {
        ElmoMode[slv_number] = EM_POSITION;

        double go_to_zero_dur = fztime * (abs(positionZeroElmo(slv_number) - elmofz[slv_number].initPos) / 0.3);
        positionDesiredElmo[slv_number] = elmoJointMove(elmofz[slv_number].initPos, positionZeroElmo(slv_number) - elmofz[slv_number].initPos, elmofz[slv_number].initTime, go_to_zero_dur);

        //go to zero position
        if (control_time_real_ > (elmofz[slv_number].initTime + go_to_zero_dur))
        {
            //std::cout << "go to zero complete !" << std::endl;
            //printf("Motor %d %s : Zero Point Found : %8.6f, homming length : %8.6f ! \n", slv_number, TOCABI::ELMO_NAME[slv_number].c_str(), positionZeroElmo[slv_number], abs(elmofz[slv_number].posStart - elmofz[slv_number].posEnd));
            pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
            //std::cout << slv_number << "Start : " << elmofz[slv_number].posStart << "End:" << elmofz[slv_number].posEnd << std::endl;
            //positionDesiredElmo[slv_number] = positionZeroElmo(slv_number);
            elmofz[slv_number].findZeroSequence = 8; // torque to zero -> 8 position hold -> 5
            ElmoMode[slv_number] = EM_TORQUE;
            torqueDemandElmo[slv_number] = 0.0;
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
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime))
        {
            elmofz[slv_number].findZeroSequence = 7;
            printf("Motor %d %s : Zero point detection Failed. Manual Detection Required. \n", slv_number, TOCABI::ELMO_NAME[slv_number].c_str());
            pub_to_gui(dc, "jointzp %d %d", slv_number, 2);
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            elmofz[slv_number].initTime = control_time_real_;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 7)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torqueDemandElmo[slv_number] = 0.0;
        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = positionElmo[slv_number];
        }
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime_manual))
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

void RealRobotInterface::sendCommand(Eigen::VectorQd command, double sim_time, int control_mode)
{
    double elmo_command[MODEL_DOF];

    for (int i = 0; i < MODEL_DOF; i++)
        elmo_command[TOCABI::JointMap[i]] = command[i];

    if (atb_elmo)
        std::copy(elmo_command, elmo_command + MODEL_DOF, ELMO_torquecommand);
    torque_desired = command;
}

void RealRobotInterface::ethercatCheck()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ethercatCheck Thread Start" << std::endl;
    while ((!shutdown_tocabi_bool))
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
                            printf("%sERROR : slave %d lost : %s%s\n", cred.c_str(), slave - 1, TOCABI::ELMO_NAME[slave - 1].c_str(), creset.c_str());
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
    std::cout << cyellow << "Ethercat Check Thread End !" << creset << std::endl;
}

void RealRobotInterface::ethercatThreadLower()
{
}

void RealRobotInterface::ethercatThreadUpper()
{
}

void RealRobotInterface::ethercatCheckLower()
{
}

void RealRobotInterface::ethercatCheckUpper()
{
}

void RealRobotInterface::ethercatThread()
{

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ethercatThread Start" << std::endl;

    char IOmap[4096];
    bool reachedInitial[MODEL_DOF] = {false};

    bool exit_middle = false;

    const char *ifname = dc.ifname.c_str();
    const char *ifname2 = dc.ifname2.c_str();

    if (ec_init(ifname))
    //if(ec_init_redundant(ifname, (char *)ifname2))
    {
        printf("ELMO : ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        /* network discovery */
        //ec_config_init()
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("ELMO : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount == ELMO_DOF)
            {
                ecat_number_ok = true;
            }
            else
            {
                std::cout << cred << "WARNING : SLAVE NUMBER INSUFFICIENT" << creset << std::endl;
            }
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
                if (true)
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
                        //0x1a1e :  Auxiliary position value    32bit
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
                    if (expectedWKC != 3 * ELMO_DOF)
                    {
                        std::cout << cred << "WARNING : Calculated Workcounter insufficient!" << creset << std::endl;
                        ecat_WKC_ok = true;
                    }
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
                        dc.rgbPubMsg.data = {0, 0, 0, 0, 0, 0, 64, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0};
                        dc.rgbPub.publish(dc.rgbPubMsg);
                        if (ecat_number_ok && ecat_WKC_ok)
                        {
                            pub_to_gui(dc, "All slaves Status GREEN");
                        }
                        else
                        {
                            pub_to_gui(dc, "Please Check Slave status");
                        }

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

                        std::chrono::steady_clock::time_point tp[10];
                        std::chrono::steady_clock::time_point time_until;

                        std::chrono::duration<double> td[8];
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

                        for (int i = 0; i < ec_slavecount; i++)
                        {
                            ElmoSafteyMode[i] = 0;
                        }

                        for (int i = 0; i < MODEL_DOF; i++)
                        {
                            ELMO_NM2CNT[i] = dc.tocabi_.vector_NM2CNT[i];
                        }

                        dc.connected = true;
                        st_start_time = std::chrono::steady_clock::now();
                        dc.start_time_point = st_start_time;

                        while (!shutdown_tocabi_bool)
                        {
                            //std::cout<<"firstrealrobot"<<std::endl;
                            //Ethercat Loop begins :: RT Thread
                            tp[0] = std::chrono::steady_clock::now();

                            std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);

                            tp[1] = std::chrono::steady_clock::now();
                            time_from_begin = std::chrono::steady_clock::now() - st_start_time;
                            control_time_real_ = time_from_begin.count();

                            static bool first = true;
                            if (first)
                            {
                                first = false;
                            }
                            else
                            {
                                int time_diff = (int)((control_time_real_ - control_time_before_) * 1.0E+6);

                                if ((time_diff > dc.ctime * 1.05) || (time_diff < dc.ctime * 0.95))
                                {
                                    std::cout << cred << "Warning : Time is not OK, time diff is : " << time_diff << std::endl;
                                    std::cout << "current time : " << control_time_real_ << std::endl;
                                    std::cout << "before time : " << control_time_before_ << std::endl;
                                    std::cout << "If you check this message, please let junhee know" << std::endl;
                                }
                            }
                            control_time_before_ = control_time_real_;

                            /** PDO I/O refresh */
                            //ec_send_processdata();

                            tp[2] = std::chrono::steady_clock::now();
                            wkc = ec_receive_processdata(0);

                            tp[3] = std::chrono::steady_clock::now();

                            td[0] = st_start_time + cycle_count * cycletime - tp[0];
                            td[1] = tp[1] - tp[0];

                            td[2] = tp[2] - tp[1];
                            td[3] = tp[3] - tp[2];

                            if (td[3] > std::chrono::microseconds(250))
                            {
                                oc_cnt++;
                            }

                            if (tp[3] > st_start_time + (cycle_count + 1) * cycletime)
                            {
                                std::cout << cred << "## ELMO LOOP INSTABILITY DETECTED ##\n START TIME DELAY :" << -td[0].count() * 1E+6 << " us\n THREAD SYNC TIME : " << td[1].count() * 1E+6 << " us\n RECV ELMO TIME : " << td[3].count() * 1E+6 << " us \t last tick : " << td[5].count() * 1E+6 << " us \t last send process : " << td[6].count() * 1E+6 << " us\n LAST LOOP :" << td[4].count() * 1E+6 << " us\n " << creset << std::endl;
                                std::cout << cred << "If you see this warning message often, Rebooting tocabi is recommanded" << creset << std::endl;
                                dc.elmoinstability = true;
                                if (dc.torqueOn)
                                {
                                    for (int i = 0; i < ec_slavecount; i++)
                                    {
                                        positionSafteyHoldElmo[i] = positionElmo[i];
                                        ElmoSafteyMode[i] = 1;
                                    }
                                }

                                while (tp[3] > st_start_time + (cycle_count + 1) * cycletime)
                                {

                                    cycle_count++;
                                }
                            }
                            td[5] = td[3];
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
                                            EXTCNT2RAD[slave - 1] * EXTDr[slave - 1];

                                        ElmoConnected = true;

                                        if (slave == 1 || slave == 2 || slave == 19 || slave == 20)
                                        {
                                            hommingElmo[slave - 1] = !hommingElmo[slave - 1];
                                        }

                                        txPDO[slave - 1]->maxTorque = (uint16)1500; // originaly 1000
                                        ElmoMode[slave - 1] = EM_TORQUE;
                                        torqueDemandElmo[slave - 1] = 0.0;
                                    }
                                }
                            }
                            //tp[4] = std::chrono::steady_clock::now();

                            atb_q = false;
                            for (int i = 0; i < ec_slavecount; i++)
                                req_[i] = positionElmo[i] - positionZeroElmo[i];
                            std::copy(velocityElmo.data(), velocityElmo.data() + ec_slavecount, req_dot_);
                            std::copy(positionExternalElmo.data(), positionExternalElmo.data() + ec_slavecount, req_ext_);
                            std::copy(torqueElmo.data(), torqueElmo.data() + ec_slavecount, req_elmo_);
                            atb_q = true;

                            //Get State Seqence End, user controller start
                            checkfirst = (stateElmo[19] & al);

                            if (shutdown_tocabi_bool || shutdown_tocabi_bool)
                            {
                                ElmoTerminate = true;
                                //std::terminate();
                                break;
                            }

                            tp[4] = std::chrono::steady_clock::now();

                            if (commutation_check)
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
                                        //std::chrono::system_clock::now().coun
                                        int time_now_from_ros = ros::Time::now().toSec();
                                        std::cout << cgreen << "ELMO : READY, WARMSTART ! LOADING ZERO POINT ... " << time_now_from_ros - (int)(floor(time_now_from_ros / 1E+5) * 1E+5) << creset << std::endl;
                                        pub_to_gui(dc, "ELMO WARM START, LOADING ZP ");

                                        bool waitop = true;
                                        for (int i = 0; i < ec_slavecount; i++)
                                        {
                                            waitop = waitop && ((stateElmo[i] & al) == 39);
                                        }
                                        if (waitop)
                                        {
                                            pub_to_gui(dc, "ecatgood");
                                            dc.ecat_state = 1;
                                            commutation_ok = true;
                                            bootseq++;
                                        }

                                        string tmp;
                                        int last_boot_time = 1;
                                        int last_save_time = 0;
                                        double zp[ec_slavecount];

                                        elmo_zp_log.open(zplog_path, ios_base::in);

                                        if (elmo_zp_log.is_open())
                                        {
                                            getline(elmo_zp_log, tmp);
                                            elmo_zp_log.close();
                                            last_boot_time = atoi(tmp.c_str());
                                            std::cout << "ECAT BOOT TIME LOADED : " << last_boot_time - (int)(floor(last_boot_time / 1E+5) * 1E+5) << std::endl;
                                        }
                                        else
                                        {
                                            std::cout << "ECAT BOOT TIME LOAD FILED" << std::endl;
                                        }
                                        elmo_zp.open(zp_path, ios_base::in);

                                        if (elmo_zp.is_open())
                                        {
                                            getline(elmo_zp, tmp);
                                            last_save_time = atoi(tmp.c_str());
                                            std::cout << "ZP LOG TIME LOADED : " << last_save_time - (int)(floor(last_save_time / 1E+5) * 1E+5) << std::endl;
                                        }
                                        else
                                        {
                                            std::cout << "ZP LOG TIME LOAD FAILED" << std::endl;
                                        }

                                        if (last_save_time < last_boot_time)
                                        {
                                            cout << "ELMO : ERROR WITH LOADING ZERO POINT! zp terr" << std::endl;
                                            pub_to_gui(dc, "ZP LOADING FAILURE");
                                            elmo_zp.close();
                                            zp_load_ok = false;
                                            commutation_ok = true;
                                        }
                                        else
                                        {
                                            if (elmo_zp.is_open())
                                            {
                                                for (int i = 0; i < ec_slavecount; i++)
                                                {
                                                    if (elmo_zp.eof())
                                                    {
                                                        std::cout << "ELMO : ERROR WITH LOADING ZERO POINT zp eof" << std::endl;
                                                        pub_to_gui(dc, "ZP LOADING FAILURE : proceed initialize");

                                                        commutation_ok = true;
                                                        zp_load_ok = false;

                                                        break;
                                                    }
                                                    getline(elmo_zp, tmp);
                                                    zp[i] = atof(tmp.c_str());
                                                }

                                                if (zp_load_ok)
                                                {
                                                    std::cout << cgreen << "ELMO : ZERO POINT LOADED ! " << creset << std::endl;
                                                    pub_to_gui(dc, "ZP LOADING SUCCESS");
                                                    pub_to_gui(dc, "zpgood");
                                                    pub_to_gui(dc, "ecatgood");
                                                    elmo_zp.close();
                                                    for (int i = 0; i < ec_slavecount; i++)
                                                    {
                                                        positionZeroElmo[i] = zp[i];
                                                    }
                                                    positionInitialElmo = positionElmo;

                                                    commutation_check = false;
                                                    commutation_ok = true;
                                                    zp_load_ok = true;
                                                    dc.elmo_Ready = true;
                                                    dc.zp_state = 2;
                                                    dc.ecat_state = 1;
                                                    operation_ready = true;
                                                }
                                            }
                                        }
                                    }
                                    if ((bootseq == 5) && (checkfirst == 64))
                                    {
                                        std::cout << cyellow << "ELMO : COMMUTATION INITIALIZING ! " << creset << std::endl;
                                        pub_to_gui(dc, "ELMO : COMMUTATION INITIALIZING ! ");
                                        pub_to_gui(dc, "ecatcommutation");
                                        dc.ecat_state = 2;
                                        bootseq = 6;
                                        zp_load_ok = false;
                                    }
                                }
                                if ((bootseq == 6))
                                {
                                    bool waitop = true;
                                    for (int i = 0; i < ec_slavecount; i++)
                                    {
                                        waitop = waitop && ((stateElmo[i] & al) == 39);
                                    }
                                    if (waitop)
                                    {
                                        std::cout << cgreen << "ELMO : COMMUTATION INITIALIZE COMPLETE" << creset << std::endl;
                                        pub_to_gui(dc, "ELMO : COMMUTATION INITIALIZE COMPLETE");
                                        pub_to_gui(dc, "ecatgood");
                                        dc.ecat_state = 1;

                                        int time_now_from_ros = ros::Time::now().toSec();
                                        elmo_zp_log.open(zplog_path, ios_base::out);
                                        if (elmo_zp_log.is_open())
                                        {
                                            elmo_zp_log << setprecision(12) << time_now_from_ros << "\n";
                                            elmo_zp_log.close();
                                            std::cout << cgreen << "ELMO : COMMUTATION LOGGED at, " << creset << time_now_from_ros - (int)(floor(time_now_from_ros / 1E+5) * 1E+5) << std::endl;
                                        }
                                        else
                                        {
                                            std::cout << cred << "ELMO : COMMUTATION LOGGING ERROR " << creset << std::endl;
                                            pub_to_gui(dc, "ELMO : COMMUTATION LOGGING ERROR ");
                                        }

                                        commutation_ok = true;

                                        bootseq++;
                                    }
                                }
                            }

                            if (commutation_ok && (!zp_load_ok) && zp_init_check)
                            {
                                std::cout << cred << "ELMO : origin searching required!" << creset << std::endl;
                                pub_to_gui(dc, "ELMO : origin searching required!");
                                dc.zp_state = 1;
                                zp_waiting_low_switch = true;
                                zp_waiting_upper_switch = true;
                                commutation_check = false;
                                zp_init_check = false;
                            }

                            checkfirst_before = checkfirst;
                            if (zp_waiting_upper_switch && dc.start_initialize_sequence)
                            {
                                dc.start_initialize_sequence = false;
                                zp_waiting_upper_switch = false;

                                zp_upper_check = true;

                                positionInitialElmo = positionElmo;
                                std::cout << cred << "ELMO : Robot Initialize Process ! Finding Origin !" << creset << std::endl;
                                pub_to_gui(dc, "ELMO : SEARCHING ZP");
                                //findZeroLeg();

                                elmofz[TOCABI::R_Shoulder3_Joint].findZeroSequence = 7;
                                elmofz[TOCABI::R_Shoulder3_Joint].initTime = control_time_real_;
                                pub_to_gui(dc, "jointzp %d %d", TOCABI::R_Shoulder3_Joint, 2);
                                elmofz[TOCABI::L_Shoulder3_Joint].findZeroSequence = 7;
                                elmofz[TOCABI::L_Shoulder3_Joint].initTime = control_time_real_;
                                pub_to_gui(dc, "jointzp %d %d", TOCABI::L_Shoulder3_Joint, 2);

                                for (int j = 0; j < ec_slavecount; j++)
                                {
                                    hommingElmo_before[j] = hommingElmo[j];
                                }
                            }

                            if (zp_waiting_low_switch && dc.start_initialize_lower)
                            {
                                dc.start_initialize_lower = false;

                                std::cout << cred << "ELMO : Robot Initialize Process ! Finding Lower leg Zero Point !" << creset << std::endl;
                                pub_to_gui(dc, "ELMO : SEARCHING LOWER BODY ZP");
                                zp_low_check = true;
                            }

                            if (zp_low_check)
                            {
                                for (int i = 0; i < 6; i++)
                                {
                                    findZeroPointlow(i + TOCABI::R_HipYaw_Joint);
                                    findZeroPointlow(i + TOCABI::L_HipYaw_Joint);
                                }
                            }

                            if (zp_upper_check)
                            {
                                if (fz_group == 0)
                                {
                                    for (int i = 0; i < fz_group1.size(); i++)
                                    {
                                        findZeroPoint(fz_group1[i]);
                                    }
                                }
                                else if (fz_group == 1)
                                {
                                    for (int i = 0; i < fz_group2.size(); i++)
                                    {
                                        findZeroPoint(fz_group2[i]);
                                    }
                                }

                                for (int i = 0; i < ELMO_DOF; i++)
                                    hommingElmo_before[i] = hommingElmo[i];
                            }

                            //checking zp success
                            if (!operation_ready)
                            {
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

                                fz_group3_check = true;
                                for (int i = 0; i < fz_group3.size(); i++)
                                {
                                    fz_group3_check = fz_group3_check && (elmofz[fz_group3[i]].result == ElmoHommingStatus::SUCCESS);
                                }

                                if (fz_group1_check && (fz_group == 0))
                                {
                                    fz_group++;
                                    std::cout << cgreen << "ELMO : Upperbody Origin Check Complete" << creset << std::endl;
                                }
                                if (fz_group2_check && (fz_group == 1))
                                {
                                    fz_group++;
                                    std::cout << cgreen << "ELMO : Waist Origin Check Complete" << creset << std::endl;
                                }
                                if (fz_group3_check)
                                {
                                    static bool once = true;
                                    if (once)
                                    {
                                        std::cout << cgreen << "ELMO : Leg Origin Check Complete" << creset << std::endl;

                                        dc.ftcalib = true;
                                        once = false;
                                    }
                                }

                                if (fz_group2_check && fz_group1_check && fz_group3_check)
                                {
                                    fz_group++;
                                    elmo_zp.open(zp_path, ios_base::out);

                                    if (elmo_zp.is_open())
                                    {
                                        int time_now_from_ros = ros::Time::now().toSec();
                                        elmo_zp << setprecision(12) << time_now_from_ros << "\n";
                                        for (int i = 0; i < ec_slavecount; i++)
                                        {
                                            elmo_zp << positionZeroElmo[i] << "\n";
                                        }
                                        elmo_zp.close();

                                        std::cout << cgreen << "ELMO : Waist zero point check complete and zero point saved. at, " << time_now_from_ros - (int)(floor(time_now_from_ros / 1E+5) * 1E+5) << creset << std::endl;
                                        pub_to_gui(dc, "ELMO : ZP SAVED");
                                    }
                                    else
                                    {
                                        std::cout << cred << "ELMO : failed to log zp " << creset << std::endl;
                                    }

                                    pub_to_gui(dc, "zpgood");
                                    dc.zp_state = 2;
                                    dc.elmo_Ready = true;
                                    operation_ready = true;
                                }
                            }

                            tp[5] = std::chrono::steady_clock::now();

                            //torqueDesiredController = getCommand();
                            if (operation_ready)
                            {
                                //torqueDesiredElmo = getCommand();
                                atb_elmo = false;
                                std::copy(ELMO_torquecommand, ELMO_torquecommand + ec_slavecount, ELMO_torque);
                                atb_elmo = true;

                                zp_low_check = false;
                                zp_upper_check = false;
                            }
                            else
                            {
                                for (int i = 0; i < ELMO_DOF; i++)
                                {
                                    ELMO_torque[i] = 0.0;
                                }
                            }

                            tp[6] = std::chrono::steady_clock::now();
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (operation_ready)
                                {
                                    if (dc.torqueOn)
                                    {
                                        //If torqueOn command received, torque will increases slowly, for rising_time, which is currently 3 seconds.
                                        to_ratio = DyrosMath::minmax_cut((control_time_real_ - dc.torqueOnTime) / rising_time, 0.0, 1.0);
                                        ElmoMode[i] = EM_TORQUE;
                                        dc.t_gain = to_ratio;

                                        ELMO_torque[i] = to_ratio * ELMO_torque[i];
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
                                        to_ratio = DyrosMath::minmax_cut(1.0 - to_calib - (control_time_real_ - dc.torqueOffTime) / rising_time, 0.0, 1.0);

                                        dc.t_gain = to_ratio;

                                        ELMO_torque[i] = to_ratio * ELMO_torque[i];
                                    }
                                    else
                                    {
                                        ElmoMode[i] = EM_TORQUE;
                                        ELMO_torque[i] = 0.0;
                                    }
                                }
                                else
                                {
                                    if ((!zp_upper_check) && (!zp_low_check))
                                    {
                                        ElmoMode[i] = EM_TORQUE;
                                        ELMO_torque[i] = 0.0;
                                    }
                                }
                            }
                            if (torqueCCEnable)
                            {
                                if ((control_time_real_ >= torqueCC_recvt) && (control_time_real_ < (torqueCC_recvt + torqueCC_comt)))
                                {
                                    for (int i = 0; i < MODEL_DOF; i++)
                                    {
                                        ElmoMode[i] = EM_TORQUE;
                                        ELMO_torque[i] = torqueCustomCommand[i];
                                    }
                                }

                                if (control_time_real_ > (torqueCC_recvt + torqueCC_comt))
                                {
                                    torqueCCEnable = false;
                                }
                            }
                            //ECAT JOINT COMMAND
                            for (int i = 0; i < ec_slavecount; i++)
                            {
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
                                        txPDO[i]->targetTorque = (int)(ELMO_torque[i] * CustomGain[i] * Dr[i]);
                                    }
                                    else
                                    {
                                        txPDO[i]->targetTorque = (roundtoint)(ELMO_torque[i] * ELMO_NM2CNT[i] * Dr[i]);
                                    }
                                }
                                else if (ElmoMode[i] == EM_COMMUTATION)
                                {
                                }
                                else
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }
                            bool ecat_lost_before = dc.ecat_lost;
                            dc.ecat_lost = false;
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ec_slave[i].islost)
                                {
                                    dc.ecat_lost = dc.ecat_lost || true;
                                }
                            }

                            if ((ecat_lost_before) && (!dc.ecat_lost))
                            {
                                dc.ecat_recovered = true;
                            }

                            if (dc.ecat_lost)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }

                            if (dc.ecat_recovered)
                            {
                                //All joint To stay to joint
                                std::cout << cred << "Tocabi Recoverd, Holding Current State" << creset << std::endl;
                                for (int i = 0; i < MODEL_DOF; i++)
                                {
                                    ElmoSafteyMode[i] = 1;
                                    positionSafteyHoldElmo[i] = positionElmo[i];
                                    pub_to_gui(dc, "Lock %d %s , ELMO Instability ", i, TOCABI::ELMO_NAME[i].c_str());
                                }
                                dc.ecat_recovered = false;
                            }

                            //Hold position if safety limit breached
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ElmoMode[i] != EM_POSITION)
                                {
                                    checkPosSafety[i] = false;
                                }

                                checkSafety(i, dc.safety_limit[i], 10.0 * dc.ctime / 1E+6); //if angular velocity exceeds 0.5rad/s, Hold to current Position ///
                            }

                            //Torque off if emergency off received
                            if (dc.emergencyoff)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }

                            //std::this_thread::sleep_until(st_start_time + cycle_count * cycletime+ std::chrono::microseconds(250));
                            tp[7] = std::chrono::steady_clock::now();
                            ec_send_processdata();
                            tp[8] = std::chrono::steady_clock::now();
                            td[6] = std::chrono::steady_clock::now() - tp[7];
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                dc.torqueElmo[i] = roundtoint(ELMO_torque[i] * ELMO_NM2CNT[i] * Dr[i]);
                            }

                            positionDesiredElmo_Before = positionDesiredElmo;
                            if (dc.disableSafetyLock)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    ElmoSafteyMode[i] = 0;
                                }
                                dc.disableSafetyLock = false;
                            }

                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ElmoMode[i] == EM_POSITION)
                                {
                                    checkPosSafety[i] = true;
                                }
                            }
                            tp[9] = std::chrono::steady_clock::now();
                            td[4] = tp[8] - (st_start_time + cycle_count * cycletime); //timestamp for send time consumption.
                            if (td[4].count() * 1E+6 > 500)
                            {
                                std::cout << "Loop time exceeded : " << std::endl;
                                std::cout << "sleep_until" << std::chrono::duration_cast<std::chrono::microseconds>(tp[1] - tp[0]).count() << std::endl;
                                std::cout << "checktime" << std::chrono::duration_cast<std::chrono::microseconds>(tp[2] - tp[1]).count() << std::endl;
                                std::cout << "ec_receive_processdata()" << std::chrono::duration_cast<std::chrono::microseconds>(tp[3] - tp[2]).count() << std::endl;
                                std::cout << "getdata & copy" << std::chrono::duration_cast<std::chrono::microseconds>(tp[4] - tp[3]).count() << std::endl;
                                std::cout << "findzero" << std::chrono::duration_cast<std::chrono::microseconds>(tp[5] - tp[4]).count() << std::endl;
                                std::cout << "torque copy" << std::chrono::duration_cast<std::chrono::microseconds>(tp[6] - tp[5]).count() << std::endl;
                                std::cout << "command op" << std::chrono::duration_cast<std::chrono::microseconds>(tp[7] - tp[6]).count() << std::endl;
                                std::cout << "ec_send_processdata()" << std::chrono::duration_cast<std::chrono::microseconds>(tp[8] - tp[7]).count() << std::endl;
                                std::cout << "rount to int" << std::chrono::duration_cast<std::chrono::microseconds>(tp[9] - tp[8]).count() << std::endl;
                                std::cout << "If you check this message, please let junhee know : " << std::endl;
                            }

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

                            if (control_time_real_ > pwait_time)
                            {
                                oct_cnt += oc_cnt;
                                if (dc.print_delay_info)
                                {
                                    printf("%3.0f, %d hz SEND min : %5.2f us, max : %5.2f us, avg : %5.2f us RECV min : %5.2f us, max : %5.2f us, avg %5.2f us, oc : %d, oct : %d \n", control_time_real_, c_count, d_min * 1.0E+6,
                                           d_max * 1.0E+6, d_mean / c_count * 1.0E+6, d1_min * 1.0E+6, d1_max * 1.0E+6, d1_mean * 1.0E+6 / c_count, oc_cnt, oct_cnt);

                                    pub_to_gui(dc, "%3.0f : tout : %d, total : %d", control_time_real_, oc_cnt, oct_cnt);

                                    //int al = 63;
                                    std::bitset<16> stx(stateElmo[0]);
                                    std::bitset<16> stx2(stateElmo[0] & al);
                                    //std::cout << "current statusword : " << stx << std::endl;
                                    //std::cout << stx2 << std::endl;

                                    for (int i = 0; i < ec_slavecount; i++)
                                    {
                                        if ((stateElmo[i] & al) != 39)
                                        {
                                            //std::cout << "Joint " << i << "Not operational, " << (stateElmo[i] & al) << std::endl;
                                        }
                                    }
                                }
                                //std::cout << control_time_real_ << ", " << c_count << std::setprecision(4) << " hz, min : " << d_min * 1.0E+6 << " us , max : " << d_max * 1.0E+6 << " us, mean " << d_mean / c_count * 1.0E+6 << " us"
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

    if (control_time_real_ < start_time)
    {
        des_pos = init;
    }
    else if ((control_time_real_ >= start_time) && (control_time_real_ < (start_time + traj_time)))
    {
        des_pos = init + angle * (control_time_real_ - start_time) / traj_time;
    }
    // else if ((control_time_real_ >= (start_time + traj_time)) && (control_time_real_ < (start_time + 3 * traj_time)))
    //{
    //    des_pos = init + angle - 2 * angle * (control_time_real_ - (start_time + traj_time)) / traj_time;
    //}
    else if (control_time_real_ > (start_time + traj_time))
    {
        des_pos = init + angle;
    }

    return des_pos;
}

void RealRobotInterface::imuThread()
{

    std::this_thread::sleep_for(std::chrono::seconds(1));
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
        std::chrono::steady_clock::time_point t_begin = std::chrono::steady_clock::now();
        while (!shutdown_tocabi_bool)
        {
            std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
            cycle_count++;
            //Code here
            //
            if (dc.signal_imu_reset)
            {
                dc.signal_imu_reset = false;
                mx5.resetEFIMU();
            }

            imu_msg = mx5.getIMU();

            mx5.checkIMUData();

            imu_quat(0) = imu_msg.orientation.x;
            imu_quat(1) = imu_msg.orientation.y;
            imu_quat(2) = imu_msg.orientation.z;
            imu_quat(3) = imu_msg.orientation.w;

            Vector3d ang_vel;
            ang_vel(0) = imu_msg.angular_velocity.x;
            ang_vel(1) = imu_msg.angular_velocity.y;
            ang_vel(2) = imu_msg.angular_velocity.z;
            Vector3d ang_vel_lpf;
            static Vector3d ang_vel_lpf_before;
            ang_vel_lpf = DyrosMath::lpf(ang_vel, ang_vel_lpf_before, 1000, 15);
            ang_vel_lpf_before = ang_vel_lpf;

            imu_ang_vel = ang_vel_lpf;

            Vector3d lin_acc;
            lin_acc(0) = imu_msg.linear_acceleration.x;
            lin_acc(1) = imu_msg.linear_acceleration.y;
            lin_acc(2) = imu_msg.linear_acceleration.z;
            Vector3d imu_lin_acc_lpf;
            static Vector3d imu_lin_acc_lpf_before;
            imu_lin_acc_lpf = DyrosMath::lpf(lin_acc, imu_lin_acc_lpf_before, 1000, 15);
            imu_lin_acc_lpf_before = imu_lin_acc_lpf;

            imu_lin_acc = imu_lin_acc_lpf;
        }
        mx5.endIMU();
    }

    std::cout << cyellow << "IMU Thread End!" << creset << std::endl;
}
void RealRobotInterface::ftsensorThread()
{

    //wait for
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::chrono::steady_clock::time_point t_begin = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_from_begin;

    std::chrono::microseconds cycletime(1000);

    int cycle_count = 0;
    int count = 0;
    int ft_cycle_count;
    bool is_ft_board_ok;
    bool ft_calib_init = false;
    bool ft_calib_finish = false;
    bool ft_calib_ui = false;
    bool ft_init_write = false;
    bool ft_init_load = false;

    int SAMPLE_RATE = 1000;

    sensoray826_dev ft = sensoray826_dev(1);

    /////SENSORYA826 & ATI/////
    is_ft_board_ok = ft.open();

    if (is_ft_board_ok == 1)
    {
        pub_to_gui(dc, "initreq");
        dc.ft_state = 1;
    }

    ft.analogSingleSamplePrepare(slotAttrs, 16);
    ft.initCalibration();

    while (!shutdown_tocabi_bool)
    {
        std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
        cycle_count++;

        ft.analogOversample();

        string tmp;
        int i = 0;

        if (ft_init_load == false)
        {
            ft_init_log.open(ft_init_path, ios_base::in);

            if (ft_init_log.is_open())
            {
                while (getline(ft_init_log, tmp))
                {
                    if (i < 6)
                    {
                        ft.leftFootBias[i] = atof(tmp.c_str());
                    }
                    else
                    {
                        ft.rightFootBias[i - 6] = atof(tmp.c_str());
                    }
                    i++;
                }

                ft_init_load = true;
                ft_init_log.close();
                pub_to_gui(dc, "ft bias loaded");
                dc.ft_state = 2;
            }
            else
            {
                pub_to_gui(dc, "ft bias load failed");
            }
        }

        if (dc.ftcalib) //enabled by gui
        {
            dc.ft_state = 1;
            if (ft_calib_init == false)
            {
                ft_cycle_count = cycle_count;
                ft_calib_init = true;
                ft_calib_finish = false;
                ft_calib_ui = false;

                for (int i = 0; i < 6; i++)
                {
                    ft._calibLFTData[i] = 0.0;
                    ft._calibRFTData[i] = 0.0;
                }

                pub_to_gui(dc, "ft sensor : calibration ... ");
            }
            if (cycle_count < 5 * SAMPLE_RATE + ft_cycle_count)
            {
                if (cycle_count == 5 * SAMPLE_RATE + ft_cycle_count - 1)
                {
                    ft_calib_finish = true;
                    dc.ftcalib = false;
                }

                double foot_plate_mass = 2.326;

                Matrix6d adt;
                adt.setIdentity();
                adt.block(3, 0, 3, 3) = DyrosMath::skm(-(link_local[Right_Foot].contact_point - link_local[Right_Foot].sensor_point)) * Matrix3d::Identity();
                Matrix6d rotrf;
                rotrf.setZero();
                rotrf.block(0, 0, 3, 3) = link_local[Right_Foot].Rotm;
                rotrf.block(3, 3, 3, 3) = link_local[Right_Foot].Rotm;
                Vector3d RF_com(-0.0162, 0.00008, -0.1209);

                Vector3d com2cp = link_local[Right_Foot].sensor_point - RF_com;

                Matrix6d adt2;
                adt2.setIdentity();
                adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();

                Vector6d Wrench_foot_plate;
                Wrench_foot_plate.setZero();
                Wrench_foot_plate(2) = foot_plate_mass * GRAVITY;

                RF_CF_FT = rotrf * adt * RF_FT + adt2 * Wrench_foot_plate;

                Eigen::Vector6d RF_FT_calib = (rotrf * adt).inverse() * (dc.tocabi_.ContactForce.segment(6, 6) - adt2 * Wrench_foot_plate);

                RF_CF_FT_local = rotrf.inverse() * RF_CF_FT;

                adt.setIdentity();
                adt.block(3, 0, 3, 3) = DyrosMath::skm(-(link_local[Left_Foot].contact_point - link_local[Left_Foot].sensor_point)) * Matrix3d::Identity();

                rotrf.setZero();
                rotrf.block(0, 0, 3, 3) = link_local[Left_Foot].Rotm;
                rotrf.block(3, 3, 3, 3) = link_local[Left_Foot].Rotm;

                Vector3d LF_com(-0.0162, -0.00008, -0.1209);

                com2cp = link_local[Left_Foot].contact_point - LF_com;

                adt2.setIdentity();
                adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();
                Wrench_foot_plate.setZero();
                Wrench_foot_plate(2) = foot_plate_mass * GRAVITY;

                LF_CF_FT = rotrf * adt * LF_FT + adt2 * Wrench_foot_plate;

                Eigen::Vector6d LF_FT_calib = (rotrf * adt).inverse() * (dc.tocabi_.ContactForce.segment(0, 6) - adt2 * Wrench_foot_plate);

                Eigen::Vector12d contact_force_calib;

                contact_force_calib.segment(0, 6) = LF_FT_calib;
                contact_force_calib.segment(6, 6) = RF_FT_calib;

                ft.calibrationFTData(ft_calib_finish, contact_force_calib);
            }
        }
        else
        {
            if (ft_calib_finish == false)
            {
            }
            else
            {
                ft_calib_init = false;
            }
        }

        if (ft_calib_ui == false && ft_calib_finish == true)
        {
            dc.print_ft_info_tofile = true;
            pub_to_gui(dc, "ft sensor : calibration finish ");
            pub_to_gui(dc, "ftgood");
            ROS_INFO("calibration finish");

            ft_init_log.open(ft_init_path, ios_base::out);
            if (ft_init_log.is_open())
            {
                for (int i = 0; i < 6; i++)
                {
                    ft_init_log << ft.leftFootBias[i] << "\n";
                }

                for (int i = 0; i < 6; i++)
                {
                    ft_init_log << ft.rightFootBias[i] << "\n";
                }
            }
            ft_init_log.close();
            dc.ft_state = 2;
            ft_calib_ui = true;
        }

        ft.computeFTData(ft_calib_finish);

        for (int i = 0; i < 6; i++)
        {
            RF_FT(i) = ft.rightFootAxisData[i];
            LF_FT(i) = ft.leftFootAxisData[i];
        }

        if (ft_calib_finish == false)
        {
            dc.ft_state = 1.0;
        }
        else
        {
            if (RF_FT(2) > 100 || LF_FT(2) > 100)
            {
                dc.ft_state = 0.0;
            }
            else
            {
                dc.ft_state = 2.0;
            }
        }
        if (dc.print_ft_info_tofile)
        {
        }
    }

    std::cout << cyellow << "FTsensor Thread End !" << creset << std::endl;
}

void RealRobotInterface::handftsensorThread()
{
    //wait for
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::chrono::steady_clock::time_point t_begin = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_from_begin;

    std::chrono::microseconds cycletime(500);

    int cycle_count = 0;
    int count = 0;
    int ft_cycle_count;
    bool is_ft_board_ok;
    bool ft_calib_init = false;
    bool ft_calib_finish = false;
    bool ft_calib_ui = false;

    int SAMPLE_RATE = 500;

    optoforcecan ft_upper;

    //////OPTOFORCE//////

    ft_upper.InitDriver();

    while (!shutdown_tocabi_bool)
    {
        std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
        cycle_count++;

        ft_upper.DAQSensorData();
        if (dc.ftcalib) //enabled by gui
        {
            if (ft_calib_init == false)
            {
                ft_cycle_count = cycle_count;
                ft_calib_init = true;
                //pub_to_gui(dc, "ft sensor : calibration ... ");
            }
            if (cycle_count < 5 * SAMPLE_RATE + ft_cycle_count)
            {
                if (cycle_count == 5 * SAMPLE_RATE + ft_cycle_count - 1)
                {
                    ft_calib_finish = true;
                    dc.ftcalib = false;
                }
                ft_upper.calibrationFTData(ft_calib_finish);
            }
        }
        else
        {
            //            pub_to_gui(dc, "initreq");
        }

        if (ft_calib_finish == true)
        {
            if (ft_calib_ui == false)
            {
                //pub_to_gui(dc, "ft sensor : calibration finish ");
                //pub_to_gui(dc, "ftgood");
                ft_calib_ui = true;
            }
        }

        ft_upper.computeFTData(ft_calib_finish);

        for (int i = 0; i < 6; i++)
        {
            RH_FT(i) = ft_upper.rightArmAxisData[i];
            LH_FT(i) = ft_upper.leftArmAxisData[i];
        }
    }
    std::cout << "HandFTsensor Thread End!" << std::endl;
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

void RealRobotInterface::gainCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
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

void RealRobotInterface::tcommandCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    if (msg->data.size() == 2)
    {
        if ((msg->data[0] >= MODEL_DOF) || (msg->data[0] < 0))
        {
            std::cout << "Joint Number Error :::\n Usage : \n 1 : joint number \n 2 : torque(NM) \n 3 : duration(option)" << std::endl;
        }
        else
        {
            std::cout << TOCABI::ELMO_NAME[(int)(msg->data[0])] << " : " << msg->data[1] << " NM for 1 seconds " << std::endl;
            torqueCustomCommand.setZero();
            torqueCustomCommand[(int)(msg->data[0])] = msg->data[1];
            torqueCC_recvt = control_time_real_;
            torqueCC_comt = 1.0;
            torqueCCEnable = true;
        }
    }
    else if (msg->data.size() == 3)
    {
        if ((msg->data[0] >= MODEL_DOF) || (msg->data[0] < 0))
        {
            std::cout << "Joint Number Error :::\n Usage : \n 1 : joint number \n 2 : torque(NM) \n 3 : duration(option)" << std::endl;
        }
        else
        {
            std::cout << TOCABI::ELMO_NAME[(int)(msg->data[0])] << " : " << msg->data[1] << " NM for " << msg->data[2] << " seconds " << std::endl;

            torqueCustomCommand.setZero();
            torqueCustomCommand[(int)(msg->data[0])] = msg->data[1];
            torqueCC_recvt = control_time_real_;
            torqueCC_comt = msg->data[2];
            torqueCCEnable = true;
        }
    }
    else if (msg->data.size() == MODEL_DOF)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            torqueCustomCommand[i] = msg->data[i];
        }
        torqueCCEnable = true;
    }
}