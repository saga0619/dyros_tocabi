#include "tocabi_controller/tocabi_controller.h"
#include "tocabi_controller/terminal.h"
#include "tocabi_controller/TaskCommand.h"
#include <tf/transform_datatypes.h>
#include "stdlib.h"
#include <sstream>
#include <iostream>
#include <string>

std::mutex mtx;
std::mutex mtx_rbdl;
std::mutex mtx_dc;
std::mutex mtx_terminal;
std::mutex mtx_ncurse;

std::atomic<bool> task_recv{false};
std::atomic<bool> taskque_recv{false};

std::chrono::duration<double> td_gs[5];

TocabiController::TocabiController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm) : dc(dc_global), s_(sm), d_(dm), tocabi_(dc_global.tocabi_), wbc_(dc_global.wbc_), mycontroller(*(new CustomController(dc_global, dc_global.tocabi_)))
{
    initialize();
    task_command = dc.nh.subscribe("/tocabi/taskcommand", 100, &TocabiController::TaskCommandCallback, this);
    task_command_que = dc.nh.subscribe("/tocabi/taskquecommand", 100, &TocabiController::TaskQueCommandCallback, this);
    taskgain_sub = dc.nh.subscribe("/tocabi/taskgaincommand", 100, &TocabiController::TaskGainCallback, this);
    vel_command_sub = dc.nh.subscribe("/tocabi/velcommand", 100, &TocabiController::VelocityCommandCallback, this);
    position_command_sub = dc.nh.subscribe("/tocabi/positioncommand", 100, &TocabiController::PositionCommandCallback, this);
}

TocabiController::~TocabiController()
{
    std::cout << "TC Destructor" << std::endl;
}

void TocabiController::GetTaskCommand()
{
    if (task_recv)
    {
        if (dc.semode || dc.use_virtual_for_mujoco)
        {
            tocabi_controller::TaskCommand tc_temp = TaskCont;
            gettaskcommand(tc_temp);
        }
        else
        {
            std::cout << "Warning ::: StateEstimation Off" << std::endl;
        }
        task_recv = false;
    }
    if (taskque_recv)
    {
        if (dc.semode || dc.use_virtual_for_mujoco)
        {
            tque_msg = TaskQueCont;
            std::cout << "Task Que Received, " << tque_msg.tque.size() << " left" << std::endl;
            tocabi_controller::TaskCommand tc_temp = tque_msg.tque[0];
            tque_msg.tque.erase(tque_msg.tque.begin());
            gettaskcommand(tc_temp);
            task_que_switch = true;
        }
        else
        {
            std::cout << "Warning ::: StateEstimation Off" << std::endl;
        }
        taskque_recv = false;
    }
}

void TocabiController::TaskQueCommandCallback(const tocabi_controller::TaskCommandQueConstPtr &msg)
{
    //const tocabi_controller::TaskCommandConstPtr &tc_temp;
    TaskQueCont = *msg;
    taskque_recv = true;
}

void TocabiController::TaskCommandCallback(const tocabi_controller::TaskCommandConstPtr &msg)
{
    TaskCont = *msg;
    task_recv = true;
}

void TocabiController::PositionCommandCallback(const tocabi_controller::positionCommandConstPtr &msg)
{
    dc.positionControl = true;
    dc.set_q_init = true;
    for (int i = 0; i < MODEL_DOF; i++)
        dc.positionDesiredExt(i) = msg->position[i];
    dc.position_command_time = control_time_;
    dc.position_traj_time = msg->traj_time;
    dc.tocabi_.task_control_switch = false;
    dc.position_command_ext = true;
    dc.pext_gravity = msg->gravity;

    if (msg->relative)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            dc.positionDesiredExt(i) = tocabi_.q_(i) + msg->position[i];
        }
    }
}
VectorQd TocabiController::positionCommandExt(double control_time, double traj_time, VectorQd current_pos, VectorQd desired_pos)
{
}

void TocabiController::VelocityCommandCallback(const tocabi_controller::VelocityCommandConstPtr &msg)
{
    vc.link_ = msg->task_link;
    for (int i = 0; i < 6; i++)
    {
        vc.des_vel[i] = msg->des_vel[i];
    }

    vc.used_ = false;
}

void TocabiController::TaskGainCallback(const tocabi_controller::TaskGainCommandConstPtr &msg)
{
    int link_id = 99;
    int link_id2 = 99;
    if (msg->mode == 0)
    {
        //Set every Gain to Zero
        double kp = 400;
        double kd = 40;
        double akp = 400;
        double akd = 40;

        if (dc.mode == "realrobot")
        {
            kp = 60;
            kd = 4;
            akp = 80;
            akd = 5;
        }

        link_id = COM_id;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }
        link_id = Pelvis;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }
        link_id = Upper_Body;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }
        link_id = Left_Foot;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }
        link_id = Left_Hand;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }
        link_id = Right_Hand;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }
        link_id = Right_Foot;
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = kp;
            dc.tocabi_.link_[link_id].pos_d_gain(i) = kd;
            dc.tocabi_.link_[link_id].rot_p_gain(i) = akp;
            dc.tocabi_.link_[link_id].rot_d_gain(i) = akd;
        }

        std::cout << "Reset Task Gain" << std::endl;
    }
    else if (msg->mode == 1)
    {
        link_id = COM_id;
        std::cout << "Set COM Task Gain" << std::endl;
    }
    else if (msg->mode == 2)
    {
        link_id = Pelvis;
        std::cout << "Set Pelvis Task Gain" << std::endl;
    }
    else if (msg->mode == 3)
    {
        link_id = Upper_Body;
        std::cout << "Set Upper_Body Task Gain" << std::endl;
    }
    else if (msg->mode == 4)
    {
        link_id = Right_Foot;
        link_id2 = Left_Foot;
        //both foot
        std::cout << "Set both foot Task Gain" << std::endl;
    }
    else if (msg->mode == 5)
    {
        link_id = Right_Hand;
        link_id2 = Left_Hand;
        //both hand
        std::cout << "Set both hand Task Gain" << std::endl;
    }
    else if (msg->mode == 6)
    {
        //R foot
        link_id = Right_Foot;
        std::cout << "Set Right_Foot Task Gain" << std::endl;
    }
    else if (msg->mode == 7)
    {
        //L foot
        link_id = Left_Foot;
        std::cout << "Set Left_Foot Task Gain" << std::endl;
    }
    else if (msg->mode == 8)
    {
        //R hand
        link_id = Right_Hand;
        std::cout << "Set Right_Hand Task Gain" << std::endl;
    }
    else if (msg->mode == 9)
    {
        //L hand
        link_id = Left_Hand;
        std::cout << "Set Left_Hand Task Gain" << std::endl;
    }

    if (link_id != 99)
    {
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id].pos_p_gain(i) = msg->pgain[i];
            dc.tocabi_.link_[link_id].pos_d_gain(i) = msg->dgain[i];
            dc.tocabi_.link_[link_id].rot_p_gain(i) = msg->pgain[i + 3];
            dc.tocabi_.link_[link_id].rot_d_gain(i) = msg->dgain[i + 3];
        }

        if (msg->mode != 0)
        {

            std::cout << "X kp : " << dc.tocabi_.link_[link_id].pos_p_gain[0] << " kd : " << dc.tocabi_.link_[link_id].pos_d_gain[0]
                      << "  Y kp : " << dc.tocabi_.link_[link_id].pos_p_gain[1] << " kd : " << dc.tocabi_.link_[link_id].pos_d_gain[1]
                      << "  Z kp : " << dc.tocabi_.link_[link_id].pos_p_gain[2] << " kd : " << dc.tocabi_.link_[link_id].pos_d_gain[2] << std::endl;
            std::cout << "R kp : " << dc.tocabi_.link_[link_id].rot_p_gain[0] << " kd : " << dc.tocabi_.link_[link_id].rot_d_gain[0]
                      << "  P kp : " << dc.tocabi_.link_[link_id].rot_p_gain[1] << " kd : " << dc.tocabi_.link_[link_id].rot_d_gain[1]
                      << "  Y kp : " << dc.tocabi_.link_[link_id].rot_p_gain[2] << " kd : " << dc.tocabi_.link_[link_id].rot_d_gain[2] << std::endl;
        }
    }

    if (link_id2 != 99)
    {
        for (int i = 0; i < 3; i++)
        {
            dc.tocabi_.link_[link_id2].pos_p_gain(i) = msg->pgain[i];
            dc.tocabi_.link_[link_id2].pos_d_gain(i) = msg->dgain[i];
            dc.tocabi_.link_[link_id2].rot_p_gain(i) = msg->pgain[i + 3];
            dc.tocabi_.link_[link_id2].rot_d_gain(i) = msg->dgain[i + 3];
        }
    }

    tocabi_.task_control_switch = false;
}

void TocabiController::gettaskcommand(tocabi_controller::TaskCommand &msg)
{
    dc.tc_state = 0;
    tocabi_.control_time_pre_ = control_time_;

    tc.command_time = control_time_;
    tc.traj_time = msg.time;
    tc.mode = msg.mode;
    tc.task_init = true;

    tc.pelv_pitch = msg.pelv_pitch;
    tc.ratio = msg.ratio;
    tc.roll = msg.roll;
    tc.pitch = msg.pitch;
    tc.yaw = msg.yaw;
    tc.height = msg.height;

    tc.contactredis = msg.contactredis;
    tocabi_.contact_redistribution_mode = tc.contactredis;
    tc.solver = msg.solver;
    tocabi_.init_qp = true;
    tc.custom_taskgain = msg.customTaskGain;
    tc.pos_p = msg.pos_p;
    tc.pos_d = msg.pos_d;
    tc.ang_p = msg.ang_p;
    tc.ang_d = msg.ang_d;
    tc.acc_p = msg.acc_p;

    tc.l_x = msg.l_x;
    tc.l_y = msg.l_y;
    tc.l_z = msg.l_z;
    tc.l_roll = msg.l_roll * DEG2RAD;
    tc.l_pitch = msg.l_pitch * DEG2RAD;
    tc.l_yaw = msg.l_yaw * DEG2RAD;

    tc.r_x = msg.r_x;
    tc.r_y = msg.r_y;
    tc.r_z = msg.r_z;
    tc.r_roll = msg.r_roll * DEG2RAD;
    tc.r_pitch = msg.r_pitch * DEG2RAD;
    tc.r_yaw = msg.r_yaw * DEG2RAD;

    tocabi_.link_[Right_Foot].Set_initpos();
    tocabi_.link_[Left_Foot].Set_initpos();
    tocabi_.link_[Right_Hand].Set_initpos();
    tocabi_.link_[Left_Hand].Set_initpos();
    tocabi_.link_[Pelvis].Set_initpos();
    tocabi_.link_[Upper_Body].Set_initpos();
    tocabi_.link_[COM_id].Set_initpos();

    tc.maintain_last_control = msg.maintain_lc;

    if (tc.maintain_last_control)
    {
        tocabi_.link_[COM_id].x_init = tocabi_.link_[COM_id].x_desired;
        tocabi_.link_[Pelvis].x_init = tocabi_.link_[Pelvis].x_desired;
        tocabi_.link_[Upper_Body].x_init = tocabi_.link_[Upper_Body].x_desired;
        tocabi_.link_[Pelvis].rot_init = tocabi_.link_[Pelvis].rot_desired;
        tocabi_.link_[Upper_Body].rot_init = tocabi_.link_[Upper_Body].rot_desired;
    }

    if (false)
    {
        std::cout << "Set init by TaskCommand" << std::endl;
        tocabi_.link_[Right_Foot].Set_initTask();
        tocabi_.link_[Left_Foot].Set_initTask();
        tocabi_.link_[Right_Hand].Set_initTask();
        tocabi_.link_[Left_Hand].Set_initTask();
        tocabi_.link_[Pelvis].Set_initTask();
        tocabi_.link_[Upper_Body].Set_initTask();
        tocabi_.link_[COM_id].Set_initTask();
    }

    tc.init_com_height = tocabi_.com_.pos(2);

    tocabi_.task_control_switch = true;
    tc_command = true;

    // Arm Desired Setting
    Eigen::Vector3d TargetDelta_l, TargetDelta_r;
    TargetDelta_l << tc.l_x, tc.l_y, tc.l_z;
    TargetDelta_r << tc.r_x, tc.r_y, tc.r_z;

    tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init + TargetDelta_l;
    tocabi_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(tc.l_roll) * DyrosMath::rotateWithY(tc.l_pitch) * DyrosMath::rotateWithZ(tc.l_yaw) * tocabi_.link_[Left_Hand].rot_init;
    tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Right_Hand].x_init + TargetDelta_r;
    tocabi_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(tc.r_roll) * DyrosMath::rotateWithY(tc.r_pitch) * DyrosMath::rotateWithZ(tc.r_yaw) * tocabi_.link_[Right_Hand].rot_init;
    tocabi_.imu_pos_.setZero();

    std::cout << control_time_ << "init set - COM x : " << tocabi_.link_[COM_id].x_init(0) << "\t y : " << tocabi_.link_[COM_id].x_init(1) << std::endl;

    //walking
    tc.walking_enable = msg.walking_enable;
    tc.ik_mode = msg.ik_mode;
    tc.walking_pattern = msg.pattern;
    tc.walking_pattern2 = msg.pattern2;
    tc.vibration_control = msg.comcontrol;
    tc.foot_step_dir = msg.first_foot_step;
    tc.target_x = msg.x;
    tc.target_y = msg.y;
    tc.target_z = msg.z;
    tc.walking_height = msg.height;
    tc.theta = msg.theta;
    tc.step_length_y = msg.step_length_y;
    tc.step_length_x = msg.step_length_x;
    tc.dob = msg.dob;
    tc.mom = msg.mom;
    tc.imu_walk = msg.imu;
    tc.walking_enable = msg.walking_enable;

    if (tc.walking_enable == 1.0 || tc.walking_enable == 3.0)
    {
        tc.mode = 11;
    }

    tc.link_contact[0] = msg.left_foot;
    tc.link_contact[1] = msg.right_foot;
    tc.link_contact[2] = msg.left_hand;
    tc.link_contact[3] = msg.right_hand;

    if (tc.mode < 10)
    {
        if (dc.positionControl)
        {
            dc.positionControl = false;
            dc.position_command_ext = false;
            std::cout << "PositionControl Disabled By Task Command" << std::endl;
        }
    }

    if (dc.print_data_ready)
    {
        dc.data_out << "###############  COMMAND RECEIVED  ###############" << control_time_ << std::endl;
    }
}

void TocabiController::stateThread()
{
    s_.connect();
    s_.stateThread();
}

void TocabiController::dynamicsThreadHigh()
{
    std::cout << "Dynamics High Thread : READY ?" << std::endl;
    while ((!dc.connected) && (!dc.firstcalcdone) && (!shutdown_tocabi_bool))
    {
        //wait for realrobot thread start
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    std::chrono::microseconds cycletime(dc.ctime);
    int cycle_count = 0;

    s_.SetPositionPDGainMatrix();

    if (!shutdown_tocabi_bool)
    {
        std::cout << "Dynamics High Thread : START" << std::endl;
        if (dc.mode != "realrobot")
        {
            dc.start_time_point = std::chrono::steady_clock::now();
        }

        while (!shutdown_tocabi_bool)
        {
            //std::cout<<"t : "<<control_time_<<std::flush;
            std::this_thread::sleep_until(dc.start_time_point + (cycle_count * cycletime));
            cycle_count++;

            if (dc.positionControl)
            {
                if (dc.set_q_init)
                {
                    tocabi_.q_desired_ = tocabi_.q_;
                    tocabi_.q_init_ = tocabi_.q_;
                    dc.set_q_init = false;
                }
                if (dc.positionGravControl)
                {
                    if (dc.positionDobControl)
                    {
                        for (int i = 0; i < MODEL_DOF; i++)
                        {
                            torque_desired(i) = dc.torque_dist(i) + tocabi_.torque_grav_cc(i) + dc.tocabi_.Kps[i] * (tocabi_.q_desired_(i) - tocabi_.q_(i)) - dc.tocabi_.Kvs[i] * (tocabi_.q_dot_virtual_(i + 6));
                        }
                    }
                    else
                    {
                        for (int i = 0; i < MODEL_DOF; i++)
                        {
                            torque_desired(i) = tocabi_.torque_grav_cc(i) + dc.tocabi_.Kps[i] * (tocabi_.q_desired_(i) - tocabi_.q_(i)) - dc.tocabi_.Kvs[i] * (tocabi_.q_dot_virtual_(i + 6));
                        }
                    }
                }
                else if (dc.position_command_ext)
                {
                    //torqueOn delay
                    if (dc.torqueOn && dc.torqueOnTime > 3.0)
                    {
                    }

                    for (int i = 0; i < MODEL_DOF; i++)
                    {
                        if (dc.pext_gravity)
                        {
                            torque_desired(i) = torque_grav(i) + dc.tocabi_.Kps[i] * (DyrosMath::cubic(control_time_, dc.position_command_time, dc.position_command_time + dc.position_traj_time, tocabi_.q_init_(i), dc.positionDesiredExt(i), 0, 0) - tocabi_.q_(i)) +
                                                dc.tocabi_.Kvs[i] * (DyrosMath::cubicDot(control_time_, dc.position_command_time, dc.position_command_time + dc.position_traj_time, tocabi_.q_init_(i), dc.positionDesiredExt(i), 0, 0, 2000) - tocabi_.q_dot_virtual_(i + 6));
                        }
                        else
                        {
                            torque_desired(i) = dc.tocabi_.Kps[i] * (DyrosMath::cubic(control_time_, dc.position_command_time, dc.position_command_time + dc.position_traj_time, tocabi_.q_init_(i), dc.positionDesiredExt(i), 0, 0) - tocabi_.q_(i)) +
                                                dc.tocabi_.Kvs[i] * (DyrosMath::cubicDot(control_time_, dc.position_command_time, dc.position_command_time + dc.position_traj_time, tocabi_.q_init_(i), dc.positionDesiredExt(i), 0, 0, 2000) - tocabi_.q_dot_virtual_(i + 6));
                        }
                    }
                }
                else
                {
                    if (dc.positionDobControl)
                    {
                        for (int i = 0; i < MODEL_DOF; i++)
                        {
                            torque_desired(i) = dc.torque_dist(i) + dc.tocabi_.Kps[i] * (tocabi_.q_desired_(i) - tocabi_.q_(i)) - dc.tocabi_.Kvs[i] * (tocabi_.q_dot_virtual_(i + 6));
                        }
                    }
                    else
                    {
                        for (int i = 0; i < MODEL_DOF; i++)
                        {
                            torque_desired(i) = dc.tocabi_.Kps[i] * (tocabi_.q_desired_(i) - tocabi_.q_(i)) - dc.tocabi_.Kvs[i] * (tocabi_.q_dot_virtual_(i + 6));
                        }
                    }
                }

                if (tocabi_.task_control_switch)
                {
                    if (tc.mode >= 10)
                    {
                        mycontroller.computeFast();
                    }
                }
            }
            else
            {
                if (tocabi_.task_control_switch)
                {
                    if (tc.mode == 2 || tc.mode == 3)
                    {
                        while (!dc.trigger_hqp)
                        {
                            std::this_thread::sleep_for(std::chrono::microseconds(1));

                            if (shutdown_tocabi_bool || !tocabi_.task_control_switch)
                                break;
                        }
                        if (dc.trigger_hqp)
                        {
                            dc.trigger_hqp = false;
                            tocabi_fast_.torque_control = wbc_.task_control_torque_hqp_threaded(tocabi_fast_, tocabi_.init_qp);
                            //std::cout<<"calc end"<<std::endl;
                        }

                        torque_desired = tocabi_fast_.torque_control;
                    }
                    else if (tc.mode >= 10)
                    {
                        mycontroller.computeFast();
                        torque_desired = mycontroller.getControl();
                    }
                }
            }

            mtx.lock();
            dc.torque_desired = torque_desired;
            s_.sendCommand(torque_desired, sim_time);
            mtx.unlock();
            dynthread2_cnt++;
        }
    }
    std::cout << cyellow << "Dynamics High Thread : End !" << creset << std::endl;
}

void TocabiController::testThread()
{
    std::cout << "TC test thread " << std::endl;
    wbc_.init(tocabi_);
    dc.testmode = true;
    int cnt = 0;
    while ((!shutdown_tocabi_bool))
    {
        cnt++;
        getState();

        wbc_.set_contact(tocabi_, 1, 1);
        std::chrono::steady_clock::time_point tp1 = std::chrono::steady_clock::now();

        std::cout << "//////////////////////////////////////" << std::endl;
        std::cout << "Original Matrix : " << std::endl;
        std::cout << tocabi_.W << std::endl
                  << std::endl;
        DyrosMath::pinv_SVD(tocabi_.W);

        std::chrono::duration<double> d1 = std::chrono::steady_clock::now() - tp1;
        std::chrono::steady_clock::time_point tp2 = std::chrono::steady_clock::now();
        DyrosMath::pinv_glsSVD(tocabi_.W);

        std::chrono::duration<double> d2 = std::chrono::steady_clock::now() - tp2;

        std::cout << "##### eigen pinv" << d1.count() * 1000000 << "#### gls svd" << d2.count() * 1000000 << std::endl;

        if (cnt > 10)
        {
            break;
        }
    }

    std::cout << "TC tes thread end " << std::endl;
}

void TocabiController::dynamicsThreadLow()
{
    std::cout << "Dynamics Low Thread : READY ?" << std::endl;
    ros::Rate r(2000);
    int calc_count = 0;
    int ThreadCount = 0;
    int i = 1;

    while ((!dc.connected) && (!shutdown_tocabi_bool))
    {
        r.sleep();
    }
    while ((!dc.firstcalcdone) && (!shutdown_tocabi_bool))
    {
        r.sleep();
    }

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::seconds sec1(1);

    bool display = false;
    std::chrono::steady_clock::time_point start_time2 = std::chrono::steady_clock::now();

    int contact_number = 2;
    int link_id[contact_number];
    int total_dof_ = MODEL_DOF;
    Eigen::MatrixXd Lambda_c, J_C_INV_T, N_C, I37, Slc_k, Slc_k_T, W, W_inv;
    Eigen::MatrixXd P_c;
    Eigen::VectorXd Grav_ref;
    Eigen::VectorXd G;
    Eigen::MatrixXd J_g;
    Eigen::MatrixXd aa;
    Eigen::VectorQd torque_task, torque_add;
    Eigen::MatrixXd ppinv;
    Eigen::MatrixXd tg_temp;
    Eigen::MatrixXd A_matrix_inverse;
    Eigen::MatrixXd J_C;
    Eigen::MatrixXd J_C_temp;
    Eigen::MatrixXd Jcon[2];
    Eigen::VectorXd qtemp[2], conp[2];

    Eigen::MatrixXd J_task;
    Eigen::VectorXd f_star;

    int task_number;

    J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
    N_C.setZero(total_dof_ + 6, MODEL_DOF_VIRTUAL);
    bool first = true;

    Eigen::Vector3d task_desired;

    acceleration_estimated_before.setZero();
    q_dot_before_.setZero();

    VectorQd TorqueDesiredLocal, TorqueContact;
    TorqueDesiredLocal.setZero();
    TorqueContact.setZero();

    Vector12d fc_redis;
    double fc_ratio;
    fc_redis.setZero();

    double kpsim, kdsim, kpasim, kdasim;

    kpsim = 400;
    kdsim = 40;
    kpasim = 400;
    kdasim = 40;
    if (dc.mode == "realrobot")
    {
        kpsim = 60;
        kdsim = 4;
        kpasim = 80;
        kdasim = 5;
    }

    Vector3d kp_, kd_, kpa_, kda_, accp_;
    for (int i = 0; i < 3; i++)
    {
        kp_(i) = kpsim;
        kd_(i) = kdsim;
        kpa_(i) = kpasim;
        kda_(i) = kdasim;
        accp_(i) = 1.0;
    }
    //kd_(1) = 120;

    std::cout << "Dynamics Low Thread : START" << std::endl;
    int dynthread_cnt = 0;

    //const char *file_name = "/home/saga/sim_data.txt";

    std::string path = dc.homedir;
    std::string current_time;

    time_t now = std::time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    current_time = buf;
    dc.print_file_name = path + "/tocabi_control_data" + current_time + ".txt";
    dc.print_file_name2 = path + "/tocabi_control_data2" + current_time + ".txt";

    wbc_.print_file_name = path + "/tocabi_wbc_slack_data" + current_time + ".txt";
    wbc_.print_file_name2 = path + "/tocabi_wbc_hqp_time_data" + current_time + ".txt";
    std::stringstream ss;

    ///////////////////////
    wbc_.init(tocabi_);
    tocabi_.TaskForce.setZero(6);

    std::chrono::steady_clock::time_point tp[6];
    std::chrono::duration<double> td[10];

    dc.nh.getParam("/tocabi_controller/TaskGainPCOM", dc.tocabi_.v_com_kp);
    dc.nh.getParam("/tocabi_controller/TaskGainDCOM", dc.tocabi_.v_com_kv);

    dc.nh.getParam("/tocabi_controller/TaskGainPPelvis", dc.tocabi_.v_pelv_kp);
    dc.nh.getParam("/tocabi_controller/TaskGainDPelvis", dc.tocabi_.v_pelv_kv);

    dc.nh.getParam("/tocabi_controller/TaskGainPUpperbody", dc.tocabi_.v_up_kp);
    dc.nh.getParam("/tocabi_controller/TaskGainDUpperbody", dc.tocabi_.v_up_kv);

    dc.nh.getParam("/tocabi_controller/TaskGainPHand", dc.tocabi_.v_hand_kp);
    dc.nh.getParam("/tocabi_controller/TaskGainDHand", dc.tocabi_.v_hand_kv);

    dc.nh.getParam("/tocabi_controller/TaskGainPFoot", dc.tocabi_.v_foot_kp);
    if (dc.nh.getParam("/tocabi_controller/TaskGainDFoot", dc.tocabi_.v_foot_kv))
    {
        for (int i = 0; i < 3; i++)
        {
            tocabi_.link_[COM_id].pos_p_gain[i] = dc.tocabi_.v_com_kp[i];
            tocabi_.link_[COM_id].pos_d_gain[i] = dc.tocabi_.v_com_kv[i];
            tocabi_.link_[COM_id].rot_p_gain[i] = dc.tocabi_.v_com_kp[i + 3];
            tocabi_.link_[COM_id].rot_d_gain[i] = dc.tocabi_.v_com_kv[i + 3];

            tocabi_.link_[Pelvis].pos_p_gain[i] = dc.tocabi_.v_pelv_kp[i];
            tocabi_.link_[Pelvis].pos_d_gain[i] = dc.tocabi_.v_pelv_kv[i];
            tocabi_.link_[Pelvis].rot_p_gain[i] = dc.tocabi_.v_pelv_kp[i + 3];
            tocabi_.link_[Pelvis].rot_d_gain[i] = dc.tocabi_.v_pelv_kv[i + 3];

            tocabi_.link_[Upper_Body].rot_p_gain[i] = dc.tocabi_.v_up_kp[i];
            tocabi_.link_[Upper_Body].rot_d_gain[i] = dc.tocabi_.v_up_kv[i];

            tocabi_.link_[Right_Foot].pos_p_gain[i] = tocabi_.link_[Left_Foot].pos_p_gain[i] = dc.tocabi_.v_foot_kp[i];
            tocabi_.link_[Right_Foot].pos_d_gain[i] = tocabi_.link_[Left_Foot].pos_d_gain[i] = dc.tocabi_.v_foot_kv[i];
            tocabi_.link_[Right_Foot].rot_p_gain[i] = tocabi_.link_[Left_Foot].rot_p_gain[i] = dc.tocabi_.v_foot_kp[i + 3];
            tocabi_.link_[Right_Foot].rot_d_gain[i] = tocabi_.link_[Left_Foot].rot_d_gain[i] = dc.tocabi_.v_foot_kv[i + 3];

            tocabi_.link_[Right_Hand].pos_p_gain[i] = tocabi_.link_[Left_Hand].pos_p_gain[i] = dc.tocabi_.v_hand_kp[i];
            tocabi_.link_[Right_Hand].pos_d_gain[i] = tocabi_.link_[Left_Hand].pos_d_gain[i] = dc.tocabi_.v_hand_kv[i];
            tocabi_.link_[Right_Hand].rot_p_gain[i] = tocabi_.link_[Left_Hand].rot_p_gain[i] = dc.tocabi_.v_hand_kp[i + 3];
            tocabi_.link_[Right_Hand].rot_d_gain[i] = tocabi_.link_[Left_Hand].rot_d_gain[i] = dc.tocabi_.v_hand_kv[i + 3];
        }
    }
    else
    {
        //Task link gain setting.
        tocabi_.link_[COM_id].pos_p_gain = kp_;
        tocabi_.link_[COM_id].pos_d_gain = kd_;
        tocabi_.link_[COM_id].rot_p_gain = tocabi_.link_[Pelvis].rot_p_gain = kpa_;
        tocabi_.link_[COM_id].rot_d_gain = tocabi_.link_[Pelvis].rot_d_gain = kda_;
        tocabi_.link_[Pelvis].pos_p_gain = kp_;
        tocabi_.link_[Pelvis].pos_d_gain = kd_;

        tocabi_.link_[Right_Foot].pos_p_gain = tocabi_.link_[Left_Foot].pos_p_gain = kp_;
        tocabi_.link_[Right_Foot].pos_d_gain = tocabi_.link_[Left_Foot].pos_d_gain = kd_;
        tocabi_.link_[Right_Foot].rot_p_gain = tocabi_.link_[Left_Foot].rot_p_gain = kpa_;
        tocabi_.link_[Right_Foot].rot_d_gain = tocabi_.link_[Left_Foot].rot_d_gain = kda_;

        tocabi_.link_[Right_Hand].pos_p_gain = tocabi_.link_[Left_Hand].pos_p_gain = kp_;
        tocabi_.link_[Right_Hand].pos_d_gain = tocabi_.link_[Left_Hand].pos_d_gain = kd_;
        tocabi_.link_[Right_Hand].rot_p_gain = tocabi_.link_[Left_Hand].rot_p_gain = kpa_ * 4;
        tocabi_.link_[Right_Hand].rot_d_gain = tocabi_.link_[Left_Hand].rot_d_gain = kda_ * 2;

        tocabi_.link_[Upper_Body].rot_p_gain = kpa_ * 4;
        tocabi_.link_[Upper_Body].rot_d_gain = kda_ * 2;

        tocabi_.link_[COM_id].pos_p_gain = kp_;
        tocabi_.link_[COM_id].pos_d_gain = kd_;

        tocabi_.link_[COM_id].acc_p_gain = accp_;
        tocabi_.link_[Pelvis].acc_p_gain = accp_;
        tocabi_.link_[Right_Foot].acc_p_gain = accp_;
        tocabi_.link_[Left_Foot].acc_p_gain = accp_;
        tocabi_.link_[Right_Hand].acc_p_gain = accp_;
        tocabi_.link_[Left_Hand].acc_p_gain = accp_;
    }

    double td_avg[12];
    for (int i = 0; i < 9; i++)
        td_avg[i] = 0.0;
    int td_cnt = 0;
    //Control Loop Start
    while ((!shutdown_tocabi_bool))
    {
        static double est;
        std::chrono::steady_clock::time_point dyn_loop_start = std::chrono::steady_clock::now();

        dynthread_cnt++;

        loop_pass = true;
        while (loop_pass && (!shutdown_tocabi_bool))
        {
            getState();
        }

        if (control_time_ == 0)
        {
            first = true;
            tocabi_.task_control_switch = false;
            tocabi_.ee_[0].contact = true;
            tocabi_.ee_[1].contact = true;
        }
        if ((dyn_loop_start - start_time2) > sec1)
        {
            start_time2 = std::chrono::steady_clock::now();
            if (dc.checkfreqency)
            {
                ss.str("");
                static int stac = 0;
                ss << "dyn : " << dynthread_cnt << " hz, dyn2 : " << dynthread2_cnt << " hz, stn : " << dc.sta_cnt - stac << " hz, time : " << est; //<< std::endl;
                dynthread2_cnt = 0;
                stac = dc.sta_cnt;
                //dc.statusPubMsg.data = ss.str();
                pub_to_gui(dc, ss.str().c_str());
            }
            if (dc.print_delay_info)
            {
                // std::cout << "td1 : " << td[0].count() * 1000 << "  td2 : " << td[1].count() * 1000 << "  td3 : " << td[2].count() * 1000 << "  td4 : " << td[3].count() * 1000 << "  td5 : " << td[4].count() * 1000 << "  td6 : " << td[5].count() * 1000 << std::endl;
            }
            dynthread_cnt = 0;
            est = 0;
        }
        GetTaskCommand();
        
        wbc_.update(tocabi_);
        
        if (control_time_ == 0)
        {
            std::cout << "Gain ?" << std::endl;

            tocabi_.link_[COM_id].pos_p_gain = kp_;
            tocabi_.link_[COM_id].pos_d_gain = kd_;
            tocabi_.link_[COM_id].rot_p_gain = tocabi_.link_[Pelvis].rot_p_gain = kpa_;
            tocabi_.link_[COM_id].rot_d_gain = tocabi_.link_[Pelvis].rot_d_gain = kda_;
            tocabi_.link_[Pelvis].pos_p_gain = kp_;
            tocabi_.link_[Pelvis].pos_d_gain = kd_;

            tocabi_.link_[Right_Foot].pos_p_gain = tocabi_.link_[Left_Foot].pos_p_gain = kp_;
            tocabi_.link_[Right_Foot].pos_d_gain = tocabi_.link_[Left_Foot].pos_d_gain = kd_;
            tocabi_.link_[Right_Foot].rot_p_gain = tocabi_.link_[Left_Foot].rot_p_gain = kpa_;
            tocabi_.link_[Right_Foot].rot_d_gain = tocabi_.link_[Left_Foot].rot_d_gain = kda_;

            tocabi_.link_[Right_Hand].pos_p_gain = tocabi_.link_[Left_Hand].pos_p_gain = kp_;
            tocabi_.link_[Right_Hand].pos_d_gain = tocabi_.link_[Left_Hand].pos_d_gain = kd_;
            tocabi_.link_[Right_Hand].rot_p_gain = tocabi_.link_[Left_Hand].rot_p_gain = kpa_ * 4;
            tocabi_.link_[Right_Hand].rot_d_gain = tocabi_.link_[Left_Hand].rot_d_gain = kda_ * 2;

            tocabi_.link_[Upper_Body].rot_p_gain = kpa_ * 4;
            tocabi_.link_[Upper_Body].rot_d_gain = kda_ * 2;

            tocabi_.link_[COM_id].pos_p_gain = kp_;
            tocabi_.link_[COM_id].pos_d_gain = kd_;

            tocabi_.link_[COM_id].acc_p_gain = accp_;
            tocabi_.link_[Pelvis].acc_p_gain = accp_;
            tocabi_.link_[Right_Foot].acc_p_gain = accp_;
            tocabi_.link_[Left_Foot].acc_p_gain = accp_;
            tocabi_.link_[Right_Hand].acc_p_gain = accp_;
            tocabi_.link_[Left_Hand].acc_p_gain = accp_;
        }
        if (tc.custom_taskgain)
            customgainhandle();

        ///////////////////////////////////////////////////////////////////////////////////////
        /////////////              Controller Code Here !                     /////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        torque_task.setZero(MODEL_DOF);
        torque_add.setZero();
        TorqueContact.setZero();

        if (dc.signal_gravityCompensation)
        {
            if (dc.positionControl)
            {
                std::cout << "Position Control Disabled!  ";
            }

            if (tocabi_.task_control_switch)
            {
                std::cout << "Task Switch Disabled!  ";
            }
            std::cout << "Gravity compensation only !" << std::endl;
            tocabi_.task_control_switch = false;
            dc.positionControl = false;
            dc.position_command_ext = false;
            dc.signal_gravityCompensation = false;
        }

        if (tocabi_.task_control_switch)
        {
            if (task_que_switch)
            {
                if (control_time_ > (tc.command_time + tc.traj_time))
                {
                    std::cout << "current ct : " << control_time_ << " cmd t : " << tc.command_time << " trj t: " << tc.traj_time << std::endl;
                    if (tque_msg.tque.size() > 0)
                    {

                        tocabi_controller::TaskCommand tc_temp = tque_msg.tque[0];
                        tque_msg.tque.erase(tque_msg.tque.begin());
                        gettaskcommand(tc_temp);
                    }
                    else
                    {
                        task_que_switch = false;
                        std::cout << "task que complete !" << std::endl;
                    }
                }
            }

            if (tc.mode == 0) //COM position control
            {
                /* 
                For Task Control, NEVER USE tocabi_
                controller.cpp.
                Use dyros_cc, CustomController for task control. 
                */
                wbc_.set_contact(tocabi_, 1, 1);

                int task_number = 6;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task = tocabi_.link_[COM_id].Jac;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star = wbc_.getfstar6d(tocabi_, COM_id);
                //tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));
                //torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);

                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
                tocabi_.contact_redistribution_mode = 2;
                torque_grav.setZero();
            }
            else if (tc.mode == 1) //COM with rotation
            {
                //tocabi_.d_time_
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. */
                int task_number = 21;

                std::vector<MatrixXd> Jtask_hqp;
                std::vector<VectorXd> fstar_hqp;

                Jtask_hqp.resize(3);
                fstar_hqp.resize(3);

                Eigen::MatrixXd Jtask_hand(12, MODEL_DOF_VIRTUAL);
                Jtask_hqp[0] = tocabi_.link_[COM_id].Jac;

                Jtask_hqp[1].resize(12, MODEL_DOF_VIRTUAL);
                Jtask_hqp[1].block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;
                Jtask_hqp[1].block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM;

                Jtask_hqp[2] = tocabi_.link_[Upper_Body].Jac_COM_r;

                /* tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                tocabi_.J_task.block(9, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_p;
                tocabi_.J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_r;
                tocabi_.J_task.block(15, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM_p;
                tocabi_.J_task.block(18, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM_r;*/

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].x_init + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].x_init;
                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[COM_id].rot_desired = DyrosMath::rotateWithY(tc.pelv_pitch * 3.1415 / 180.0);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0); //Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                //tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init;
                /*tocabi_.f_star.segment(9, 3) = tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].v + tocabi_.link_[Pelvis].v);
                tocabi_.f_star.segment(12, 3) = tocabi_.link_[Left_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].w + tocabi_.link_[Pelvis].w);
                tocabi_.f_star.segment(15, 3) = tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].v + tocabi_.link_[Pelvis].v);
                tocabi_.f_star.segment(18, 3) = tocabi_.link_[Right_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].w + tocabi_.link_[Pelvis].w);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);*/

                Eigen::Vector3d left_hand_local(tc.l_x + 0.3, tc.l_y + 0.3, tc.l_z);
                Eigen::Vector3d right_hand_local(tc.r_x + 0.3, tc.r_y - 0.3, tc.r_z);

                //tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * left_hand_local;
                //tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * right_hand_local;

                tocabi_.link_[Left_Hand].x_init_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Upper_Body].x_init);
                tocabi_.link_[Right_Hand].x_init_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Upper_Body].x_init);

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Left_Hand].x_init_local, left_hand_local);
                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Right_Hand].x_init_local, right_hand_local);

                Eigen::Vector3d l_pos_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Left_Hand].xpos - tocabi_.link_[Upper_Body].xpos);
                Eigen::Vector3d r_pos_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Right_Hand].xpos - tocabi_.link_[Upper_Body].xpos);

                Eigen::Vector3d l_v_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Left_Hand].v - tocabi_.link_[Upper_Body].v);
                Eigen::Vector3d r_v_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Right_Hand].v - tocabi_.link_[Upper_Body].v);

                fstar_hqp[0] = wbc_.getfstar6d(tocabi_, COM_id);
                fstar_hqp[1].resize(12);
                //fstar_hqp[1].segment(0, 3) = tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].v + tocabi_.link_[Pelvis].v);
                fstar_hqp[1].segment(0, 3) = tocabi_.link_[Left_Hand].pos_p_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Left_Hand].x_traj - l_pos_local)) + tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Left_Hand].v_traj - l_v_local));
                fstar_hqp[1].segment(3, 3) = tocabi_.link_[Left_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].w + tocabi_.link_[Upper_Body].w);
                //fstar_hqp[1].segment(6, 3) = tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].v + tocabi_.link_[Pelvis].v);
                fstar_hqp[1].segment(6, 3) = tocabi_.link_[Right_Hand].pos_p_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Right_Hand].x_traj - r_pos_local)) + tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Right_Hand].v_traj - r_v_local));
                fstar_hqp[1].segment(9, 3) = tocabi_.link_[Right_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].w + tocabi_.link_[Upper_Body].w);

                // tocabi_.link_[Left_Hand].x_init_local = tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Pelvis].x_init;
                //tocabi_.link_[Right_Hand].x_init_local = tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Pelvis].x_init;

                //tocabi_.link_[Left_Hand].

                //fstar_hqp[1].segment(0,3) = tocabi_.link_[Left_Hand].pos_p_gain.cwiseProduct()
                //fstar_hqp[1].block(0,6) = wbc_.getfs
                fstar_hqp[2] = wbc_.getfstar_rot(tocabi_, Upper_Body);

                Jtask_hqp.push_back(tocabi_.Slc_k);

                fstar_hqp.push_back(-0.1 * tocabi_.q_dot_);

                //tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));

                //(tocabi_.lambda * tocabi_.f_star)

                //torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver); // + wbc_.contact_torque_calc_from_QP(tocabi_, torque_grav);
                //wbc_.calc_winv(tocabi_);
                //wbc_.set_contact(tocabi_, 1, 1);

                wbc_.set_contact(tocabi_, 1, 1);

                wbc_.gravity_compensation_torque(tocabi_);

                //wbc_.copy_robot_fast(tocabi_, tocabi_fast_, Jtask_hqp, fstar_hqp);

                //torque_task = wbc_.task_control_torque_hqp_threaded(tocabi_fast_, tocabi_.init_qp);

                torque_task = wbc_.task_control_torque_hqp2(tocabi_, Jtask_hqp, fstar_hqp);

                //torque_task = wbc_.task_control_torque_hqp(tocabi_, Jtask_hqp, fstar_hqp);
                torque_grav.setZero();
                tocabi_.contact_redistribution_mode = 2;

                //wbc_.contact_torque_calc_from_QP(tocabi_, torque_task);

                // std::cout << "ContactForce Without : " << std::endl
                //          << (wbc_.get_contact_force(tocabi_, torque_task)).transpose() << std::endl;
            }
            else if (tc.mode == 2) //COM pos&rot control + upper rotation with jhpark
            {

                //tocabi_.d_time_
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. */
                int task_number = 21;

                std::vector<MatrixXd> Jtask_hqp;
                std::vector<VectorXd> fstar_hqp;

                Jtask_hqp.resize(3);
                fstar_hqp.resize(3);

                Eigen::MatrixXd Jtask_hand(12, MODEL_DOF_VIRTUAL);
                Jtask_hqp[0] = tocabi_.link_[COM_id].Jac;

                Jtask_hqp[1].resize(12, MODEL_DOF_VIRTUAL);
                Jtask_hqp[1].block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;
                Jtask_hqp[1].block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM;

                Jtask_hqp[2] = tocabi_.link_[Upper_Body].Jac_COM_r;

                /* tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                tocabi_.J_task.block(9, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_p;
                tocabi_.J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_r;
                tocabi_.J_task.block(15, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM_p;
                tocabi_.J_task.block(18, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM_r;*/

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].x_init + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].x_init;
                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[COM_id].rot_desired = DyrosMath::rotateWithY(tc.pelv_pitch * 3.1415 / 180.0);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0); //Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                //tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init;
                /*tocabi_.f_star.segment(9, 3) = tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].v + tocabi_.link_[Pelvis].v);
                tocabi_.f_star.segment(12, 3) = tocabi_.link_[Left_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].w + tocabi_.link_[Pelvis].w);
                tocabi_.f_star.segment(15, 3) = tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].v + tocabi_.link_[Pelvis].v);
                tocabi_.f_star.segment(18, 3) = tocabi_.link_[Right_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].w + tocabi_.link_[Pelvis].w);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);*/

                Eigen::Vector3d left_hand_local(tc.l_x + 0.3, tc.l_y + 0.3, tc.l_z);
                Eigen::Vector3d right_hand_local(tc.r_x + 0.3, tc.r_y - 0.3, tc.r_z);

                //tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * left_hand_local;
                //tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * right_hand_local;

                tocabi_.link_[Left_Hand].x_init_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Upper_Body].x_init);
                tocabi_.link_[Right_Hand].x_init_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Upper_Body].x_init);

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Left_Hand].x_init_local, left_hand_local);
                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Right_Hand].x_init_local, right_hand_local);

                Eigen::Vector3d l_pos_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Left_Hand].xpos - tocabi_.link_[Upper_Body].xpos);
                Eigen::Vector3d r_pos_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Right_Hand].xpos - tocabi_.link_[Upper_Body].xpos);

                Eigen::Vector3d l_v_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Left_Hand].v - tocabi_.link_[Upper_Body].v);
                Eigen::Vector3d r_v_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Right_Hand].v - tocabi_.link_[Upper_Body].v);

                fstar_hqp[0] = wbc_.getfstar6d(tocabi_, COM_id);
                fstar_hqp[1].resize(12);
                //fstar_hqp[1].segment(0, 3) = tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].v + tocabi_.link_[Pelvis].v);
                fstar_hqp[1].segment(0, 3) = tocabi_.link_[Left_Hand].pos_p_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Left_Hand].x_traj - l_pos_local)) + tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Left_Hand].v_traj - l_v_local));
                fstar_hqp[1].segment(3, 3) = tocabi_.link_[Left_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].w + tocabi_.link_[Upper_Body].w);
                //fstar_hqp[1].segment(6, 3) = tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].v + tocabi_.link_[Pelvis].v);
                fstar_hqp[1].segment(6, 3) = tocabi_.link_[Right_Hand].pos_p_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Right_Hand].x_traj - r_pos_local)) + tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Right_Hand].v_traj - r_v_local));
                fstar_hqp[1].segment(9, 3) = tocabi_.link_[Right_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].w + tocabi_.link_[Upper_Body].w);

                // tocabi_.link_[Left_Hand].x_init_local = tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Pelvis].x_init;
                //tocabi_.link_[Right_Hand].x_init_local = tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Pelvis].x_init;

                //tocabi_.link_[Left_Hand].

                //fstar_hqp[1].segment(0,3) = tocabi_.link_[Left_Hand].pos_p_gain.cwiseProduct()
                //fstar_hqp[1].block(0,6) = wbc_.getfs
                fstar_hqp[2] = wbc_.getfstar_rot(tocabi_, Upper_Body);

                //tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));

                //(tocabi_.lambda * tocabi_.f_star)

                //torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver); // + wbc_.contact_torque_calc_from_QP(tocabi_, torque_grav);
                //wbc_.calc_winv(tocabi_);
                //std::cout << "tm2" << std::endl;
                wbc_.set_contact(tocabi_, 1, 1);
                //std::cout << "tm2" << std::endl;

                wbc_.copy_robot_fast(tocabi_, tocabi_fast_, Jtask_hqp, fstar_hqp);
                //std::cout << "tm2" << std::endl;

                dc.trigger_hqp = true;

                //torque_task = wbc_.task_control_torque_hqp(tocabi_, Jtask_hqp, fstar_hqp);
                //torque_grav = tocabi_fast_.torque_grav;
                tocabi_.contact_redistribution_mode = 2;

                //wbc_.contact_torque_calc_from_QP(tocabi_, torque_task);

                // std::cout << "ContactForce Without : " << std::endl
                //          << (wbc_.get_contact_force(tocabi_, torque_task)).transpose() << std::endl;
            }
            else if (tc.mode == 3) //Pelv pos&rot control + upper rotation
            {
                //tocabi_.d_time_
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. */
                int task_number = 21;

                std::vector<MatrixXd> Jtask_hqp;
                std::vector<VectorXd> fstar_hqp;

                Jtask_hqp.resize(3);
                fstar_hqp.resize(3);

                Eigen::MatrixXd Jtask_hand(12, MODEL_DOF_VIRTUAL);
                Jtask_hqp[0] = tocabi_.link_[COM_id].Jac;

                Jtask_hqp[1].resize(12, MODEL_DOF_VIRTUAL);
                Jtask_hqp[1].block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;
                Jtask_hqp[1].block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM;

                Jtask_hqp[2] = tocabi_.link_[Upper_Body].Jac_COM_r;

                /* tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                tocabi_.J_task.block(9, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_p;
                tocabi_.J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_r;
                tocabi_.J_task.block(15, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM_p;
                tocabi_.J_task.block(18, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM_r;*/

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].x_init + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].x_init;
                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                double start = tc.ratio * tocabi_.link_[Left_Foot].x_init(1) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].x_init(1);
                double end = (1.0 - tc.ratio) * tocabi_.link_[Left_Foot].x_init(1) + (tc.ratio) * tocabi_.link_[Right_Foot].x_init(1);
                double length = start - end;

                tocabi_.link_[COM_id].x_traj(1) = start - length * 0.5 + cos((tocabi_.control_time_ - tc.command_time) * 2.0 * 3.141592 / tc.traj_time) * length * 0.5;
                tocabi_.link_[COM_id].v_traj(1) = -0.5 * length * 2.0 * 3.141592 / tc.traj_time * sin((tocabi_.control_time_ - tc.command_time) * 2.0 * 3.141592 / tc.traj_time);

                tocabi_.link_[COM_id].rot_desired = DyrosMath::rotateWithY(tc.pelv_pitch * 3.1415 / 180.0);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0); //Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                /*tocabi_.f_star.segment(9, 3) = tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].v + tocabi_.link_[Pelvis].v);
                tocabi_.f_star.segment(12, 3) = tocabi_.link_[Left_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].w + tocabi_.link_[Pelvis].w);
                tocabi_.f_star.segment(15, 3) = tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].v + tocabi_.link_[Pelvis].v);
                tocabi_.f_star.segment(18, 3) = tocabi_.link_[Right_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].w + tocabi_.link_[Pelvis].w);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);*/

                Eigen::Vector3d left_hand_local(tc.l_x + 0.3, tc.l_y + 0.3, tc.l_z);
                Eigen::Vector3d right_hand_local(tc.r_x + 0.3, tc.r_y - 0.3, tc.r_z);

                //tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * left_hand_local;
                //tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * right_hand_local;

                tocabi_.link_[Left_Hand].x_init_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Upper_Body].x_init);
                tocabi_.link_[Right_Hand].x_init_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Upper_Body].x_init);

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Left_Hand].x_init_local, left_hand_local);
                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Right_Hand].x_init_local, right_hand_local);

                Eigen::Vector3d l_pos_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Left_Hand].xpos - tocabi_.link_[Upper_Body].xpos);
                Eigen::Vector3d r_pos_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Right_Hand].xpos - tocabi_.link_[Upper_Body].xpos);

                Eigen::Vector3d l_v_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Left_Hand].v - tocabi_.link_[Upper_Body].v);
                Eigen::Vector3d r_v_local = tocabi_.link_[Upper_Body].Rotm.transpose() * (tocabi_.link_[Right_Hand].v - tocabi_.link_[Upper_Body].v);

                fstar_hqp[0] = wbc_.getfstar6d(tocabi_, COM_id);
                fstar_hqp[1].resize(12);
                //fstar_hqp[1].segment(0, 3) = tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].v + tocabi_.link_[Pelvis].v);
                double hand_z_diff = tc.l_z - tc.r_z;
                tocabi_.link_[Left_Hand].x_traj(2) = tc.l_z - 0.5 * hand_z_diff + 0.5 * hand_z_diff * cos((tocabi_.control_time_ - tc.command_time) * 2.0 * 3.141592 / tc.traj_time);
                tocabi_.link_[Right_Hand].x_traj(2) = tc.r_z + 0.5 * hand_z_diff - 0.5 * hand_z_diff * cos((tocabi_.control_time_ - tc.command_time) * 2.0 * 3.141592 / tc.traj_time);

                tocabi_.link_[Left_Hand].v_traj(2) = -0.5 * hand_z_diff * sin((tocabi_.control_time_ - tc.command_time) * 2 * 3.141592 / tc.traj_time) * 2.0 * 3.141592 / tc.traj_time;
                tocabi_.link_[Right_Hand].v_traj(2) = +0.5 * hand_z_diff * sin((tocabi_.control_time_ - tc.command_time) * 2 * 3.141592 / tc.traj_time) * 2.0 * 3.141592 / tc.traj_time;

                //tocabi_.link_[Left_Hand].pos_p_gain<<200.0,200.0,200.0;
                //tocabi_.link_[Right_Hand].pos_p_gain<<200.0,200.0,200.0;

                //tocabi_.link_[Left_Hand].pos_d_gain<<20.0,20.0,20.0;
                //tocabi_.link_[Right_Hand].pos_d_gain<<20.0,20.0,20.0;

                //tocabi_.link_[Left_Hand].rot_d_gain<<20.0,20.0,20.0;
                //tocabi_.link_[Right_Hand].rot_d_gain<<20.0,20.0,20.0;

                fstar_hqp[1].segment(0, 3) = tocabi_.link_[Left_Hand].pos_p_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Left_Hand].x_traj - l_pos_local)) + tocabi_.link_[Left_Hand].pos_d_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Left_Hand].v_traj - l_v_local));
                fstar_hqp[1].segment(3, 3) = tocabi_.link_[Left_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Left_Hand].w + tocabi_.link_[Upper_Body].w);
                //fstar_hqp[1].segment(6, 3) = tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].v + tocabi_.link_[Pelvis].v);
                fstar_hqp[1].segment(6, 3) = tocabi_.link_[Right_Hand].pos_p_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Right_Hand].x_traj - r_pos_local)) + tocabi_.link_[Right_Hand].pos_d_gain.cwiseProduct(tocabi_.link_[Upper_Body].Rotm * (tocabi_.link_[Right_Hand].v_traj - r_v_local));
                fstar_hqp[1].segment(9, 3) = tocabi_.link_[Right_Hand].rot_d_gain.cwiseProduct(-tocabi_.link_[Right_Hand].w + tocabi_.link_[Upper_Body].w);

                //fstar_hqp[1].segment(0,3) = tocabi_.link_[Left_Hand].
                //tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].rotm *

                // tocabi_.link_[Left_Hand].x_init_local = tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Pelvis].x_init;
                //tocabi_.link_[Right_Hand].x_init_local = tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Pelvis].x_init;

                //tocabi_.link_[Left_Hand].

                //fstar_hqp[1].segment(0,3) = tocabi_.link_[Left_Hand].pos_p_gain.cwiseProduct()
                //fstar_hqp[1].block(0,6) = wbc_.getfs

                tocabi_.link_[Upper_Body].r_traj = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0 * cos((tocabi_.control_time_ - tc.command_time) * 2 * 3.141592 / tc.traj_time)) * DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0);
                tocabi_.link_[Upper_Body].w_traj.setZero();
                tocabi_.link_[Upper_Body].w_traj(2) = -tc.yaw * 3.1415 / 180.0 * sin((tocabi_.control_time_ - tc.command_time) * 2 * 3.141592 / tc.traj_time) * 2 * 3.141592 / tc.traj_time;
                fstar_hqp[2] = wbc_.getfstar_rot(tocabi_, Upper_Body);

                //tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));

                //(tocabi_.lambda * tocabi_.f_star)

                //torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver); // + wbc_.contact_torque_calc_from_QP(tocabi_, torque_grav);
                //wbc_.calc_winv(tocabi_);

                std::chrono::steady_clock::time_point tp_qp = std::chrono::steady_clock::now();
                wbc_.set_contact(tocabi_, 1, 1);
                static double max_time = 0;
                static double all_time = 0;
                static double aall_time = 0;
                static int tcc = 0;

                double cctime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tp_qp).count();
                all_time += cctime;
                aall_time += cctime;

                if (cctime > max_time)
                    max_time = cctime;
                static int tc = 0;
                tc++;
                tcc++;
                if (tc > 2000)
                {
                    std::cout << "sc avg compute time : " << all_time / tc << "us"
                              << "total : " << aall_time / tcc << " max : " << max_time << std::endl;
                    all_time = 0;
                    tc = 0;
                }

                wbc_.copy_robot_fast(tocabi_, tocabi_fast_, Jtask_hqp, fstar_hqp);

                //print trajectory info
                tocabi_.data_print = true;

                dc.data_out << control_time_ << " \t"
                            << tocabi_.link_[COM_id].x_traj.transpose() << " \t" << tocabi_.link_[COM_id].xpos.transpose() << " \t"
                            << DyrosMath::rot2Euler_tf(tocabi_.link_[Pelvis].r_traj).transpose() << " \t" << DyrosMath::rot2Euler_tf(tocabi_.link_[Pelvis].Rotm).transpose() << " \t"
                            << tocabi_.link_[Left_Hand].x_traj.transpose() << " \t" << l_pos_local.transpose() << " \t"
                            << tocabi_.link_[Right_Hand].x_traj.transpose() << " \t" << r_pos_local.transpose() << " \t"
                            << DyrosMath::rot2Euler_tf(tocabi_.link_[Upper_Body].r_traj).transpose() << " \t" << DyrosMath::rot2Euler_tf(tocabi_.link_[Upper_Body].Rotm).transpose() << std::endl;

                dc.data_out2 << control_time_ << " \t"
                             << tocabi_.ee_[0].zmp(0) - tocabi_.ee_[0].cp_(0) << " \t" << tocabi_.ee_[0].zmp(1) - tocabi_.ee_[0].cp_(1) << " \t"
                             << tocabi_.ee_[1].zmp(0) - tocabi_.ee_[1].cp_(0) << " \t" << tocabi_.ee_[1].zmp(1) - tocabi_.ee_[1].cp_(1) << " \t"
                             << tocabi_.ContactForce.transpose() << " \t"
                             << dc.torque_desired.transpose() << std::endl;

                dc.trigger_hqp = true;

                //torque_task = wbc_.task_control_torque_hqp(tocabi_, Jtask_hqp, fstar_hqp);
                torque_grav = tocabi_fast_.torque_grav;
                tocabi_.contact_redistribution_mode = 2;
            }
            else if (tc.mode == 4) //left single
            {
                int task_number = 9;
                wbc_.set_contact(tocabi_, 1, 0);

                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[Left_Foot].xpos;

                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = DyrosMath::rotateWithY(tc.pelv_pitch * 3.1415 / 180.0);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[COM_id].x_traj(1) = tocabi_.link_[Left_Foot].xpos(1);
                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0);
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
                torque_grav.setZero();
            }
            else if (tc.mode == 5) //right single
            {
                int task_number = 6;
                wbc_.set_contact(tocabi_, 0, 1);

                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                //tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[Right_Foot].x_init;

                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = DyrosMath::rotateWithY(tc.pelv_pitch * 3.1415 / 180.0);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[COM_id].x_traj(1) = tocabi_.link_[Right_Foot].xpos(1);
                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX((tc.roll) * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0);
                //tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                //tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver) + wbc_.footRotateAssist(tocabi_, 1, 0);
                torque_grav.setZero(); // = wbc_.gravity_compensation_torque(tocabi_);
            }
            else if (tc.mode == 6) // double right
            {
                int task_number = 9;
                wbc_.set_contact(tocabi_, 1, 1);

                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[Right_Foot].xpos;

                tocabi_.link_[COM_id].x_desired(2) = tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                tocabi_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                //std::cout << "R_ fstar 0 :" << tocabi_.f_star(0) << " \t1: " << tocabi_.f_star(1) << " \t2: " << tocabi_.f_star(2) << " \ta: " << tocabi_.link_[COM_id].a_traj(1) << " \tx: " << tocabi_.link_[COM_id].x_traj(1) << " \tv: " << tocabi_.link_[COM_id].v_traj(1) <<" \tv: " << tocabi_.link_[COM_id].xpos(1) << std::endl;
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
                torque_grav.setZero(); // = wbc_.gravity_compensation_torque(tocabi_);
            }
            else if (tc.mode == 7) //walking test
            {
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. */
                if (tc.task_init)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (tocabi_.ee_[i].contact != tc.link_contact[i])
                        {
                            if (tc.link_contact[i])
                            {
                                if (tocabi_.ee_[i].contact_transition_mode == -1)
                                {
                                    std::cout << i << " : Contact Enable Sequence " << std::endl;
                                    tocabi_.ee_[i].contact_transition_mode = 1;
                                    tocabi_.ee_[i].contact_time = tocabi_.control_time_;
                                }
                            }
                            else
                            {
                                if (tocabi_.ee_[i].contact_transition_mode == -1)
                                {
                                    std::cout << i << " : Contact Disable Sequence " << std::endl;
                                    tocabi_.ee_[i].contact_transition_mode = 0;
                                    tocabi_.ee_[i].contact_time = tocabi_.control_time_;
                                }
                            }
                        }
                        else
                        {
                            tocabi_.ee_[i].contact_transition_mode = -1;
                        }
                    }
                    tc.task_init = false;
                }

                bool cc_[4];

                for (int i = 0; i < 4; i++)
                {

                    if (tocabi_.ee_[i].contact_transition_mode == 0)
                    {
                        cc_[i] = true;
                        if (tocabi_.control_time_ > (tocabi_.ee_[i].contact_time + tocabi_.contact_transition_time))
                        {
                            tocabi_.ee_[i].contact_transition_mode = -1;
                        }
                    }
                    else if (tocabi_.ee_[i].contact_transition_mode == 1)
                    {
                        cc_[i] = true;
                        if (tocabi_.control_time_ > (tocabi_.ee_[i].contact_time + tocabi_.contact_transition_time))
                        {
                            tocabi_.ee_[i].contact_transition_mode = -1;
                        }
                    }
                    else
                    {
                        cc_[i] = tc.link_contact[i];
                    }
                }

                wbc_.set_contact(tocabi_, cc_[0], cc_[1], cc_[2], cc_[3]);
                //torque_grav = wbc_.gravity_compensation_torque(tocabi_);

                int task_number = 6;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL);
                tocabi_.J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                tocabi_.link_[COM_id].x_desired(0) = tocabi_.link_[COM_id].x_desired(0) + tc.ratio;
                tocabi_.link_[COM_id].x_desired(1) = tocabi_.link_[COM_id].x_desired(1) + tc.height;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX((tc.roll) * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0);
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 3) = wbc_.getfstar_tra(tocabi_, COM_id);
                tocabi_.f_star.segment(3, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);

                if ((tc.ratio == 0) && (tc.height == 0))
                {
                    tocabi_.f_star.setZero();
                }
                tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));

                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);

                torque_grav.setZero(); // = wbc_.gravity_compensation_torque_QP2(tocabi_);

                /*
                int task_number = 9;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Pelvis].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                tocabi_.link_[Pelvis].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[Pelvis].x_desired(2) = tc.height;
                tocabi_.link_[Pelvis].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[Pelvis].rot_desired = DyrosMath::rotateWithY(tc.pelv_pitch * 3.1415 / 180.0);
                tocabi_.link_[Pelvis].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithZ(tc.yaw * 3.1415 / 180.0) * DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0) * DyrosMath::rotateWithY(tc.pitch * 3.1415 / 180.0);
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, Pelvis);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);

                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
                torque_grav.setZero();*/

                /*
                //Pre parameter settings
                double foot_distance = 0.2;
                double step_length = 0.0;
                int step_number = 10;
                double step_time = tc.traj_time;
                double gc = 9.81;
                double w = sqrt(gc / tc.init_com_height);
                double b = exp(w * step_time);
                double footswing_start_ratio = 0.8;

                Vector3d footup;
                footup.setZero();
                footup(2) = 0.03;

                Vector3d footdown;
                footdown.setZero();
                footdown(2) = -0.00;

                //Footstep settings
                Vector2d footstep[step_number];
                footstep[0] = tocabi_.link_[Left_Foot].x_init.segment(0, 2);
                footstep[1] = tocabi_.link_[Right_Foot].x_init.segment(0, 2);

                for (int i = 2; i < step_number; i++)
                {
                    footstep[i](0) = footstep[0](0) + (i - 1) * step_length;
                    footstep[i](1) = footstep[i - 2](1);
                }
                footstep[step_number - 1](0) = footstep[step_number - 2](0);

                //Capture point settings
                Vector2d cp[step_number];
                cp[0] = (footstep[0] + footstep[1]) / 2;
                cp[step_number - 1] = (footstep[step_number - 1] + footstep[step_number - 2]) / 2;
                for (int i = 1; i < (step_number - 1); i++)
                {
                    cp[step_number - 1 - i] = 1 / b * cp[step_number - i] - (1 - b) / b * footstep[step_number - i - 1];
                }

                //Zmp point settings
                Vector2d zmp[step_number];
                for (int i = 0; i < (step_number - 1); i++)
                {
                    zmp[i] = cp[i + 1] / (1 - b) - b / (1 - b) * cp[i];
                }
                zmp[step_number - 1] = cp[step_number - 1];

                Vector2d pos_init_state[step_number + 1];
                Vector2d vel_init_state[step_number + 1];
                pos_init_state[0] = cp[0];
                vel_init_state[0].setZero();
                for (int i = 1; i <= step_number; i++)
                {
                    pos_init_state[i] = cosh(w * step_time) * pos_init_state[i - 1] + 1 / w * sinh(w * step_time) * vel_init_state[i - 1] + (1 - cosh(w * step_time)) * zmp[i - 1];
                    vel_init_state[i] = w * sinh(w * step_time) * pos_init_state[i - 1] + cosh(w * step_time) * vel_init_state[i - 1] - w * sinh(w * step_time) * zmp[i - 1];
                }

                Vector2d p_ds1[step_number];
                Vector2d v_ds1[step_number];
                Vector2d a_ds1[step_number];

                Vector2d p_ds2[step_number];
                Vector2d v_ds2[step_number];
                Vector2d a_ds2[step_number];

                bool ds_enable = true;

                double ds_r1, ds_r2;
                double ds_ratio = 0.1;
                ds_r1 = ds_ratio / 2;
                ds_r2 = 1 - ds_ratio / 2;
                static int before_step = 0;

                for (int i = 0; i < step_number; i++)
                {
                    p_ds1[i] = cosh(w * step_time * ds_r1) * pos_init_state[i] + 1 / w * sinh(w * step_time * ds_r1) * vel_init_state[i] + (1 - cosh(w * step_time * ds_r1)) * zmp[i];
                    v_ds1[i] = w * sinh(w * step_time * ds_r1) * pos_init_state[i] + cosh(w * step_time * ds_r1) * vel_init_state[i] - w * sinh(w * step_time * ds_r1) * zmp[i];
                    a_ds1[i] = w * w * cosh(w * step_time * ds_r1) * pos_init_state[i] + w * sinh(w * step_time * ds_r1) * vel_init_state[i] - w * w * cosh(w * step_time * ds_r1) * zmp[i];

                    p_ds2[i] = cosh(w * step_time * ds_r2) * pos_init_state[i] + 1 / w * sinh(w * step_time * ds_r2) * vel_init_state[i] + (1 - cosh(w * step_time * ds_r2)) * zmp[i];
                    v_ds2[i] = w * sinh(w * step_time * ds_r2) * pos_init_state[i] + cosh(w * step_time * ds_r2) * vel_init_state[i] - w * sinh(w * step_time * ds_r2) * zmp[i];
                    a_ds2[i] = w * w * cosh(w * step_time * ds_r2) * pos_init_state[i] + w * sinh(w * step_time * ds_r2) * vel_init_state[i] - w * w * cosh(w * step_time * ds_r2) * zmp[i];
                }

                //zmp[step_number - 1] = (cp[step_number - 1] - cosh(w * step_time) * pos_init_state[step_number - 1] - 1 / w * sinh(w * step_time) * vel_init_state[step_number - 1]) / (1 - cosh(w * step_time));

                int current_step = (int)((tocabi_.control_time_ - tc.command_time) / tc.traj_time);

                if (current_step == before_step)
                {
                    std::cout << control_time_ << " current step : " << current_step << std::endl;
                    before_step++;
                    if (before_step > step_number)
                    {
                        before_step = 0;
                    }
                }

                if (current_step >= step_number)
                    current_step = step_number;

                double currnet_time_w = tocabi_.control_time_ - tc.command_time - (double)current_step * tc.traj_time;

                double foot_up_time = tc.command_time + (double)current_step * tc.traj_time + (1 - footswing_start_ratio) * tc.traj_time * 0.5;
                double foot_down_time = tc.command_time + (double)current_step * tc.traj_time + 0.5 * tc.traj_time;
                double foot_move_dur = tc.traj_time * footswing_start_ratio * 0.5;

                if (current_step < step_number)
                {
                    tocabi_.link_[COM_id].x_traj.segment(0, 2) = cosh(w * currnet_time_w) * pos_init_state[current_step] + 1 / w * sinh(w * currnet_time_w) * vel_init_state[current_step] + (1 - cosh(w * currnet_time_w)) * zmp[current_step];
                    tocabi_.link_[COM_id].v_traj.segment(0, 2) = w * sinh(w * currnet_time_w) * pos_init_state[current_step] + cosh(w * currnet_time_w) * vel_init_state[current_step] - w * sinh(w * currnet_time_w) * zmp[current_step];
                    tocabi_.link_[COM_id].a_traj.segment(0, 2) = w * w * cosh(w * currnet_time_w) * pos_init_state[current_step] + w * sinh(w * currnet_time_w) * vel_init_state[current_step] - w * w * cosh(w * currnet_time_w) * zmp[current_step];

                    if ((current_step > 0) && (current_step < (step_number - 1)))
                    {

                        if (current_step % 2 == 0)
                        {
                            int task_number = 15;
                            wbc_.set_contact(tocabi_, 1, 0);
                            tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                            tocabi_.f_star.setZero(task_number);
                            tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                            tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                            tocabi_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Foot].Jac_COM;
                            tocabi_.link_[Right_Foot].x_traj = tocabi_.link_[Right_Foot].x_init;

                            if (tocabi_.control_time_ > foot_up_time)
                            {

                                tocabi_.link_[Right_Foot].Set_Trajectory_from_quintic(tocabi_.control_time_, foot_up_time, foot_up_time + foot_move_dur, tocabi_.link_[Right_Foot].x_init, tocabi_.link_[Right_Foot].x_init + footup);
                            }
                            if (tocabi_.control_time_ >= foot_down_time)
                            {
                                tocabi_.link_[Right_Foot].Set_Trajectory_from_quintic(tocabi_.control_time_, foot_down_time, foot_down_time + foot_move_dur, tocabi_.link_[Right_Foot].x_init + footup, tocabi_.link_[Right_Foot].x_init + footdown);
                            }
                        }
                        else
                        {

                            int task_number = 15;
                            wbc_.set_contact(tocabi_, 0, 1);
                            tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                            tocabi_.f_star.setZero(task_number);
                            tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                            tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                            tocabi_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Foot].Jac_COM;
                            tocabi_.link_[Left_Foot].x_traj = tocabi_.link_[Left_Foot].x_init;
                            if (tocabi_.control_time_ > foot_up_time)
                            {
                                tocabi_.link_[Left_Foot].Set_Trajectory_from_quintic(tocabi_.control_time_, foot_up_time, foot_up_time + foot_move_dur, tocabi_.link_[Left_Foot].x_init, tocabi_.link_[Left_Foot].x_init + footup);
                            }
                            if (tocabi_.control_time_ >= foot_down_time)
                            {
                                tocabi_.link_[Left_Foot].Set_Trajectory_from_quintic(tocabi_.control_time_, foot_down_time, foot_down_time + foot_move_dur, tocabi_.link_[Left_Foot].x_init + footup, tocabi_.link_[Left_Foot].x_init + footdown);
                            }
                        }
                    }
                    else
                    {

                        int task_number = 9;
                        wbc_.set_contact(tocabi_, 1, 1);
                        tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                        tocabi_.f_star.setZero(task_number);
                        tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                        tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                    }

                    if (tocabi_.control_time_)
                    {
                    }

                    if (ds_enable)
                    {

                        if (current_step == 0)
                        {
                            if (currnet_time_w > step_time * ds_r2)
                            {
                                Vector3d cx_init, cv_init, cx_des, cv_des, ca_init, ca_des;
                                cx_init.segment(0, 2) = p_ds2[current_step];
                                cx_init(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_init.segment(0, 2) = v_ds2[current_step];
                                cv_init(2) = 0;
                                ca_init.segment(0, 2) = a_ds2[current_step];
                                ca_init(2) = 0;
                                cx_des.segment(0, 2) = p_ds1[current_step + 1];
                                cx_des(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_des.segment(0, 2) = v_ds1[current_step + 1];
                                cv_des(2) = 0;
                                ca_des.segment(0, 2) = a_ds1[current_step + 1];
                                ca_des(2) = 0;

                                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(currnet_time_w, step_time * ds_r2, step_time * (1 + ds_r1), cx_init, cv_init, ca_init, cx_des, cv_des, ca_des);
                            }
                        }
                        else if ((current_step > 0) && (current_step < (step_number - 1)))
                        {
                            if (currnet_time_w < step_time * ds_r1)
                            {
                                Vector3d cx_init, cv_init, cx_des, cv_des, ca_init, ca_des;
                                cx_init.segment(0, 2) = p_ds2[current_step - 1];
                                cx_init(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_init.segment(0, 2) = v_ds2[current_step - 1];
                                cv_init(2) = 0;
                                ca_init.segment(0, 2) = a_ds2[current_step - 1];
                                ca_init(2) = 0;
                                cx_des.segment(0, 2) = p_ds1[current_step];
                                cx_des(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_des.segment(0, 2) = v_ds1[current_step];
                                cv_des(2) = 0;
                                ca_des.segment(0, 2) = a_ds1[current_step];
                                ca_des(2) = 0;

                                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(currnet_time_w, -step_time * ds_r1, step_time * ds_r1, cx_init, cv_init, ca_init, cx_des, cv_des, ca_des);
                            }
                            else if (currnet_time_w > step_time * ds_r2)
                            {
                                Vector3d cx_init, cv_init, cx_des, cv_des, ca_init, ca_des;
                                cx_init.segment(0, 2) = p_ds2[current_step];
                                cx_init(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_init.segment(0, 2) = v_ds2[current_step];
                                cv_init(2) = 0;
                                ca_init.segment(0, 2) = a_ds2[current_step];
                                ca_init(2) = 0;
                                cx_des.segment(0, 2) = p_ds1[current_step + 1];
                                cx_des(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_des.segment(0, 2) = v_ds1[current_step + 1];
                                cv_des(2) = 0;
                                ca_des.segment(0, 2) = a_ds1[current_step + 1];
                                ca_des(2) = 0;

                                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(currnet_time_w, step_time * ds_r2, step_time * (1 + ds_r1), cx_init, cv_init, ca_init, cx_des, cv_des, ca_des);
                            }
                        }
                        else if (current_step == (step_number - 1))
                        {
                            if (currnet_time_w < step_time * ds_r1)
                            {
                                Vector3d cx_init, cv_init, cx_des, cv_des, ca_init, ca_des;
                                cx_init.segment(0, 2) = p_ds2[current_step - 1];
                                cx_init(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_init.segment(0, 2) = v_ds2[current_step - 1];
                                cv_init(2) = 0;
                                ca_init.segment(0, 2) = a_ds2[current_step - 1];
                                ca_init(2) = 0;
                                cx_des.segment(0, 2) = p_ds1[current_step];
                                cx_des(2) = tocabi_.link_[COM_id].x_init(2);
                                cv_des.segment(0, 2) = v_ds1[current_step];
                                cv_des(2) = 0;
                                ca_des.segment(0, 2) = a_ds1[current_step];
                                ca_des(2) = 0;

                                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(currnet_time_w, -step_time * ds_r1, step_time * ds_r1, cx_init, cv_init, ca_init, cx_des, cv_des, ca_des);
                            }
                        }
                    }
                }
                else
                {
                    Vector3d cx_init, cv_init, cx_des, cv_des;
                    cx_init.segment(0, 2) = pos_init_state[step_number];
                    cx_init(2) = tocabi_.link_[COM_id].x_init(2);
                    cv_init.segment(0, 2) = vel_init_state[step_number];
                    cv_init(2) = 0;
                    cx_des.segment(0, 2) = cp[step_number - 1];
                    cx_des(2) = tocabi_.link_[COM_id].x_init(2);
                    cv_des.setZero();
                    tocabi_.link_[COM_id].Set_Trajectory_from_quintic(currnet_time_w, 0, step_time, cx_init, cv_init, cx_des, cv_des);
                    //tocabi_.link_[COM_id].v_traj.segment(0, 2).setZero();
                    //tocabi_.link_[COM_id].a_traj.segment(0, 2).setZero();
                }

                //for (int i =)

                tocabi_.link_[COM_id].x_traj(2) = tocabi_.link_[COM_id].x_init(2);

                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                tocabi_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                //std::cout << "R_ fstar 0 :" << tocabi_.f_star(0) << " \t1: " << tocabi_.f_star(1) << " \t2: " << tocabi_.f_star(2) << " \ta: " << tocabi_.link_[COM_id].a_traj(1) << " \tx: " << tocabi_.link_[COM_id].x_traj(1) << " \tv: " << tocabi_.link_[COM_id].v_traj(1) <<" \tv: " << tocabi_.link_[COM_id].xpos(1) << std::endl;
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                if (current_step < (step_number - 1))
                {

                    if (current_step > 0)
                    {
                        if (current_step % 2 == 0)
                        {
                            tocabi_.f_star.segment(9, 6) = wbc_.getfstar6d(tocabi_, Right_Foot);
                        }
                        else
                        {
                            tocabi_.f_star.segment(9, 6) = wbc_.getfstar6d(tocabi_, Left_Foot);
                        }
                    }
                }

                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
                torque_grav.setZero(); // = wbc_.gravity_compensation_torque(tocabi_);*/
            }
            else if (tc.mode == 8) //UpperBody Joint control
            {

                //Online CP dynamics Trajectory Generator
                //ZMP generator
                //ZMP generated with 10hz;/
                //Ooooooo!

                //ZMP of right, left, all

                //

                static int step_cnt = -1;
                static int step_cnt_before = -1;

                double step_time = tc.traj_time;
                double step_length = tc.ratio;
                double elapsed_time = control_time_ - tc.command_time;
                double elapsed_step_time;
                double left_step_time;
                int step_number = 20;

                double foot_distance = 0.201;
                double gc = 9.81;
                double w = sqrt(gc / tc.init_com_height);
                double b = exp(w * step_time);

                Vector2d footstep[10];

                static Vector2d cp_init;
                static Vector2d cp_desired;
                Vector2d cp_current = 1 / w * tocabi_.link_[COM_id].v.segment(0, 2) + tocabi_.link_[COM_id].xpos.segment(0, 2);
                tocabi_.CP_(0) = cp_current(0);
                tocabi_.CP_(1) = cp_current(1);
                tocabi_.CP_(2) = 0;

                static int support_foot = -1;
                static int swing_foot = -1;

                Vector2d cp_mod[2];
                cp_mod[0] = {0.03, -0.02};
                cp_mod[1] = {0.03, 0.02};

                static Vector2d cp_est;
                static Vector2d zmp_desired;
                static bool first_loop = true;

                Vector2d est_com_pos;
                Vector2d est_com_vel;

                Vector2d current_com_pos = tocabi_.com_.pos.segment(0, 2);
                Vector2d current_com_vel = tocabi_.com_.vel.segment(0, 2);

                Eigen::MatrixXd lambda_temp_inv;
                Vector6d com_fs;
                Vector2d acc_desired;

                static bool next_step = true;

                if (tc.task_init)
                {
                    next_step = true;
                    step_cnt = -1;
                    tc.task_init = false;
                    zmp_desired.setZero();

                    support_foot = -1;
                    swing_foot = -1;
                }
                if ((cp_current - cp_desired).norm() < 0.02)
                {
                    next_step = true;
                }
                else if (elapsed_step_time > step_time)
                {
                    next_step = true;
                }

                if (next_step)
                {
                    next_step = false;
                    step_cnt++;

                    std::cout << "current step : " << step_cnt; // std::endl;
                    if (step_cnt == 0)
                    {
                        cp_init = tocabi_.link_[COM_id].xpos.segment(0, 2);
                        cp_desired = tocabi_.ee_[1].cp_.segment(0, 2) + cp_mod[1];
                    }
                    else if (step_cnt > 0)
                    {
                        support_foot = step_cnt % 2;
                        swing_foot = (step_cnt + 1) % 2;

                        cp_init = tocabi_.ee_[support_foot].cp_.segment(0, 2) + cp_mod[support_foot];
                        cp_desired = tocabi_.ee_[swing_foot].cp_.segment(0, 2) + cp_mod[swing_foot];
                    }
                    if (support_foot == 0)
                    {
                        std::cout << "  support foot : left" << std::endl;
                        //tocabi_.link_[Right_Foot].x_init = tocabi_.link_[Right_Foot].xpos;
                    }
                    else
                    {
                        std::cout << "  support foot : right" << std::endl;
                        //tocabi_.link_[Left_Foot].x_init = tocabi_.link_[Left_Foot].xpos;
                    }
                }
                tocabi_.CP_desired(0) = cp_desired(0);
                tocabi_.CP_desired(1) = cp_desired(1);
                elapsed_step_time = elapsed_time - step_cnt * (step_time);
                left_step_time = step_time - elapsed_step_time;

                if (left_step_time > step_time * 0.01)
                    zmp_desired = (cp_desired - exp(w * left_step_time) * cp_current) / (1 - exp(w * left_step_time));

                //Check if is in foot or not//

                Vector3d zmp_checker;
                static Vector3d zmp_desired_before;

                zmp_checker.setZero();
                zmp_checker.segment(0, 2) = zmp_desired;
                Vector3d ee_local_zmp;
                ee_local_zmp = tocabi_.ee_[support_foot].rotm.inverse() * (zmp_checker - tocabi_.ee_[support_foot].cp_);
                //tocabi_.zmp_
                if (step_cnt > 0)
                {

                    tocabi_.ZMP_ft = wbc_.GetZMPpos_fromFT(tocabi_);
                    //std::cout << "ZMP error x : " << tocabi_.ZMP_error(0) << "  y : " << tocabi_.ZMP_error(1) << std::endl;
                    bool zmp_regulated = false;
                    if (abs(ee_local_zmp(0)) > tocabi_.ee_[support_foot].cs_x_length)
                    {
                        zmp_regulated = true;
                        if (ee_local_zmp(0) > 0)
                        {
                            ee_local_zmp(0) = tocabi_.ee_[support_foot].cs_x_length;
                        }
                        else
                        {
                            ee_local_zmp(0) = -tocabi_.ee_[support_foot].cs_x_length;
                        }
                    }
                    if (abs(ee_local_zmp(1)) > tocabi_.ee_[support_foot].cs_y_length)
                    {
                        zmp_regulated = true;
                        if (ee_local_zmp(1) > 0)
                        {
                            ee_local_zmp(1) = tocabi_.ee_[support_foot].cs_y_length;
                        }
                        else
                        {
                            ee_local_zmp(1) = -tocabi_.ee_[support_foot].cs_y_length;
                        }
                    }

                    if (zmp_regulated)
                    {
                        std::cout << "zmp regulated  x : " << ee_local_zmp(0) << " y : " << ee_local_zmp(1) << std::endl;
                    }
                    if (left_step_time > step_time * 0.01)
                        zmp_desired = (tocabi_.ee_[support_foot].rotm * ee_local_zmp + tocabi_.ee_[support_foot].cp_).segment(0, 2);

                    tocabi_.ZMP_error.segment(0, 2) = zmp_desired_before.segment(0, 2) - tocabi_.ZMP_ft.segment(0, 2);
                    if (zmp_desired_before(0) * tocabi_.ZMP_ft(0) < 0)
                    {
                        tocabi_.ZMP_error(0) = 0;
                    }
                    if (zmp_desired_before(1) * tocabi_.ZMP_ft(1) < 0)
                    {
                        tocabi_.ZMP_error(1) = 0;
                    }
                }
                double T;
                double Tx = 0;
                Vector3d x_selector;
                x_selector << 1, 0, 0;

                static Vector3d link_rot_vel[12];
                Vector3d link_acc[12];
                for (int i = 0; i < 12; i++)
                {
                    link_acc[i] = (tocabi_.link_[4 + i].v - link_rot_vel[i]) / tocabi_.d_time_;
                    Tx = Tx + (x_selector.transpose() * (tocabi_.link_[4 + i].Rotm * tocabi_.link_[4 + i].inertia * tocabi_.link_[4 + i].Rotm.transpose()) * link_acc[i] / (tocabi_.total_mass * 9.81)).sum();

                    link_rot_vel[i] = tocabi_.link_[4 + i].v;
                }

                tocabi_.ZMP_desired2(0) = Tx;

                est_com_pos = cosh(w * left_step_time) * current_com_pos + 1 / w * sinh(2 * left_step_time) * current_com_vel + (1 - cosh(w * left_step_time)) * tocabi_.ZMP_ft.segment(0, 2);
                est_com_vel = w * sinh(w * left_step_time) * current_com_pos + cosh(w * left_step_time) * current_com_vel - w * sinh(w * left_step_time) * tocabi_.ZMP_ft.segment(0, 2);
                tocabi_.ZMP_desired.segment(0, 2) = zmp_desired;
                cp_est = est_com_vel / w + est_com_pos;

                //est_com_pos = cosh(w*)
                int task_number;
                if (step_cnt > 0)
                {
                    if (support_foot == 0) //left foot support
                    {
                        wbc_.set_contact(tocabi_, 1, 0);
                        task_number = 6 + 3 + 6;
                    }
                    else if (support_foot == 1) //right foot support
                    {
                        wbc_.set_contact(tocabi_, 0, 1);
                        task_number = 6 + 3 + 6;
                    }
                }
                else
                {
                    wbc_.set_contact(tocabi_, 1, 1);
                    task_number = 6 + 3;
                }

                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                bool ra_right, ra_left;

                ra_right = false;
                ra_left = false;

                Vector3d foot_up, foot_down;

                foot_up << 0, 0, 0.03;
                foot_down << 0, 0, -0.03;
                if (step_cnt > 0)
                {
                    if (support_foot == 0) //left foot support
                    {
                        tocabi_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Foot].Jac_Contact;
                        tocabi_.link_[Right_Foot].Set_Trajectory_from_quintic(elapsed_step_time, step_time * 0.05, step_time * 0.4, tocabi_.link_[Right_Foot].x_init, tocabi_.link_[Right_Foot].x_init + foot_up);
                        if (elapsed_step_time >= step_time * 0.4)
                            tocabi_.link_[Right_Foot].Set_Trajectory_from_quintic(elapsed_step_time, step_time * 0.4, step_time * 0.7, tocabi_.link_[Right_Foot].x_init + foot_up, tocabi_.link_[Right_Foot].x_init);
                        tocabi_.f_star.segment(9, 6) = wbc_.getfstar6d(tocabi_, Right_Foot);
                        ra_right = true;
                    }
                    else if (support_foot == 1) //right foot support
                    {
                        tocabi_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Foot].Jac_Contact;
                        tocabi_.link_[Left_Foot].Set_Trajectory_from_quintic(elapsed_step_time, step_time * 0.05, step_time * 0.4, tocabi_.link_[Left_Foot].x_init, tocabi_.link_[Left_Foot].x_init + foot_up);
                        if (elapsed_step_time >= step_time * 0.4)
                            tocabi_.link_[Left_Foot].Set_Trajectory_from_quintic(elapsed_step_time, step_time * 0.4, step_time * 0.7, tocabi_.link_[Left_Foot].x_init + foot_up, tocabi_.link_[Left_Foot].x_init);
                        tocabi_.f_star.segment(9, 6) = wbc_.getfstar6d(tocabi_, Left_Foot);
                        ra_left = true;
                    }
                }
                //tocabi_.task_force_control = true;

                //lambda_temp_inv = tocabi_.J_task * tocabi_.A_matrix_inverse * tocabi_.N_C * tocabi_.J_task.transpose();

                acc_desired = w * w * (tocabi_.link_[COM_id].xpos.segment(0, 2) - zmp_desired);

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.f_star.segment(0, 3) = wbc_.getfstar_tra(tocabi_, COM_id);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                tocabi_.f_star.segment(3, 3) = wbc_.getfstar_rot(tocabi_, COM_id);
                tocabi_.f_star.segment(0, 2) = acc_desired;

                tocabi_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                //torque_task = wbc_.task_control_torque_with_gravity(tocabi_, tocabi_.J_task, tocabi_.f_star);
                VectorXd force_desired_;
                wbc_.GetTaskLambda(tocabi_, tocabi_.J_task);

                force_desired_ = tocabi_.lambda * tocabi_.f_star;

                if (step_cnt > 0)
                {
                    if (elapsed_step_time > 0.45 * step_time)
                        force_desired_(11) = force_desired_(11) + DyrosMath::minmax_cut((elapsed_step_time - 0.45 * step_time) / (0.4 * step_time), 0, 1) * -45.0;
                }

                VectorQd torque_task_initial = wbc_.task_control_torque_with_gravity(tocabi_, tocabi_.J_task, force_desired_, true); //wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
                Vector3d zmp_id = wbc_.GetZMPpos(tocabi_, wbc_.get_contact_force(tocabi_, torque_task_initial));

                ee_local_zmp = tocabi_.ee_[support_foot].rotm.inverse() * (zmp_id - tocabi_.ee_[support_foot].cp_);
                //tocabi_.zmp_
                if (step_cnt > 0)
                {
                    bool zmp_regulated = false;
                    if (abs(ee_local_zmp(0)) > tocabi_.ee_[support_foot].cs_x_length)
                    {
                        zmp_regulated = true;
                        if (ee_local_zmp(0) > 0)
                        {
                            ee_local_zmp(0) = tocabi_.ee_[support_foot].cs_x_length;
                        }
                        else
                        {
                            ee_local_zmp(0) = -tocabi_.ee_[support_foot].cs_x_length;
                        }
                    }
                    if (abs(ee_local_zmp(1)) > tocabi_.ee_[support_foot].cs_y_length)
                    {
                        zmp_regulated = true;
                        if (ee_local_zmp(1) > 0)
                        {
                            ee_local_zmp(1) = tocabi_.ee_[support_foot].cs_y_length;
                        }
                        else
                        {
                            ee_local_zmp(1) = -tocabi_.ee_[support_foot].cs_y_length;
                        }
                    }
                    if (zmp_regulated)
                    {
                        std::cout << "zmp regulated 2 x : " << ee_local_zmp(0) << " y : " << ee_local_zmp(1) << std::endl;
                        zmp_id = tocabi_.ee_[support_foot].rotm * ee_local_zmp + tocabi_.ee_[support_foot].cp_;
                    }
                }

                Vector3d zmp_error_id = zmp_id - tocabi_.ZMP_ft;
                VectorQd torque_foot_control_add;
                VectorQd torque_pelvis_rot_add;
                torque_pelvis_rot_add.setZero();
                torque_foot_control_add.setZero();
                torque_foot_control_add(5) = 0.0;
                torque_foot_control_add(5 + 6) = 0.0;
                Vector3d eulr_f;

                Vector3d ZMP_command;
                Vector3d p_gain, i_gain;
                Vector3d p_control;

                static Vector3d integral_;
                static double int_x, int_y;
                Vector3d i_control;

                ZMP_command.setZero();

                p_gain << 0.5, 0.5, 0;
                i_gain << 1.0, 1.0, 0;

                p_control(0) = p_gain(0) * zmp_error_id(0);
                p_control(1) = p_gain(1) * zmp_error_id(1);

                if (step_cnt_before != step_cnt)
                {
                    std::cout << "integral initialize" << std::endl;
                    int_x = 0;
                    int_y = 0;
                    integral_.setZero();
                }

                double int_before = integral_(1);

                int_x = int_x + zmp_error_id(0) * tocabi_.d_time_;
                int_y = int_y + zmp_error_id(1) * tocabi_.d_time_;

                i_control(0) = i_gain(0) * int_x;
                i_control(1) = i_gain(1) * int_y;

                ZMP_command = p_control + i_control;

                if (step_cnt_before != step_cnt)
                {
                    ZMP_command.setZero();
                }
                if (step_cnt >= 1)
                {
                    //std::cout << control_time_ << " zmp int_y : " << int_y << "  zmp error : " << tocabi_.ZMP_error(1) << std::endl;
                    // std::cout << control_time_ << "zmp c x : " << ZMP_command(0) << " y : " << ZMP_command(1) << std::endl;
                    //std::cout << "integral : " << int_y << "  zmp_ft : " << tocabi_.ZMP_ft(1) << "  zmp des : " << zmp_desired_before(1) << "  zmperr : " << tocabi_.ZMP_error(1) << " dtime : " << tocabi_.d_time_ << std::endl;
                }

                Vector3d pelv_v = tocabi_.link_[Pelvis].Rotm.transpose() * tocabi_.link_[Pelvis].w;

                double pelv_rot_ass_p = 100;
                double pelv_rot_ass_d = 20;

                if (support_foot == 0)
                {
                    torque_foot_control_add(4) = -ZMP_command(0) * tocabi_.ContactForce_FT(2); //pitch
                    torque_foot_control_add(5) = ZMP_command(1) * tocabi_.ContactForce_FT(2);  //roll

                    //torque_pelvis_rot_add(1) = -(tocabi_.roll * pelv_rot_ass_p + pelv_v(0) * pelv_rot_ass_d);

                    //eulr_f = DyrosMath::rot2Euler_tf(tocabi_.link_[Right_Foot].Rotm);
                    //eulr_f = DyrosMath::rot2Euler_tf(tocabi_.link_[Right_Foot].Rotm * DyrosMath::rotateWithZ(-eulr_f(2)));
                    //torque_foot_control_add += wbc_.footRotateAssist(tocabi_, 0, 1);
                }
                else if (support_foot == 1)
                {
                    torque_foot_control_add(4 + 6) = -ZMP_command(0) * tocabi_.ContactForce_FT(2 + 6);
                    torque_foot_control_add(5 + 6) = ZMP_command(1) * tocabi_.ContactForce_FT(2 + 6);
                    //torque_foot_control_add += wbc_.footRotateAssist(tocabi_, 1, 0);
                    //torque_pelvis_rot_add(1 + 6) = tocabi_.roll * pelv_rot_ass_p + pelv_v(0) * pelv_rot_ass_d;
                }

                //std::cout << "excuse me?" << std::endl;
                tocabi_.ZMP_desired2(1) = zmp_id(1);
                torque_task = torque_task_initial;
                //DyrosMath::minmax_cut(elapsed_step_time / step_time * 20.0, 0, 1) *
                torque_add = torque_foot_control_add + wbc_.footRotateAssist(tocabi_, ra_left, ra_right) + torque_pelvis_rot_add;

                //std::cout << "excuse u!" << std::endl;
                tocabi_.f_star.setZero();
                torque_grav.setZero();
                //tocabi_.ZMP_desired2(0) = int_y;
                zmp_desired_before.segment(0, 2) = zmp_desired;
                first_loop = false;
                step_cnt_before = step_cnt;
            }
            else if (tc.mode == 9)
            {
                wbc_.set_contact(tocabi_, 1, 1);
                torque_grav = wbc_.gravity_compensation_torque(tocabi_);

                int task_number = 9 + 12;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                tocabi_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM;
                tocabi_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;

                if (tc.task_init)
                {
                    std::cout << "Velocity Control initialized " << std::endl;

                    tocabi_.link_[COM_id].x_traj = tocabi_.link_[COM_id].x_init;
                    tocabi_.link_[COM_id].v_traj = tocabi_.link_[COM_id].v_init;

                    tocabi_.link_[COM_id].r_traj = tocabi_.link_[COM_id].rot_init;
                    tocabi_.link_[COM_id].w_traj = tocabi_.link_[COM_id].w_init;

                    tocabi_.link_[Upper_Body].r_traj = tocabi_.link_[Upper_Body].rot_init;
                    tocabi_.link_[Upper_Body].w_traj = tocabi_.link_[Upper_Body].w_init;

                    tocabi_.link_[Right_Hand].x_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Upper_Body].x_init);
                    tocabi_.link_[Right_Hand].v_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Right_Hand].v_init - tocabi_.link_[Upper_Body].v_init);
                    tocabi_.link_[Right_Hand].r_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * tocabi_.link_[Right_Hand].rot_init;
                    tocabi_.link_[Right_Hand].w_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Right_Hand].w_init - tocabi_.link_[Upper_Body].w_init);

                    tocabi_.link_[Left_Hand].x_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Upper_Body].x_init);
                    tocabi_.link_[Left_Hand].v_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Left_Hand].v_init - tocabi_.link_[Upper_Body].v_init);
                    tocabi_.link_[Left_Hand].r_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * tocabi_.link_[Left_Hand].rot_init;
                    tocabi_.link_[Left_Hand].w_traj_local = tocabi_.link_[Upper_Body].rot_init.transpose() * (tocabi_.link_[Left_Hand].w_init - tocabi_.link_[Upper_Body].w_init);

                    tc.task_init = false;
                }

                if (vc.link_ == 0)
                {
                    tocabi_.link_[COM_id].x_traj = tocabi_.link_[COM_id].x_traj + 0.5 * tocabi_.d_time_ * vc.des_vel.segment(0, 3);
                    tocabi_.link_[COM_id].v_traj = vc.des_vel.segment(0, 3);
                    tocabi_.link_[COM_id].r_traj = DyrosMath::Add_vel_to_Rotm(tocabi_.link_[COM_id].r_traj, vc.des_vel.segment(3, 3), tocabi_.d_time_);
                    tocabi_.link_[COM_id].w_traj = vc.des_vel.segment(3, 3);
                }
                else if (vc.link_ == 1)
                {
                    tocabi_.link_[Upper_Body].r_traj = DyrosMath::Add_vel_to_Rotm(tocabi_.link_[Upper_Body].r_traj, vc.des_vel.segment(3, 3), tocabi_.d_time_);
                    tocabi_.link_[Upper_Body].w_traj = vc.des_vel.segment(3, 3);
                }
                else if (vc.link_ == 2)
                {
                    tocabi_.link_[Right_Hand].x_traj_local = tocabi_.link_[Right_Hand].x_traj_local + 0.5 * tocabi_.d_time_ * vc.des_vel.segment(0, 3);
                    tocabi_.link_[Right_Hand].v_traj_local = vc.des_vel.segment(0, 3);
                    tocabi_.link_[Right_Hand].r_traj_local = DyrosMath::Add_vel_to_Rotm(tocabi_.link_[Right_Hand].r_traj_local, vc.des_vel.segment(3, 3), tocabi_.d_time_);
                    tocabi_.link_[Right_Hand].w_traj_local = vc.des_vel.segment(3, 3);
                }
                else if (vc.link_ == 3)
                {
                    tocabi_.link_[Left_Hand].x_traj_local = tocabi_.link_[Left_Hand].x_traj_local + 0.5 * tocabi_.d_time_ * vc.des_vel.segment(0, 3);
                    tocabi_.link_[Left_Hand].v_traj_local = vc.des_vel.segment(0, 3);
                    tocabi_.link_[Left_Hand].r_traj_local = DyrosMath::Add_vel_to_Rotm(tocabi_.link_[Left_Hand].r_traj_local, vc.des_vel.segment(3, 3), tocabi_.d_time_);
                    tocabi_.link_[Left_Hand].w_traj_local = vc.des_vel.segment(3, 3);
                }

                tocabi_.link_[Right_Hand].x_traj = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Right_Hand].x_traj_local;
                tocabi_.link_[Right_Hand].v_traj = tocabi_.link_[Upper_Body].v + tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Right_Hand].v_traj_local;
                tocabi_.link_[Right_Hand].r_traj = tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Right_Hand].r_traj_local;
                tocabi_.link_[Right_Hand].w_traj = tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Right_Hand].w_traj_local;

                tocabi_.link_[Left_Hand].x_traj = tocabi_.link_[Upper_Body].xpos + tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Left_Hand].x_traj_local;
                tocabi_.link_[Left_Hand].v_traj = tocabi_.link_[Upper_Body].v + tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Left_Hand].v_traj_local;
                tocabi_.link_[Left_Hand].r_traj = tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Left_Hand].r_traj_local;
                tocabi_.link_[Left_Hand].w_traj = tocabi_.link_[Upper_Body].Rotm * tocabi_.link_[Left_Hand].w_traj_local;

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                tocabi_.f_star.segment(9, 6) = wbc_.getfstar6d(tocabi_, Right_Hand);
                tocabi_.f_star.segment(15, 6) = wbc_.getfstar6d(tocabi_, Left_Hand);

                torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);

                torque_grav.setZero();
            }
            else if (tc.mode >= 10)
            {
                tocabi_.contact_redistribution_mode = 2;

                if (tc_command == true)
                {
                    mycontroller.taskCommandToCC(tc);
                    tc_command = false;
                }
                torque_grav.setZero();
                try
                {
                    tp[3] = std::chrono::steady_clock::now();
                    mycontroller.computeSlow();
                }
                catch (exception &e)
                {
                    std::cout << "Error Thrown from customcontroller::computeSlow : " << e.what() << std::endl;
                }
            }

            if (dc.print_data_ready)
            {
                //dc.data_out << control_time_ << "\t" << tocabi_.link_[COM_id].xpos(1) << "\t" << tocabi_.link_[COM_id].x_traj(1) << std::endl;
            }

            if (tocabi_.lambda_calc)
            {
                if (tocabi_.lambda.cols() == tocabi_.f_star.rows())
                    tocabi_.TaskForce = tocabi_.lambda * tocabi_.f_star;
            }

            for (int i = 0; i < tocabi_.TaskForce.size(); i++)
            {
                if (tocabi_.TaskForce(i) > 300)
                {
                    std::cout << cyellow << control_time_ << " : task force warning detected. at " << i << ", " << tocabi_.TaskForce(i) << creset << std::endl;

                    std::cout << "pelv z : " << tocabi_.link_[Pelvis].xpos(2) << "  \tcom xpos : " << tocabi_.link_[COM_id].xpos(2) << "  \tcom x traj : " << tocabi_.link_[COM_id].x_traj(2) << "  \tcom v : " << tocabi_.link_[COM_id].v(2) << "  \tcom v traj : " << tocabi_.link_[COM_id].v_traj(2) << std::endl;

                    dc.tc_state = 1;
                    if (tocabi_.TaskForce(i) > 1000)
                    {
                        std::cout << cred << " task force limit approach. task control disabled. " << creset << std::endl;
                        dc.signal_gravityCompensation = true;
                        dc.tc_state = 2;
                    }
                }
            }
        }
        else
        {
            wbc_.set_contact(tocabi_, 1, 1);
            torque_grav = wbc_.gravity_compensation_torque(tocabi_);
        }

        TorqueDesiredLocal = torque_grav + torque_task;
        
        if (dc.signal_contactTorqueRedistribution)
        {
            dc.signal_contactTorqueRedistribution = false;
            tocabi_.contact_redistribution_mode++;
            if (tocabi_.contact_redistribution_mode > 2)
                tocabi_.contact_redistribution_mode = 0;

            if (tocabi_.contact_redistribution_mode == 0)
                std::cout << "contact torque redistribution by yslee " << std::endl;
            else if (tocabi_.contact_redistribution_mode == 1)
                std::cout << "contact torque redistribution by qp " << std::endl;
            else if (tocabi_.contact_redistribution_mode == 2)
                std::cout << "contact torque redistribution disabled " << std::endl;
        }

        if (tocabi_.contact_redistribution_mode == 0)
        {
            TorqueContact = wbc_.contact_force_redistribution_torque(tocabi_, TorqueDesiredLocal, fc_redis, fc_ratio);
        }
        else if (tocabi_.contact_redistribution_mode == 1)
        {
            TorqueContact = wbc_.contact_torque_calc_from_QP(tocabi_, TorqueDesiredLocal);
        }
        tocabi_.torque_contact = TorqueContact;
        ///////////////////////////////////////////////////////////////////////////////////////
        //////////////////              Controller Code End             ///////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////
        mtx.lock();
        if (dc.positionControl == false)
        {
            torque_desired = TorqueDesiredLocal + TorqueContact + torque_add;
        }
        if (dc.positionGravControl == true)
        {
            if (tc.mode >= 10)
                torque_grav = tocabi_.torque_grav_cc;
        }

        mtx.unlock();

        tocabi_.ContactForce = wbc_.get_contact_force(tocabi_, torque_desired);

        // for (int i = 0; i < 2; i++)
        // {
        //     if (tocabi_.ee_[i].contact)
        //     {
        //         tocabi_.ee_[i].contact_accuracy = abs((tocabi_.ee_[i].contact_force.segment(0, 3).norm() - tocabi_.ee_[i].contact_force_ft.segment(0, 3).norm()) / tocabi_.ee_[i].contact_force.segment(0, 3).norm());
        //     }
        //     else
        //     {
        //         tocabi_.ee_[i].contact_accuracy = 0;
        //     }
        // }

        tocabi_.ZMP = wbc_.GetZMPpos(tocabi_);
        tocabi_.ZMP_ft = wbc_.GetZMPpos_fromFT(tocabi_);
        //tocabi_.ZMP_eqn_calc(0) = (tocabi_.link_[COM_id].x_traj(0) * 9.8 - tocabi_.com_.pos(2) * tocabi_.link_[COM_id].a_traj(0)) / 9.8;
        tocabi_.ZMP_eqn_calc(0) = (tocabi_.link_[COM_id].x_traj(1) * 9.81 - (tocabi_.com_.pos(2) - tocabi_.link_[Right_Foot].xpos(2) * 0.5 - tocabi_.link_[Left_Foot].xpos(2) * 0.5) * tocabi_.link_[COM_id].a_traj(1)) / 9.81;
        tocabi_.ZMP_eqn_calc(1) = (tocabi_.link_[COM_id].x_traj(1) * 9.81 - (tocabi_.com_.pos(2) - tocabi_.link_[Right_Foot].xpos(2) * 0.5 - tocabi_.link_[Left_Foot].xpos(2) * 0.5) * tocabi_.link_[COM_id].a_traj(1)) / 9.81 + tocabi_.com_.angular_momentum(0) / (tocabi_.com_.mass * 9.81);
        tocabi_.ZMP_eqn_calc(2) = 0.0;

        if (shutdown_tocabi_bool)
            break;
        first = false;

        std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - dyn_loop_start;

        est += elapsed_time.count();

        if (dc.showdata)
        {

            //wbc_.fstar_regulation(tocabi_, tocabi_.f_star);
            /*
            std::chrono::steady_clock::time_point tp_pinv = std::chrono::steady_clock::now();
            DyrosMath::pinv_SVD(tocabi_.W);
            std::chrono::duration<double> pinv_time = std::chrono::steady_clock::now() - tp_pinv;
            double time_base = pinv_time.count();

            std::cout << "pinv eigen SVD  : " << pinv_time.count() * 1000.0 << std::endl;

            tp_pinv = std::chrono::steady_clock::now();
            DyrosMath::pinv_QR2(tocabi_.W);
            pinv_time = std::chrono::steady_clock::now() - tp_pinv;
            std::cout << "pinv eigen QR : " << pinv_time.count() * 1000.0 << "  " << pinv_time.count() / time_base * 100 << " %" << std::endl;

            tp_pinv = std::chrono::steady_clock::now();
            DyrosMath::pinv_glsSVD(tocabi_.W);
            pinv_time = std::chrono::steady_clock::now() - tp_pinv;
            std::cout << "pinv gls SVD : " << pinv_time.count() * 1000.0 << "  " << pinv_time.count() / time_base * 100 << " %" << std::endl;

            tp_pinv = std::chrono::steady_clock::now();
            DyrosMath::pinv_glsQR2(tocabi_.W);
            pinv_time = std::chrono::steady_clock::now() - tp_pinv;
            std::cout << "pinv gls QR : " << pinv_time.count() * 1000.0 << "  " << pinv_time.count() / time_base * 100 << " %" << std::endl;
*/
            //std::cout << "test" << std::endl
            //          << DyrosMath::pinv_QR2(tocabi_.W) - DyrosMath::pinv_glsQR2(tocabi_.W) << std::endl;

            for (int i = 0; i < 9; i++)
            {
                std::cout << "td" << i << "  " << td[i].count() * 1000 << std::endl;
            }

            std::cout << elapsed_time.count() * 1000 << "ms !!" << std::endl;

            dc.showdata = false;
        }
    }
    std::cout << cyellow << "Dynamics Slow Thread End !" << creset << std::endl;
}
void TocabiController::customgainhandle()
{
    if (tc.custom_taskgain)
    {
        tocabi_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[COM_id].acc_p_gain = Vector3d::Ones() * tc.acc_p;

        tocabi_.link_[Pelvis].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[Pelvis].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[Pelvis].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[Pelvis].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[Pelvis].acc_p_gain = Vector3d::Ones() * tc.acc_p;

        tocabi_.link_[Upper_Body].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[Upper_Body].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[Upper_Body].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[Upper_Body].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[Upper_Body].acc_p_gain = Vector3d::Ones() * tc.acc_p;

        tocabi_.link_[Right_Foot].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[Right_Foot].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[Right_Foot].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[Right_Foot].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[Right_Foot].acc_p_gain = Vector3d::Ones() * tc.acc_p;

        tocabi_.link_[Right_Hand].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[Right_Hand].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[Right_Hand].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[Right_Hand].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[Right_Hand].acc_p_gain = Vector3d::Ones() * tc.acc_p;

        tocabi_.link_[Left_Foot].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[Left_Foot].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[Left_Foot].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[Left_Foot].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[Left_Foot].acc_p_gain = Vector3d::Ones() * tc.acc_p;

        tocabi_.link_[Left_Hand].pos_p_gain = Vector3d::Ones() * tc.pos_p;
        tocabi_.link_[Left_Hand].pos_d_gain = Vector3d::Ones() * tc.pos_d;
        tocabi_.link_[Left_Hand].rot_p_gain = Vector3d::Ones() * tc.ang_p;
        tocabi_.link_[Left_Hand].rot_d_gain = Vector3d::Ones() * tc.ang_d;
        tocabi_.link_[Left_Hand].acc_p_gain = Vector3d::Ones() * tc.acc_p;
    }
}
void TocabiController::tuiThread()
{
    int ch;
    double before_time;
    bool pub_ = false;
    std::string str_text;

    while (!shutdown_tocabi_bool)
    {
        while ((!dc.connected) && (!shutdown_tocabi_bool))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (shutdown_tocabi_bool)
                break;
        }

        if (shutdown_tocabi_bool)
            break;

        for (int i = 0; i < 50; i++)
        { /*
            if (mtx_terminal.try_lock())
            {
                if (dc.Tq_[i].update)
                {
                    str_text = std::string(dc.Tq_[i].text);
                    dc.Tq_[i].update = false;
                    pub_ = true;
                }
                mtx_terminal.unlock();
            }
            if (pub_)
            {
                std::cout << str_text << std::endl;
                pub_ = false;
            }*/
        }

        ch = kbhit();
        //kch = -1;
        if (ch != -1)
        {
            std::cout << "key input : " << (char)(ch % 256) << std::endl;
            if ((ch % 256 == 'q'))
            {
                std::cout << "shutdown request" << std::endl;
                shutdown_tocabi_bool = true;
            }
            else if ((ch % 256 == 'p'))
            {
                std::cout << "position mode" << std::endl;
            }
            else if ((ch % 256 == 'r'))
            {
                std::cout << "disable safety lock" << std::endl;
                dc.disableSafetyLock = true;
            }
            else if ((ch % 256 == 'e'))
            {
                if (dc.torqueOn)
                {
                    std::cout << "torque is already enabled, command duplicated, ignoring command!" << std::endl;
                }
                else
                {
                    std::cout << "torque ON !" << std::endl;
                    dc.torqueOnTime = control_time_;
                    dc.torqueOn = true;
                    dc.torqueOff = false;
                }
            }
            else if ((ch % 256 == 'c'))
            {
                if (dc.torqueOn)
                {
                    std::cout << "Torque OFF ! " << std::endl;
                    dc.torqueOffTime = control_time_;
                    dc.torqueOn = false;
                    dc.torqueOff = true;
                }
                else if (dc.torqueOff)
                {
                    std::cout << "Torque is already disabled, command duplicated, ignoring command! " << std::endl;
                }
            }
            else if ((ch % 256 == 't'))
            {
                std::cout << "torque mode " << std::endl;
                dc.torquezeroByTerminal = true;
            }
            else if (ch % 256 == 'd')
            {
                dc.print_delay_info = !dc.print_delay_info;
            }
            else if (ch % 256 == 'i')
            {
                std::cout << "starting init seqence " << std::endl;

                dc.start_initialize_sequence = true;
            }
            else if (ch % 256 == 's')
            {
                dc.semode = !dc.semode;
            }
            else if (ch % 256 == 'f')
            {
                dc.showdata = true;
            }
        }

        if (dc.open_file_for_print && !dc.print_data_ready)
        {
            dc.tocabi_.data_print_switch = true;
            dc.data_out = std::ofstream(dc.print_file_name.c_str());
            dc.data_out << fixed;
            dc.data_out.precision(6);
            dc.data_out.width(15);
            dc.data_out2 = std::ofstream(dc.print_file_name2.c_str());
            dc.data_out2 << fixed;
            dc.data_out2.precision(6);
            dc.data_out2.width(15);
            if (dc.data_out.is_open())
            {
                std::cout << "Start Logging Data .... to " << dc.print_file_name << std::endl;
                dc.data_out << "Start Data Print" << std::endl;
                dc.print_data_ready = true;
            }
            else
            {
                std::cout << "Failed to open tocabi_data.txt" << std::endl;
            }
            dc.open_file_for_print = false;
        }

        //AURA CONTROL PART

        static bool safetymode_led = false;

        int tg1 = 255 * dc.t_gain;
        int tg2 = (255 - 63) * dc.t_gain;

        dc.rgbPubMsg.data = {0, 0, tg1, 0, 0, tg1, 0, 0, 63 + tg2, 0, 0, 63 + tg2, 0, 0, tg1, 0, 0, tg1};

        if (dc.safetyison)
        {
            if (((2 * control_time_ - floor(2 * control_time_)) - 0.5) >= 0)
            {
                dc.rgbPubMsg.data[0] = 255;
                dc.rgbPubMsg.data[1] = 0;
                dc.rgbPubMsg.data[2] = 0;

                dc.rgbPubMsg.data[15] = 0;
                dc.rgbPubMsg.data[16] = 0;
                dc.rgbPubMsg.data[17] = 0;
            }
            else
            {
                dc.rgbPubMsg.data[0] = 0;
                dc.rgbPubMsg.data[1] = 0;
                dc.rgbPubMsg.data[2] = 0;

                dc.rgbPubMsg.data[15] = 255;
                dc.rgbPubMsg.data[16] = 0;
                dc.rgbPubMsg.data[17] = 0;
            }
        }

        if (dc.elmoinstability)
        {
            if (((2 * control_time_ - floor(2 * control_time_)) - 0.5) >= 0)
            {
                dc.rgbPubMsg.data[0] = 255;
                dc.rgbPubMsg.data[1] = 0;
                dc.rgbPubMsg.data[2] = 0;

                dc.rgbPubMsg.data[15] = 255;
                dc.rgbPubMsg.data[16] = 0;
                dc.rgbPubMsg.data[17] = 0;
            }
            else
            {
                dc.rgbPubMsg.data[0] = 0;
                dc.rgbPubMsg.data[1] = 0;
                dc.rgbPubMsg.data[2] = 0;

                dc.rgbPubMsg.data[15] = 0;
                dc.rgbPubMsg.data[16] = 0;
                dc.rgbPubMsg.data[17] = 0;
            }
        }

        bool diff = false;

        for (int i = 0; i < 18; i++)
        {
            if (dc.rgbPubMsg_before.data[i] != dc.rgbPubMsg.data[i])
            {
                diff = true;
                break;
            }
        }

        if (diff)
        {
            dc.rgbPub.publish(dc.rgbPubMsg);
        }

        dc.rgbPubMsg_before = dc.rgbPubMsg;
        before_time = control_time_;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //std::cout<<control_time_<<"tui test"<<std::endl;
    }
    dc.rgbPubMsg.data = {0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0};
    dc.rgbPub.publish(dc.rgbPubMsg);

    std::cout << cyellow << "Terminal Thread End !" << creset << std::endl;
}

void TocabiController::getState()
{
    while (dc.atb_dc && (!shutdown_tocabi_bool))
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
    dc.atb_dc = true;

    time = dc.time;

    if (control_time_ != time)
    {
        control_time_ = dc.time;
        sim_time = dc.sim_time;
        dym_hz = dc.dym_hz;
        stm_hz = dc.stm_hz;
        //q_ = dc.q_;
        //q_virtual_ = dc.q_virtual_;
        //q_dot_ = dc.q_dot_;
        //q_dot_virtual_ = dc.q_dot_virtual_;
        //q_ddot_virtual_ = dc.q_ddot_virtual_;
        torque_ = dc.torque_;
        tocabi_.d_time_ = dc.time - tocabi_.control_time_;
        tocabi_.control_time_ = dc.time;
        tocabi_.q_ = dc.q_;
        tocabi_.q_virtual_ = dc.q_virtual_;
        tocabi_.q_dot_ = dc.q_dot_virtual_.segment(6, MODEL_DOF);
        tocabi_.q_dot_virtual_ = dc.q_dot_virtual_;
        tocabi_.q_ddot_virtual_ = dc.q_ddot_virtual_;
        tocabi_.q_dot_diff_ = tocabi_.q_dot_ - tocabi_.q_dot_before_;
        tocabi_.q_dot_before_ = tocabi_.q_dot_;
        //tocabi_.q_dot_virtual_before_ = tocabi_.q_dot_virtual_;
        tocabi_.q_dot_virtual_lpf_ = dc.q_dot_virtual_lpf;

        static bool first_run = true;
        if (first_run)
        {
            for (int i = 0; i < LINK_NUMBER + 1; i++)
            {
                dc.link_[i].pos_p_gain = tocabi_.link_[i].pos_p_gain;
                dc.link_[i].pos_d_gain = tocabi_.link_[i].pos_d_gain;
                dc.link_[i].rot_p_gain = tocabi_.link_[i].rot_p_gain;
                dc.link_[i].rot_d_gain = tocabi_.link_[i].rot_d_gain;

                tocabi_.link_[i] = dc.link_[i];
            }
            first_run = false;
        }
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {
            tocabi_.link_[i].xpos = dc.link_[i].xpos;
            tocabi_.link_[i].xipos = dc.link_[i].xipos;
            tocabi_.link_[i].Rotm = dc.link_[i].Rotm;
            tocabi_.link_[i].Jac = dc.link_[i].Jac;
            tocabi_.link_[i].Jac_COM = dc.link_[i].Jac_COM;
            tocabi_.link_[i].Jac_COM_p = dc.link_[i].Jac_COM_p;
            tocabi_.link_[i].Jac_COM_r = dc.link_[i].Jac_COM_r;
            tocabi_.link_[i].COM_position = dc.link_[i].COM_position;
            //tocabi_.link_[i].xpos_contact = dc.link_[i].xpos_contact;
            tocabi_.link_[i].v = dc.link_[i].v;
            tocabi_.link_[i].w = dc.link_[i].w;
        }

        tocabi_.roll = dc.roll;
        tocabi_.pitch = dc.pitch;
        tocabi_.yaw = dc.yaw;

        tocabi_.A_ = dc.A_;
        tocabi_.A_matrix = dc.A_;
        tocabi_.A_matrix_inverse = dc.A_inv;
        tocabi_.Motor_inertia = dc.Motor_inertia;
        tocabi_.Motor_inertia_inverse = dc.Motor_inertia_inverse;
        tocabi_.com_ = dc.com_;
        loop_pass = false;
    }
    else
    {
        loop_pass = true;
    }
    dc.atb_dc = false;
}

void TocabiController::trajectoryplannar()
{
    //wait for
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::chrono::steady_clock::time_point t_begin = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_from_begin, time_interval;
    std::chrono::microseconds cycletime(1000);
    int cycle_count = 0;

    while (!shutdown_tocabi_bool)
    {
        std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
        cycle_count++;
        std::chrono::steady_clock::time_point t_begin1 = std::chrono::steady_clock::now();
        time_from_begin = (t_begin1 - t_begin);

        if (tocabi_.task_control_switch)
        {
            if (tc.mode >= 10)
            {
                tocabi_.contact_redistribution_mode = 2;

                if (tc_command == true)
                {
                    mycontroller.taskCommandToCC(tc);
                    tc_command = false;
                }
                try
                {
                    mycontroller.computePlanner();
                }
                catch (exception &e)
                {
                    std::cout << "exception Thrown from customcontroller::computePlanner : " << e.what() << std::endl;
                }
                if (dc.positionControl)
                {
                    tocabi_.q_desired_ = mycontroller.getControl();
                }
            }
        }
    }
    std::cout << cyellow << "Planner Thread End!" << creset << std::endl;
}

void TocabiController::initialize()
{
    torque_desired.setZero();
    torque_grav.setZero();

    tocabi_.ee_[0].contact = true;
    tocabi_.ee_[1].contact = true;
    tocabi_.ee_[2].contact = false;
    tocabi_.ee_[3].contact = false;
}

void TocabiController::ContinuityChecker(double data)
{
}

void TocabiController::ZMPmonitor()
{
}

Eigen::Vector3d TocabiController::velRegulation(Eigen::Vector3d traj_before, Eigen::Vector3d traj_now, Eigen::Vector3d acc_max)
{
    Eigen::Vector3d res_traj;
    for (int i = 0; i < 3; i++)
    {
        if (abs((traj_before(i) - traj_now(i)) / tocabi_.d_time_) > acc_max(i))
        {
            if (traj_now(i) > traj_before(i))
            {
                res_traj(i) = traj_before(i) + acc_max(i) * 0.5 * tocabi_.d_time_ * tocabi_.d_time_;
            }
        }
    }
}
