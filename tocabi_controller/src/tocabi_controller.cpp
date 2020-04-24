#include "tocabi_controller/tocabi_controller.h"
#include "tocabi_controller/terminal.h"
#include "tocabi_controller/TaskCommand.h"
#include <tf/transform_datatypes.h>
#include "stdlib.h"
#include <fstream>
#include <sstream>

#include <iostream>
#include <string>

std::mutex mtx;
std::mutex mtx_rbdl;
std::mutex mtx_dc;
std::mutex mtx_terminal;
std::mutex mtx_ncurse;

TocabiController::TocabiController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm) : dc(dc_global), s_(sm), d_(dm), tocabi_(dc_global.tocabi_), wbc_(dc_global.wbc_), mycontroller(*(new CustomController(dc_global, dc_global.tocabi_)))
{
    initialize();
    task_command = dc.nh.subscribe("/tocabi/taskcommand", 100, &TocabiController::TaskCommandCallback, this);
    point_pub = dc.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/cdata_pub", 1);
    pointpub_msg.polygon.points.resize(13);
    point_pub2 = dc.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/point_pub2", 1);
    //pointpub_msg.polygon.points.reserve(20);
}

void TocabiController::pubfromcontroller()
{
    /*
    * Point pub info : 
    * 0 : com position
    * 1 : com velocity
    * 2 : com acceleration
    * 3 : com angular momentum
    * 4 : com position desired
    * 5 : com velocity desired
    * 6 : com acceleration desired
    * 7 : fstar
    */

    pointpub_msg.header.stamp = ros::Time::now();

    geometry_msgs::Point32 point;

    point.x = dc.tocabi_.com_.pos(0);
    point.y = dc.tocabi_.com_.pos(1);
    point.z = dc.tocabi_.com_.pos(2);
    pointpub_msg.polygon.points[0] = point;

    point.x = dc.tocabi_.com_.vel(0);
    point.y = dc.tocabi_.com_.vel(1);
    point.z = dc.tocabi_.com_.vel(2);
    pointpub_msg.polygon.points[1] = point;

    point.x = dc.tocabi_.com_.accel(0);
    point.y = dc.tocabi_.com_.accel(1);
    point.z = dc.tocabi_.com_.accel(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.tocabi_.com_.angular_momentum(0);
    point.y = dc.tocabi_.com_.angular_momentum(1);
    point.z = dc.tocabi_.com_.angular_momentum(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.tocabi_.link_[COM_id].x_traj(0);
    point.y = dc.tocabi_.link_[COM_id].x_traj(1);
    point.z = dc.tocabi_.link_[COM_id].x_traj(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.tocabi_.link_[COM_id].v_traj(0);
    point.y = dc.tocabi_.link_[COM_id].v_traj(1);
    point.z = dc.tocabi_.link_[COM_id].v_traj(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.tocabi_.link_[COM_id].a_traj(0);
    point.y = dc.tocabi_.link_[COM_id].a_traj(1);
    point.z = dc.tocabi_.link_[COM_id].a_traj(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.tocabi_.fstar(0);
    point.y = dc.tocabi_.fstar(1);
    point.z = dc.tocabi_.fstar(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.tocabi_.ZMP(0); //from task torque -> contact force -> zmp
    point.y = dc.tocabi_.ZMP(1);
    point.z = dc.tocabi_.ZMP(2);
    pointpub_msg.polygon.points[2] = point;

    point.x = dc.tocabi_.ZMP_local(0); //from acceleration trajecoty -> tasktorque -> contactforce -> zmp
    point.y = dc.tocabi_.ZMP_local(1);
    point.z = dc.tocabi_.ZMP_local(2);
    pointpub_msg.polygon.points[3] = point;

    point.x = dc.tocabi_.ZMP_eqn_calc(0); //from acceleration trajectory with zmp equation : xddot = w^2(x-p),  zmp = x - xddot/w^2
    point.y = dc.tocabi_.ZMP_eqn_calc(1);
    point.z = dc.tocabi_.ZMP_eqn_calc(2);
    pointpub_msg.polygon.points[4] = point;

    point.x = dc.tocabi_.ZMP_desired(0); //from acceleration trajectory with zmp equation : xddot = w^2(x-p),  zmp = x - xddot/w^2
    point.y = dc.tocabi_.ZMP_desired(1);
    point.z = dc.tocabi_.ZMP_desired(2);
    pointpub_msg.polygon.points[5] = point;

    point.x = dc.tocabi_.ZMP_ft(0); //calc from ft sensor
    point.y = dc.tocabi_.ZMP_ft(1);
    point.z = dc.tocabi_.ZMP_ft(2);
    pointpub_msg.polygon.points[6] = point;

    point.x = dc.tocabi_.LH_FT(0);
    point.y = dc.tocabi_.LH_FT(1);
    point.z = dc.tocabi_.LH_FT(2);
    pointpub_msg.polygon.points[7] = point;

    if (dc.tocabi_.ContactForce.size() == 18)
    {
        point.x = dc.tocabi_.ContactForce(12 + 0);
        point.y = dc.tocabi_.ContactForce(12 + 1);
        point.z = dc.tocabi_.ContactForce(12 + 2);
    }
    else
    {
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
    }

    pointpub_msg.polygon.points[8] = point;

    point.x = dc.tocabi_.link_[Left_Hand].xpos_contact(0);
    point.y = dc.tocabi_.link_[Left_Hand].xpos_contact(1);
    point.z = dc.tocabi_.link_[Left_Hand].xpos_contact(2);
    pointpub_msg.polygon.points[9] = point;

    point_pub.publish(pointpub_msg);
}

void TocabiController::TaskCommandCallback(const tocabi_controller::TaskCommandConstPtr &msg)
{
    tocabi_.control_time_pre_ = control_time_;

    tc.command_time = control_time_;
    tc.traj_time = msg->time;
    tc.mode = msg->mode;
    tc.task_init = true;

    tc.ratio = msg->ratio;
    tc.angle = msg->angle;
    tc.height = msg->height;

    tc.custom_taskgain = msg->customTaskGain;
    tc.pos_p = msg->pos_p;
    tc.pos_d = msg->pos_d;
    tc.ang_p = msg->ang_p;
    tc.ang_d = msg->ang_d;

    tc.l_x = msg->l_x;
    tc.l_y = msg->l_y;
    tc.l_z = msg->l_z;
    tc.l_roll = msg->l_roll * DEG2RAD;
    tc.l_pitch = msg->l_pitch * DEG2RAD;
    tc.l_yaw = msg->l_yaw * DEG2RAD;

    tc.r_x = msg->r_x;
    tc.r_y = msg->r_y;
    tc.r_z = msg->r_z;
    tc.r_roll = msg->r_roll * DEG2RAD;
    tc.r_pitch = msg->r_pitch * DEG2RAD;
    tc.r_yaw = msg->r_yaw * DEG2RAD;

    tocabi_.link_[Right_Foot].Set_initpos();
    tocabi_.link_[Left_Foot].Set_initpos();
    tocabi_.link_[Right_Hand].Set_initpos();
    tocabi_.link_[Left_Hand].Set_initpos();
    tocabi_.link_[Pelvis].Set_initpos();
    tocabi_.link_[Upper_Body].Set_initpos();
    tocabi_.link_[COM_id].Set_initpos();

    task_switch = true;
    tc_command = true;

    // Arm Desired Setting
    Eigen::Vector3d TargetDelta_l, TargetDelta_r;
    TargetDelta_l << tc.l_x, tc.l_y, tc.l_z;
    TargetDelta_r << tc.r_x, tc.r_y, tc.r_z;

    tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init + TargetDelta_l;
    tocabi_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(tc.l_roll) * DyrosMath::rotateWithY(tc.l_pitch) * DyrosMath::rotateWithZ(tc.l_yaw) * tocabi_.link_[Left_Hand].rot_init;
    tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Right_Hand].x_init + TargetDelta_r;
    tocabi_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(tc.r_roll) * DyrosMath::rotateWithY(tc.r_pitch) * DyrosMath::rotateWithZ(tc.r_yaw) * tocabi_.link_[Right_Hand].rot_init;

    std::cout << "init set - COM x : " << tocabi_.link_[COM_id].x_init(0) << "\t y : " << tocabi_.link_[COM_id].x_init(1) << std::endl;

    //walking
    tc.walking_enable = msg->walking_enable;
    tc.ik_mode = msg->ik_mode;
    tc.walking_pattern = msg->pattern;
    tc.foot_step_dir = msg->first_foot_step;
    tc.target_x = msg->x;
    tc.target_y = msg->y;
    tc.target_z = msg->z;
    tc.walking_height = msg->height;
    tc.theta = msg->theta;
    tc.step_length_y = msg->step_length_y;
    tc.step_length_x = msg->step_length_x;
    tc.dob = msg->dob;
    tc.walking_enable = msg->walking_enable;
    if (tc.walking_enable == 1.0 || tc.walking_enable == 2.0)
    {
        tc.mode = 11;
    }

    data_out << "###############  COMMAND RECEIVED  ###############" << std::endl;
}

void TocabiController::stateThread()
{
    s_.connect();
    s_.stateThread2();
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
                if (set_q_init)
                {
                    tocabi_.q_desired_ = tocabi_.q_;
                    tocabi_.q_init_ = tocabi_.q_;
                    set_q_init = false;
                }
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    torque_desired(i) = dc.tocabi_.Kps[i] * (tocabi_.q_desired_(i) - tocabi_.q_(i)) - dc.tocabi_.Kvs[i] * (tocabi_.q_dot_(i));
                }
                if (task_switch)
                {
                    if (tc.mode >= 10)
                    {
                        mycontroller.computeFast();
                    }
                }
            }
            else
            {
                if (task_switch)
                {
                    if (tc.mode >= 10)
                    {
                        mycontroller.computeFast();
                        torque_desired = mycontroller.getControl();
                    }
                }
            }
            mtx.lock();
            s_.sendCommand(torque_desired, sim_time);
            mtx.unlock();
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
    std::cout << "DynamicsThreadLow : READY ?" << std::endl;
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

    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    std::chrono::seconds sec1(1);

    std::chrono::duration<double> sec = std::chrono::high_resolution_clock::now() - start_time;
    bool display = false;
    std::chrono::high_resolution_clock::time_point start_time2 = std::chrono::high_resolution_clock::now();

    int contact_number = 2;
    int link_id[contact_number];
    int total_dof_ = MODEL_DOF;
    Eigen::MatrixXd Lambda_c, J_C_INV_T, N_C, I37, Slc_k, Slc_k_T, W, W_inv;
    Eigen::MatrixXd P_c;
    Eigen::VectorXd Grav_ref;
    Eigen::VectorXd G;
    Eigen::MatrixXd J_g;
    Eigen::MatrixXd aa;
    Eigen::VectorQd torque_grav, torque_task;
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

    Vector3d kp_, kd_, kpa_, kda_;
    for (int i = 0; i < 3; i++)
    {
        kp_(i) = 400;
        kd_(i) = 40;
        kpa_(i) = 400;
        kda_(i) = 40;
    }

    //kd_(1) = 120;

    std::cout << "DynamicsThreadLow : START" << std::endl;
    int dynthread_cnt = 0;

    //const char *file_name = "/home/saga/sim_data.txt";

    std::string path = dc.homedir + "/red_sim_data/";
    std::string current_time;

    time_t now = std::time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    current_time = buf;
    std::string file_name = path + "sim_data" + current_time + ".txt";
    data_out = std::ofstream(file_name.c_str());

    std::stringstream ss;

    ///////////////////////
    wbc_.init(tocabi_);

    std::chrono::steady_clock::time_point tp[6];
    std::chrono::duration<double> td[7];

    //Control Loop Start
    while ((!shutdown_tocabi_bool))
    {
        static double est;
        std::chrono::high_resolution_clock::time_point dyn_loop_start = std::chrono::high_resolution_clock::now();
        dynthread_cnt++;

        td[0] = tp[1] - tp[0];
        td[1] = tp[2] - tp[1];
        td[2] = tp[3] - tp[2];
        td[3] = tp[4] - tp[3];
        td[4] = tp[5] - tp[4];

        if (control_time_ == 0)
        {
            first = true;
            task_switch = false;
        }
        if ((dyn_loop_start - start_time2) > sec1)
        {
            start_time2 = std::chrono::high_resolution_clock::now();
            if (dc.checkfreqency)
            {
                ss.str("");
                ss << "dynamics thread : " << dynthread_cnt << " hz, time : " << est; //<< std::endl;
                //dc.statusPubMsg.data = ss.str();
                pub_to_gui(dc, ss.str().c_str());
                //dc.statusPub.publish(dc.statusPubMsg);
            }
            if (dc.print_delay_info)
            {
                std::cout << "td1 : " << td[0].count() * 1000 << "  td2 : " << td[1].count() * 1000 << "  td3 : " << td[2].count() * 1000 << "  td4 : " << td[3].count() * 1000 << "  td5 : " << td[4].count() * 1000 << std::endl;
            }
            dynthread_cnt = 0;
            est = 0;
        }
        tp[0] = std::chrono::steady_clock::now();
        getState(); //link data override

        tp[1] = std::chrono::steady_clock::now();
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

        wbc_.update(tocabi_);

        tp[2] = std::chrono::steady_clock::now();
        sec = std::chrono::high_resolution_clock::now() - start_time;

        ///////////////////////////////////////////////////////////////////////////////////////
        /////////////              Controller Code Here !                     /////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        torque_task.setZero(MODEL_DOF);
        TorqueContact.setZero();

        if (dc.gravityMode)
        {
            std::cout << "Task Turned Off,, gravity compensation only !" << std::endl;
            task_switch = false;
            dc.gravityMode = false;
        }
        if (task_switch)
        {
            if (tc.mode == 0) //Pelvis position control
            {
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. 
                */
                wbc_.set_contact(tocabi_, 1, 1);

                int task_number = 6;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task = tocabi_.link_[Pelvis].Jac;

                if (tc.custom_taskgain)
                {
                    tocabi_.link_[Pelvis].pos_p_gain = Vector3d::Ones() * tc.pos_p;
                    tocabi_.link_[Pelvis].pos_d_gain = Vector3d::Ones() * tc.pos_d;
                    tocabi_.link_[Pelvis].rot_p_gain = Vector3d::Ones() * tc.ang_p;
                    tocabi_.link_[Pelvis].rot_d_gain = Vector3d::Ones() * tc.ang_d;
                }

                tocabi_.link_[Pelvis].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[Pelvis].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.f_star = wbc_.getfstar6d(tocabi_, Pelvis);
                wbc_.task_control_torque_QP(tocabi_, tocabi_.J_task, tocabi_.f_star, torque_task);
                torque_grav.setZero();
            }
            else if (tc.mode == 1) //Pelvis position control
            {
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. 
                */
                wbc_.set_contact(tocabi_, 1, 1);

                int task_number = 6;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task = tocabi_.link_[COM_id].Jac;

                if (tc.custom_taskgain)
                {
                    tocabi_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
                    tocabi_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
                    tocabi_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
                    tocabi_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
                }

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star = wbc_.getfstar6d(tocabi_, COM_id);
                cr_mode = 2;
                torque_task = wbc_.task_control_torque_QP2(tocabi_, tocabi_.J_task, tocabi_.f_star);
                torque_grav.setZero();
            }
            else if (tc.mode == 2) //Pelvis position control
            {
                /* 
                For Task Control, NEVER USE tocabi_controller.cpp.
                Use dyros_cc, CustomController for task control. */
                wbc_.set_contact(tocabi_, 1, 1);

                int task_number = 9;
                tocabi_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                tocabi_.f_star.setZero(task_number);

                tocabi_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                tocabi_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                if (tc.custom_taskgain)
                {
                    tocabi_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
                    tocabi_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
                    tocabi_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
                    tocabi_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
                    tocabi_.link_[Upper_Body].pos_p_gain = Vector3d::Ones() * tc.pos_p;
                    tocabi_.link_[Upper_Body].pos_d_gain = Vector3d::Ones() * tc.pos_d;
                    tocabi_.link_[Upper_Body].rot_p_gain = Vector3d::Ones() * tc.ang_p;
                    tocabi_.link_[Upper_Body].rot_d_gain = Vector3d::Ones() * tc.ang_d;
                }

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(tocabi_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.f_star.segment(0, 6) = wbc_.getfstar6d(tocabi_, COM_id);
                tocabi_.f_star.segment(6, 3) = wbc_.getfstar_rot(tocabi_, Upper_Body);
                cr_mode = 2;
                torque_task = wbc_.task_control_torque_QP2(tocabi_, tocabi_.J_task, tocabi_.f_star);
                torque_grav.setZero();
            }
            else if (tc.mode >= 10)
            {
                cr_mode = 2;

                if (tc_command == true)
                {
                    mycontroller.taskCommandToCC(tc);
                    tc_command = false;
                }
                mycontroller.computeSlow();
                /*     if (dc.positionControl)
                {
                    tocabi_.q_desired_ = mycontroller.getControl();
                }*/
            }
        }
        else
        {
            wbc_.set_contact(tocabi_, 1, 1);
            tp[3] = std::chrono::steady_clock::now();
            torque_grav = wbc_.gravity_compensation_torque(tocabi_);
            //torque_grav = wbc_.task_control_torque_QP_gravity(red_);
        }

        tp[4] = std::chrono::steady_clock::now();
        TorqueDesiredLocal = torque_grav + torque_task;

        if (dc.torqueredis)
        {
            dc.torqueredis = false;
            cr_mode++;
            if (cr_mode > 2)
                cr_mode = 0;

            if (cr_mode == 0)
                std::cout << "contact torque redistribution by yslee " << std::endl;
            else if (cr_mode == 1)
                std::cout << "contact torque redistribution by qp " << std::endl;
            else if (cr_mode == 2)
                std::cout << "contact torque redistribution disabled " << std::endl;
        }

        if (cr_mode == 0)
        {
            TorqueContact = wbc_.contact_force_redistribution_torque(tocabi_, TorqueDesiredLocal, fc_redis, fc_ratio);
        }
        else if (cr_mode == 1)
        {
            TorqueContact = wbc_.contact_torque_calc_from_QP(tocabi_, TorqueDesiredLocal);
        }

        tp[5] = std::chrono::steady_clock::now();
        ///////////////////////////////////////////////////////////////////////////////////////
        //////////////////              Controller Code End             ///////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        mtx.lock();
        if (dc.positionControl == false)
        {
            torque_desired = TorqueDesiredLocal + TorqueContact;
        }
        mtx.unlock();

        //wbc_.task_control_torque(J_task,Eigen)
        //wbc_.get_contact_force(TorqueDesiredLocal);
        //tocabi_.ZMP_local = wbc_.GetZMPpos();

        tocabi_.ContactForce = wbc_.get_contact_force(tocabi_, torque_desired);
        tocabi_.ZMP = wbc_.GetZMPpos(tocabi_);
        tocabi_.ZMP_ft = wbc_.GetZMPpos_fromFT(tocabi_);

        //tocabi_.ZMP_eqn_calc(0) = (tocabi_.link_[COM_id].x_traj(0) * 9.8 - tocabi_.com_.pos(2) * tocabi_.link_[COM_id].a_traj(0)) / 9.8;
        tocabi_.ZMP_eqn_calc(0) = (tocabi_.link_[COM_id].x_traj(1) * 9.81 - (tocabi_.com_.pos(2) - tocabi_.link_[Right_Foot].xpos(2) * 0.5 - tocabi_.link_[Left_Foot].xpos(2) * 0.5) * tocabi_.link_[COM_id].a_traj(1)) / 9.81;
        tocabi_.ZMP_eqn_calc(1) = (tocabi_.link_[COM_id].x_traj(1) * 9.81 - (tocabi_.com_.pos(2) - tocabi_.link_[Right_Foot].xpos(2) * 0.5 - tocabi_.link_[Left_Foot].xpos(2) * 0.5) * tocabi_.link_[COM_id].a_traj(1)) / 9.81 + tocabi_.com_.angular_momentum(0) / (tocabi_.com_.mass * 9.81);
        tocabi_.ZMP_eqn_calc(2) = 0.0;

        //pubfromcontroller();

        //std::cout << "ZMP desired : " << tocabi_.ZMP_desired(1) << "\tZMP ft : " << tocabi_.ZMP_ft(1) << "\tZMP error : " << tocabi_.ZMP_error(1) << std::endl;
        //std::cout << "zmp error x : "<< zmp1(0) <<"  y : "<< zmp1(1)<<std::endl;

        //VectorXd tau_coriolis;
        //RigidBodyDynamics::NonlinearEffects(model_,tocabi_.q_virtual_,tocabi_.q_dot_virtual_,tau_coriolis)

        if (shutdown_tocabi_bool)
            break;
        first = false;

        std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - dyn_loop_start;

        est += elapsed_time.count();

        //std::this_thread::sleep_until(dyn_loop_start + dc.dym_timestep);
    }
    std::cout << cyellow << "Dynamics Slow Thread End !" << creset << std::endl;
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

        if (control_time_ != before_time)
            if (dc.mode != "ethercattest")
                pubfromcontroller();

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
                std::cout << dc.tocabi_.ContactForce_FT << std::endl;
            }
        }
        before_time = control_time_;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //std::cout<<control_time_<<"tui test"<<std::endl;
    }

    std::cout << cyellow << "Terminal Thread End !" << creset << std::endl;
}

void TocabiController::getState()
{
    int count = 0;
    if (dc.testmode == false)
    {
        while ((time == dc.time) && (!shutdown_tocabi_bool))
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            count++;
        }
    }
    mtx_dc.lock();

    time = dc.time;
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
    tocabi_.control_time_ = dc.time;
    tocabi_.q_ = dc.q_;
    tocabi_.q_virtual_ = dc.q_virtual_;
    tocabi_.q_dot_ = dc.q_dot_;
    tocabi_.q_dot_virtual_ = dc.q_dot_virtual_;
    tocabi_.q_ddot_virtual_ = dc.q_ddot_virtual_;

    static bool first_run = true;
    if (first_run)
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {

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
    tocabi_.com_ = dc.com_;

    mtx_dc.unlock();
}

void TocabiController::trajectoryplannar()
{
    //wait for
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_from_begin, time_interval;
    std::chrono::microseconds cycletime(1000);
    int cycle_count = 0;

    while (!shutdown_tocabi_bool)
    {
        std::this_thread::sleep_until(t_begin + cycle_count * cycletime);
        cycle_count++;
        std::chrono::high_resolution_clock::time_point t_begin1 = std::chrono::high_resolution_clock::now();
        time_from_begin = (t_begin1 - t_begin);

        if (tc.mode >= 10)
        {
            cr_mode = 2;

            if (tc_command == true)
            {
                mycontroller.taskCommandToCC(tc);
                tc_command = false;
            }

            mycontroller.computePlanner();

            if (dc.positionControl)
            {
                tocabi_.q_desired_ = mycontroller.getControl();
            }
        }
    }
}

void TocabiController::initialize()
{
    torque_desired.setZero();
    set_q_init = true;
}

void TocabiController::ContinuityChecker(double data)
{
}

void TocabiController::ZMPmonitor()
{
}