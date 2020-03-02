#include "tocabi_controller/tocabi_controller.h"
#include "tocabi_controller/terminal.h"
#include "tocabi_controller/wholebody_controller.h"
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

TocabiController::TocabiController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm) : dc(dc_global), s_(sm), d_(dm), tocabi_(dc_global.tocabi_), mycontroller(*(new CustomController(dc_global, dc_global.tocabi_)))
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
    control_time_pre_ = control_time_;

    tc.command_time = control_time_;
    tc.traj_time = msg->time;
    tc.mode = msg->mode;
    tc.task_init = true;

    tc.ratio = msg->ratio;
    tc.angle = msg->angle;
    tc.height = msg->height;

    tc.l_x = msg->l_x;
    tc.l_y = msg->l_y;
    tc.l_z = msg->l_z;
    tc.l_roll = msg->l_roll*DEG2RAD;
    tc.l_pitch = msg->l_pitch*DEG2RAD;
    tc.l_yaw = msg->l_yaw*DEG2RAD;

    tc.r_x = msg->r_x;
    tc.r_y = msg->r_y;
    tc.r_z = msg->r_z;
    tc.r_roll = msg->r_roll*DEG2RAD;
    tc.r_pitch = msg->r_pitch*DEG2RAD;
    tc.r_yaw = msg->r_yaw*DEG2RAD;

    tocabi_.link_[Right_Foot].Set_initpos();
    tocabi_.link_[Left_Foot].Set_initpos();
    tocabi_.link_[Right_Hand].Set_initpos();
    tocabi_.link_[Left_Hand].Set_initpos();
    tocabi_.link_[Pelvis].Set_initpos();
    tocabi_.link_[Upper_Body].Set_initpos();
    tocabi_.link_[COM_id].Set_initpos();

    task_switch = true;

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
    if(msg->walking_enable == 1)
    {
        walkingCallbackOn = true;
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
            { /*
                if (set_q_init)
                {
                    q_desired_ = q_;
                    set_q_init = false;
                }*/
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    torque_desired(i) = Kps[i] * (tocabi_.q_desired_(i) - tocabi_.q_(i)) - Kvs[i] * (tocabi_.q_dot_(i));
                }
            }

            if (tc.mode == 14)
            {
                mycontroller.compute_fast();
                torque_desired = mycontroller.getControl();
            }

            mtx.lock();
            s_.sendCommand(torque_desired, sim_time);
            mtx.unlock();
        }
    }
    std::cout << cyellow << "Dynamics High Thread : End !" << creset << std::endl;
}

void TocabiController::dynamicsThreadLow()
{
    std::cout << "DynamicsThreadLow : READY ?" << std::endl;
    int controller_Hz = 2000;
    ros::Rate r(controller_Hz);
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

    Wholebody_controller wc_(dc, tocabi_);
    Walking_controller walkc_(dc,tocabi_);

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
    Eigen::VectorXd torque_grav, torque_task;
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

    //Control Loop Start
    while ((!shutdown_tocabi_bool))
    {
        static double est;
        std::chrono::high_resolution_clock::time_point dyn_loop_start = std::chrono::high_resolution_clock::now();
        dynthread_cnt++;
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
            dynthread_cnt = 0;
            est = 0;
        }
        getState(); //link data override

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

        wc_.init(tocabi_);
        wc_.update(tocabi_);

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
            if (tc.mode == 0 && tc.walking_enable == 0) //Pelvis position control
            {   
                wc_.set_contact(tocabi_, 1, 1);

                //torque_grav = wc_.gravity_compensation_torque(tocabi_);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[Pelvis].Jac;

                tocabi_.link_[Pelvis].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[Pelvis].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //tocabi_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                f_star = wc_.getfstar6d(tocabi_, Pelvis);
                //torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);

                cr_mode = 2;
                //torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 1) //COM position control
            {
                wc_.set_contact(tocabi_, 1, 1);

                //torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[COM_id].Jac;
                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac_COM_p;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //tocabi_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                f_star = wc_.getfstar6d(tocabi_, COM_id);
                //std::cout<<"f_star : "<<std::endl<<tocabi_.link_[COM_id].f_star<<std::endl;
                //torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);

                torque_grav.setZero();
                //std::chrono::high_resolution_clock::time_point stc = std::chrono::high_resolution_clock::now();
                torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);

                //std::chrono::duration<double> slc = std::chrono::high_resolution_clock::now() - stc;
                //std::cout << "time spend : " << slc.count() << std::endl;
                cr_mode = 2;

                /*
                MatrixXd lambda_;
                std::cout << "####################" << std::endl;
                lambda_ = (J_task * tocabi_.A_matrix_inverse * J_task.transpose()).inverse();

                MatrixXd jtinv_;
                jtinv_ = (tocabi_.A_matrix_inverse * J_task.transpose() * lambda_).transpose();
                jtinv_ = lambda_ * J_task * tocabi_.A_matrix_inverse;

                std::cout << "####################" << std::endl;
                std::cout << "operational space : " << std::endl
                        << lambda_ * f_star + jtinv_ * tocabi_.G << std::endl;
                std::cout << "contact consistant : " << std::endl
                        << tocabi_.lambda * f_star + tocabi_.J_task_inv_T * tocabi_.G << std::endl;

                //torque_task = wc_.task_control_torque(J_task, f_star);
                f_star.segment(0, 3) = tocabi_.link_[COM_id].a_traj;

                TorqueDesiredLocal = wc_.task_control_torque(tocabi_, J_task, f_star) + torque_grav;

                torque_grav.setZero();
                //TorqueDesiredLocal = wc_.task_control_torque_QP(tocabi_, J_task, f_star);
                cr_mode = 2;

                tocabi_.ContactForce = wc_.get_contact_force(tocabi_, TorqueDesiredLocal);
                tocabi_.ZMP_local = wc_.GetZMPpos(tocabi_);

                tocabi_.ZMP_ft = wc_.GetZMPpos_fromFT(tocabi_);*/

                //std::cout << "contact force from controller :::: " << std::endl
                //          << wc_.get_contact_force(tocabi_, torque_task) << std::endl;
            }
            else if (tc.mode == 2) //COM to Left foot, then switch double support to single support
            {
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    wc_.set_contact(tocabi_, 1, 1);
                    if (tc.ratio >= 0.5)
                    {
                        tocabi_.link_[COM_id].x_desired = tocabi_.link_[Left_Foot].xpos;
                        tocabi_.link_[COM_id].x_desired(2) = tc.height + tocabi_.link_[Left_Foot].xpos(2);
                    }
                    else if (tc.ratio < 0.5)
                    {
                        tocabi_.link_[COM_id].x_desired = tocabi_.link_[Right_Foot].xpos;
                        tocabi_.link_[COM_id].x_desired(2) = tc.height + tocabi_.link_[Right_Foot].xpos(2);
                    }
                }
                else if (tc.ratio < 0.5)
                {
                    wc_.set_contact(tocabi_, 0, 1);
                }
                else if (tc.ratio > 0.5)
                {
                    wc_.set_contact(tocabi_, 1, 0);
                }

                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[COM_id].Jac;

                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();

                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                f_star = wc_.getfstar6d(tocabi_, COM_id);

                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
            }
            else if (tc.mode == 3) //COM to Left foot, then switch double support to single support while holding com rotation.
            {
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    wc_.set_contact(tocabi_, 1, 1);
                }
                else
                {
                    wc_.set_contact(tocabi_, 1, 0);
                }

                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[COM_id].Jac;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[Left_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height + tocabi_.link_[Left_Foot].xpos(2);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();

                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                f_star = wc_.getfstar6d(tocabi_, COM_id);

                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
            }
            else if (tc.mode == 4) //left foot controller
            {
                //if(tocabi_.contact_part[0].)
                int task_link;
                if (tocabi_.ee_[0].contact)
                {
                    wc_.set_contact(tocabi_, 1, 0);
                    task_link = Right_Foot;
                }
                else if (tocabi_.ee_[1].contact)
                {
                    wc_.set_contact(tocabi_, 0, 1);
                    task_link = Left_Foot;
                }

                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                task_number = 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);
                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;

                J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[task_link].Jac;
                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();

                tocabi_.link_[task_link].x_desired(0) = tocabi_.link_[task_link].x_init(0) + tc.ratio;
                tocabi_.link_[task_link].x_desired(1) = tocabi_.link_[task_link].x_init(1);
                tocabi_.link_[task_link].x_desired(2) = tocabi_.link_[task_link].x_init(2) + tc.height;
                tocabi_.link_[task_link].rot_desired = Matrix3d::Identity();

                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[task_link].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[task_link].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                f_star.segment(6, 6) = wc_.getfstar6d(tocabi_, task_link);

                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
                //tocabi_.link_[Right_Foot].x_desired = tc.
            }
            else if (tc.mode == 5)
            {

                if (tocabi_.ee_[0].contact && tocabi_.ee_[1].contact)
                {
                    tocabi_.ContactForce = tocabi_.ContactForce_FT;
                    tocabi_.ZMP_ft = wc_.GetZMPpos(tocabi_);
                }
                else if (tocabi_.ee_[0].contact)
                {
                    tocabi_.ContactForce = tocabi_.ContactForce_FT.segment(0, 6);
                    tocabi_.ZMP_ft = wc_.GetZMPpos(tocabi_);
                }
                else if (tocabi_.ee_[1].contact)
                {
                    tocabi_.ContactForce = tocabi_.ContactForce_FT.segment(6, 6);
                    tocabi_.ZMP_ft = wc_.GetZMPpos(tocabi_);
                }

                tocabi_.ZMP_error = tocabi_.ZMP_desired - tocabi_.ZMP_ft;

                wc_.set_contact(tocabi_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                double r_dis, l_dis;

                r_dis = abs(tocabi_.link_[COM_id].xpos(1) - tocabi_.link_[Right_Foot].xpos(1));
                l_dis = abs(tocabi_.link_[COM_id].xpos(1) - tocabi_.link_[Left_Foot].xpos(1));

                bool transition;
                double t = 1.0;
                static double com_height = tocabi_.com_.pos(2) - (tocabi_.link_[Right_Foot].xpos(2) + tocabi_.link_[Left_Foot].xpos(2)) * 0.5;

                double tc = sqrt(com_height / 9.81);
                static double st;
                double tf = 1.0;
                static int left_c = 1;
                static int right_c = 1;
                static int left_t = 0;
                static int right_t = 0;
                static int step = 0;
                static double tn;

                double d_t = 1.0;
                double t_gain = 0.05;
                double freeze_tick = 10;

                int transition_step = 100;
                int transition_step2 = 200;

                double f_star_y;

                if (l_dis < r_dis)
                {
                    //tocabi_.ZMP_desired(1) = tocabi_.link_[Left_Foot].xpos(1);
                    tn = control_time_ - st;
                    if (right_c == 1)
                    {
                        st = control_time_;
                    }
                    if (step > transition_step2)
                    {
                        tocabi_.ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh((tf - tn) / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh((tf - tn) / tc)) / (cosh((tf - tn) / tc) - 1);
                        std::cout << "tocabi_.ZMP_desired : " << tocabi_.ZMP_desired(1) << std::endl;
                    }

                    if (right_c == 1)
                    {
                        if (step <= transition_step2)
                        {
                            tocabi_.ZMP_desired(1) = tocabi_.link_[Left_Foot].xpos(1);
                        }
                        else
                        {

                            std::cout << "transition!" << std::endl;
                        }
                        if (step > transition_step)
                        {
                            tocabi_.ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf / tc)) / (cosh(tf / tc) - 1);
                        }

                        right_c = 0;
                        tocabi_.ZMP_error(1) = 0.0;
                        std::cout << step << " left close : " << tn << " des zmp : " << tocabi_.ZMP_desired(1) << "com p : " << tocabi_.link_[COM_id].xpos(1) << " com v : " << tocabi_.link_[COM_id].v(1) << std::endl;
                        tocabi_.ZMP_command(1) = tocabi_.ZMP_desired(1);
                        right_t = 0;
                        step++;
                        if ((step > 3) && (tn > d_t))
                        {
                            //std::cout<< (tocabi_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1) - (tocabi_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1)<<std::endl;
                            //tocabi_.ZMP_mod(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1) - (tocabi_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1);
                            tocabi_.ZMP_mod = tocabi_.ZMP_desired * (tn - d_t) * t_gain;
                            tocabi_.ZMP_desired(1) = tocabi_.ZMP_desired(1) + tocabi_.ZMP_mod(1);
                            tocabi_.ZMP_command(1) = tocabi_.ZMP_desired(1);
                            std::cout << "lc add : " << tocabi_.ZMP_mod(1) << std::endl;
                        }
                        else
                        {
                            tocabi_.ZMP_mod(1) = 0;
                        }
                    }

                    if (right_t < freeze_tick)
                    {
                        tocabi_.ZMP_error(1) = 0.0;
                        right_t++;
                    }

                    //tf = 2.0 - control_time_ + st;
                    //if (tf < 0)
                    //    tf = 0;
                    //ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf / tc) - 0.0) / (cosh(tf / tc) - 1);

                    //tf = 2.0 - control_time_ + st;
                    //ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf/tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf/tc) -0.0)/(cosh(tf/tc)-1);

                    left_c = 1;
                }
                else if (r_dis < l_dis)
                {
                    tn = control_time_ - st;
                    //tocabi_.ZMP_desired(1) = tocabi_.link_[Right_Foot].xpos(1);
                    if (left_c == 1)
                    {
                        st = control_time_;
                    }
                    if (step > transition_step2)
                    {
                        tocabi_.ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh((tf - tn) / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh((tf - tn) / tc)) / (cosh((tf - tn) / tc) - 1);
                        std::cout << "tocabi_.ZMP_desired : " << tocabi_.ZMP_desired(1) << std::endl;
                    }

                    if (left_c == 1)
                    {
                        if (step <= transition_step2)
                        {
                            tocabi_.ZMP_desired(1) = tocabi_.link_[Right_Foot].xpos(1);
                        }
                        else
                        {

                            std::cout << "transition!" << std::endl;
                        }

                        if (step > transition_step)
                            tocabi_.ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf / tc)) / (cosh(tf / tc) - 1);

                        left_c = 0;
                        tocabi_.ZMP_error(1) = 0.0;
                        std::cout << step << " right close : " << tn << " des zmp : " << tocabi_.ZMP_desired(1) << "com p : " << tocabi_.link_[COM_id].xpos(1) << " com v : " << tocabi_.link_[COM_id].v(1) << std::endl;
                        tocabi_.ZMP_command(1) = tocabi_.ZMP_desired(1);
                        left_t = 0;
                        step++;
                        if ((step > 3) && (tn > d_t))
                        {
                            //std::cout<< (tocabi_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1) - (tocabi_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1)<<std::endl;
                            //tocabi_.ZMP_mod(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1) - (tocabi_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1);
                            //tocabi_.ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf / tc) - 0.0) / (cosh(tf / tc) - 1);
                            tocabi_.ZMP_mod = tocabi_.ZMP_desired * (tn - d_t) * t_gain;
                            std::cout << "rc add : " << tocabi_.ZMP_mod(1) << std::endl;
                            tocabi_.ZMP_desired(1) = tocabi_.ZMP_desired(1) + tocabi_.ZMP_mod(1) * 6;
                            tocabi_.ZMP_command(1) = tocabi_.ZMP_desired(1);
                        }
                        else
                        {
                            tocabi_.ZMP_mod(1) = 0;
                        }
                        //tf = 2.0 - control_time_ + st;
                    }

                    if (left_t < freeze_tick)
                    {
                        tocabi_.ZMP_error(1) = 0.0;
                        left_t++;
                    }
                    //tf = 2.0 - control_time_ + st;
                    //if (tf < 0)
                    //    tf = 0;
                    //tocabi_.ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf / tc)) / (cosh(tf / tc) - 1);
                    //ZMP_desired(1) = (tocabi_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * tocabi_.link_[COM_id].v(1) * sinh(tf / tc) - 0.0) / (cosh(tf / tc) - 1);

                    right_c = 1;
                }
                tocabi_.ZMP_desired(0) = tocabi_.link_[COM_id].xpos(0);

                //wc_.set_zmp_control(tocabi_, tocabi_.ZMP_desired.segment(0, 2), 1.0);

                tocabi_.ZMP_command = tocabi_.ZMP_command + 0.05 * tocabi_.ZMP_error; //+ rk_.ZMP_mod;
                f_star_y = 9.81 / (tocabi_.com_.pos(2) - tocabi_.link_[Right_Foot].xpos(2) * 0.5 - tocabi_.link_[Left_Foot].xpos(2) * 0.5) * (tocabi_.com_.pos(1) - tocabi_.ZMP_command(1));

                f_star = wc_.getfstar6d(tocabi_, COM_id);
                f_star(1) = f_star_y;
                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
                VectorXd cf_pre = wc_.get_contact_force(tocabi_, torque_task + torque_grav);

                //tocabi_.ZMP_desired.segment(0, 2);
                tocabi_.ZMP_desired(2) = 0.0;
            }
            else if (tc.mode == 6)
            {
                static int loop_temp;
                static int loop_;
                static bool cgen_init;
                static bool loop_cnged;
                static bool walking_init;
                static double foot_height;
                static bool zmperror_reset;

                double footstep_y_length = 0.1024;
                //QP_switch = true;
                tocabi_.ee_[0].contact = true;
                tocabi_.ee_[1].contact = true;

                //right_foot_contact_ = true;
                //left_foot_contact_ = true;
                double time_segment_origin = tc.traj_time;
                static double time_segment = tc.traj_time;
                static double loop_start_time;
                static double loop_end_time;

                if (tc.task_init)
                {
                    std::cout << "Control init!!!!!!!!!!!!!!!" << std::endl;
                    cgen_init = true;
                    loop_cnged = false;
                    walking_init = true;
                    foot_height = (tocabi_.link_[Right_Foot].xpos(2) + tocabi_.link_[Left_Foot].xpos(2)) / 2.0;
                    zmperror_reset = true;
                    time_segment = tc.traj_time;
                    loop_start_time = 0.0;
                    loop_end_time = 0.0;
                    tc.task_init = false;
                }

                double step_length = tc.ratio;

                double task_time = control_time_ - tc.command_time;

                loop_temp = loop_;
                loop_ = (int)(task_time / time_segment);
                double loop_time = task_time - (double)loop_ * time_segment;

                double lr_st, lr_mt, lr_et;
                lr_st = time_segment / 8.0;
                lr_mt = time_segment / 8.0 * 4.0;
                lr_et = time_segment / 8.0 * 7.0;

                if ((double)loop_ > 0.1)
                {
                    if (loop_ % 2)
                    {
                        if ((loop_time < lr_et))
                        {

                            tocabi_.ee_[1].contact = false;
                            tocabi_.ee_[0].contact = true;
                        }
                        else
                        {
                            tocabi_.ee_[1].contact = true;
                            tocabi_.ee_[0].contact = true;
                        }
                    }
                    else
                    {
                        if ((loop_time < lr_et))
                        {
                            tocabi_.ee_[1].contact = true;
                            tocabi_.ee_[0].contact = false;
                        }
                        else
                        {
                            tocabi_.ee_[1].contact = true;
                            tocabi_.ee_[0].contact = true;
                        }
                    }
                }

                //(model_.link_[model_.COM_id].x_init - zmp)*cosh(loop_time/time_segment)+time_segment * model_.link_[model_.COM_id]
                task_desired.setZero();
                task_desired(0) = tocabi_.link_[COM_id].x_init(0);
                task_desired(2) = tc.height + foot_height;

                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, task_desired);

                tocabi_.ZMP_error = tocabi_.ZMP_desired - tocabi_.ZMP_ft;

                // model_.link_[model_.COM_id].x_traj.segment(0, 2) = (cx_init - zmp) * cosh(loop_time * w_) + cv_init * sinh(loop_time * w_) / w_ + zmp;

                // model_.link_[model_.COM_id].v_traj.segment(0, 2) = (cx_init - zmp) * w_ * sinh(loop_time * w_) + cv_init * cosh(loop_time * w_);

                // std::cout << " xtraj : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].x_traj.segment(0, 2) << std::endl;
                //std::cout << "vtrah : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].v_traj.segment(0, 2) << std::endl;

                if (tocabi_.ee_[1].contact && tocabi_.ee_[0].contact)
                {
                    walking_init = true;
                    task_number = 6;
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);
                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;

                    wc_.set_contact(tocabi_, 1, 1);
                    torque_grav = wc_.gravity_compensation_torque(tocabi_);

                    tocabi_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                }
                else if (tocabi_.ee_[0].contact)
                {
                    if (walking_init)
                    {
                        tocabi_.link_[Right_Foot].x_init = tocabi_.link_[Right_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 12;

                    wc_.set_contact(tocabi_, 1, 0);
                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = tocabi_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = tocabi_.link_[Right_Foot].Jac;

                    torque_grav = wc_.gravity_compensation_torque(tocabi_);

                    tocabi_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    Vector3d lf_desired;
                    lf_desired = tocabi_.link_[Right_Foot].x_init;
                    lf_desired(1) = -footstep_y_length;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    tocabi_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        tocabi_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.04, tocabi_.link_[Right_Foot].x_init(0), 0, 0, tocabi_.link_[Left_Foot].xpos(0) + step_length, 0, 0);
                    tocabi_.link_[Right_Foot].x_traj(0) = quintic(0);
                    tocabi_.link_[Right_Foot].v_traj(0) = quintic(1);

                    tocabi_.link_[Right_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                    f_star.segment(6, 6) = wc_.getfstar6d(tocabi_, Right_Foot);
                }
                else if (tocabi_.ee_[1].contact) // rightfoot contact
                {

                    Vector3d lf_desired;
                    if (walking_init)
                    {
                        tocabi_.link_[Left_Foot].x_init = tocabi_.link_[Left_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 12;
                    wc_.set_contact(tocabi_, 0, 1);

                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = tocabi_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = tocabi_.link_[Left_Foot].Jac;

                    torque_grav = wc_.gravity_compensation_torque(tocabi_);

                    tocabi_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);
                    //model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

                    lf_desired = tocabi_.link_[Left_Foot].x_init;

                    lf_desired(1) = footstep_y_length;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    tocabi_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        tocabi_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.04, tocabi_.link_[Left_Foot].x_init(0), 0, 0, tocabi_.link_[Right_Foot].xpos(0) + step_length, 0, 0);
                    tocabi_.link_[Left_Foot].x_traj(0) = quintic(0);
                    tocabi_.link_[Left_Foot].v_traj(0) = quintic(1);

                    tocabi_.link_[Left_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                    f_star.segment(6, 6) = wc_.getfstar6d(tocabi_, Left_Foot);
                }

                Vector2d cp_current = tocabi_.com_.CP;
                double w_ = sqrt(9.81 / tocabi_.com_.pos(2));
                double b_ = exp(w_ * (time_segment - loop_time));

                Vector2d desired_cp;
                Vector2d right_cp, left_cp, cp_mod;
                //cp_mod << -0.02, -0.00747;
                //right_cp << -0.04, -0.095;
                //left_cp << -0.04, 0.095;
                cp_mod << -0.0, -0.00747;
                right_cp << 0.02, -0.09;
                left_cp << 0.02, 0.09;

                if (loop_ - loop_temp)
                {
                    loop_cnged = true;
                    cgen_init = true;
                }
                double st_temp;

                if (loop_ % 2)
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;

                    desired_cp = right_cp;
                    desired_cp(0) = right_cp(0) + ((double)loop_) * st_temp + tocabi_.link_[COM_id].x_init(0);
                }
                else
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;
                    desired_cp = left_cp;
                    desired_cp(0) = left_cp(0) + ((double)loop_) * st_temp + tocabi_.link_[COM_id].x_init(0);
                }

                Vector2d zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * tocabi_.com_.CP;

                if (cgen_init)
                {
                    tc.command_time = tc.command_time + time_segment - time_segment_origin;
                    time_segment = time_segment_origin;
                    std::cout << "###############################################" << std::endl;
                    std::cout << "loop : " << loop_ << " loop time : " << loop_time << std::endl;
                    //cx_init = model_.com_.pos.segment(0, 2);
                    //cv_init = model_.com_.vel.segment(0, 2);
                    std::cout << "desired cp   x : " << desired_cp(0) << "  y : " << desired_cp(1) << std::endl;
                    std::cout << "zmp gen   x : " << zmp(0) << "  y : " << zmp(1) << std::endl;
                    zmperror_reset = true;
                    //std::cout << "c CP" << std::endl;
                    //std::cout << model_.com_.CP << std::endl;

                    //zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * model_.com_.CP;
                    //cgen_init = false;
                }

                /*
                if (zmp(1) > 0.11)
                {
                    zmp(1) = 0.11;
                }
                if (zmp(1) < -0.11)
                {
                    zmp(1) = -0.11;
                }*/

                double y_margin, x_margin;
                double over_ratio;
                y_margin = 0.04;
                x_margin = 0.11;

                if (loop_ > 0)
                {
                    if (loop_ % 2)
                    {
                        if (zmp(1) > (tocabi_.link_[Left_Foot].xpos_contact(1) + y_margin))
                        {
                            //std::cout << "ZMP regulate active : " << tocabi_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                            zmp(1) = tocabi_.link_[Left_Foot].xpos_contact(1) + y_margin;

                            //std::cout << loop_ << "Left ZMP regulate active : " << tocabi_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }
                        if (zmp(1) < (tocabi_.link_[Left_Foot].xpos_contact(1) - y_margin))
                        {
                            //std::cout << "ZMP regulate active : " << tocabi_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                            zmp(1) = tocabi_.link_[Left_Foot].xpos_contact(1) - y_margin;
                            //std::cout << loop_ << "Left ZMP regulate active : " << tocabi_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }

                        if (zmp(0) < (tocabi_.link_[Left_Foot].xpos_contact(0) - x_margin))
                        {
                            zmp(0) = (tocabi_.link_[Left_Foot].xpos_contact(0) - x_margin);
                        }
                        if (zmp(0) > (tocabi_.link_[Left_Foot].xpos_contact(0) + x_margin))
                        {
                            zmp(0) = (tocabi_.link_[Left_Foot].xpos_contact(0) + x_margin);
                        }
                    }
                    else
                    {
                        if (zmp(1) > (tocabi_.link_[Right_Foot].xpos_contact(1) + y_margin))
                        {
                            zmp(1) = tocabi_.link_[Right_Foot].xpos_contact(1) + y_margin;
                            //std::cout << loop_ << "Right ZMP regulate active : " << tocabi_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }
                        if (zmp(1) < (tocabi_.link_[Right_Foot].xpos_contact(1) - y_margin))
                        {
                            zmp(1) = tocabi_.link_[Right_Foot].xpos_contact(1) - y_margin;
                            //std::cout << loop_ << "Right ZMP regulate active : " << tocabi_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }

                        if (zmp(0) < (tocabi_.link_[Right_Foot].xpos_contact(0) - x_margin))
                        {
                            zmp(0) = (tocabi_.link_[Right_Foot].xpos_contact(0) - x_margin);
                        }
                        if (zmp(0) > (tocabi_.link_[Right_Foot].xpos_contact(0) + x_margin))
                        {
                            zmp(0) = (tocabi_.link_[Right_Foot].xpos_contact(0) + x_margin);
                        }
                    }
                }

                double extend_t = 1 / w_ * log((desired_cp(1) - zmp(1)) / (cp_current(1) - zmp(1))) - time_segment + loop_time;
                if (extend_t > 0.0001)
                {
                    std::cout << extend_t << std::endl;
                    time_segment = time_segment + extend_t;
                }
                // est_cp_ = b_ * (model_.com_.pos.segment(0, 2) + model_.com_.vel.segment(0, 2) / w_) + (1 - b_) * zmp;
                // std::cout << "estimated_cp" << std::endl;
                // std::cout << est_cp_ << std::endl;
                if (cgen_init)
                {
                    std::cout << "1 ZMP : " << zmp(1) << std::endl;
                    cgen_init = false;
                }

                tocabi_.ZMP_desired(0) = zmp(0);
                tocabi_.ZMP_desired(1) = zmp(1);

                wc_.set_zmp_control(tocabi_, zmp, 1.0);
                //wc_.set_zmp_feedback_control(red_, zmperror_reset);
                //MatrixXd damping_matrix;
                //damping_matrix.setIdentity(total_dof_, total_dof_);

                // //torque_dc_.segment(0, 12) = q_dot_.segment(0, 12) * 2.0;
                // std::cout << "zmp_des" << std::endl;
                // std::cout << zmp << std::endl;
                //std::cout << "zmp_cur" << std::endl;
                //std::cout << model_.com_.ZMP << std::endl;
                //std::cout << "Sensor ZMP" << std::endl;
                //std::cout << body_zmp_ << std::endl;

                //single support test at tc_
                // single support , 0.1 ~ 0.9s foot up -> down just for 4cm?
                // at loop_ = 0( first phase)
                // at loop_ = 1, zmp at right foot.
                // at each loop, 0~0.1 double support 0.1~ 0.9 singlesupport 0.9~1.0 double support
                //

                tocabi_.ZMP_ft = wc_.GetZMPpos_fromFT(tocabi_);
                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
            }
                    //////////////////////////////////// Arm Control /////////////////////////////////////////
            else if (tc.mode == 7)
            {
                const int arm_task_number = 6;
                const int arm_dof = 8;
                ////////// CoM Control //////////////////////
                wc_.set_contact(tocabi_, 1, 1);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[COM_id].Jac;
                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac_COM_p;
                J_task.block(0, 21, 3, arm_dof).setZero(); // Exclude Left Arm Jacobian
                J_task.block(0, 31, 3, arm_dof).setZero(); // Exclude Right Arm Jacobian

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                f_star = wc_.getfstar6d(tocabi_, COM_id);
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);

                ///////// Jacobian based ik arm controller (Daegyu, Donghyeon)/////////////////
                Eigen::Matrix<double, 2*arm_task_number, 2*arm_dof> J_task_Arm;
                J_task_Arm.setZero();
                J_task_Arm.block(0, 0, arm_task_number, arm_dof) = tocabi_.link_[Left_Hand].Jac.block(0,21,arm_task_number,arm_dof);
                J_task_Arm.block(arm_task_number, arm_dof, arm_task_number, arm_dof) = tocabi_.link_[Right_Hand].Jac.block(0,31,arm_task_number,arm_dof);
                Eigen::Matrix<double, 2*arm_dof, 2*arm_task_number> J_task_inv;
                J_task_inv = DyrosMath::pinv_SVD(J_task_Arm);

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                
                Eigen::Vector12d x_dot_desired;
                Eigen::Vector6d error_v;
                Eigen::Vector6d error_w;                
                Eigen::Vector6d k_pos;
                Eigen::Vector6d k_rot;

                for (int i = 0; i<6; i++)
                {
                    k_pos(i) = 10;
                    k_rot(i) = 4;
                }

                error_v.segment<3>(0) = tocabi_.link_[Left_Hand].x_traj -  tocabi_.link_[Left_Hand].xpos;
                error_v.segment<3>(3) = tocabi_.link_[Right_Hand].x_traj -  tocabi_.link_[Right_Hand].xpos;

                error_w.segment<3>(0) = -DyrosMath::getPhi(tocabi_.link_[Left_Hand].Rotm, tocabi_.link_[Left_Hand].r_traj);
                error_w.segment<3>(3) = -DyrosMath::getPhi(tocabi_.link_[Right_Hand].Rotm, tocabi_.link_[Right_Hand].r_traj);

                for(int i = 0; i<3; i++)
                {
                    x_dot_desired(i) = tocabi_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
                    x_dot_desired(i+3) = tocabi_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
                    x_dot_desired(i+6) = tocabi_.link_[Right_Hand].v_traj(i) + k_pos(i+3)*error_v(i+3); // linear velocity
                    x_dot_desired(i+9) = tocabi_.link_[Right_Hand].w_traj(i) + k_rot(i+3)*error_w(i+3);
                }
                VectorXd q_dot_arm;
                q_dot_arm = J_task_inv*x_dot_desired;
                for (int i=0; i<arm_dof; i++)
                {
                    q_dot_desired_(15+i) = q_dot_arm(i);
                    q_dot_desired_(25+i) = q_dot_arm(i+arm_dof);
                }
                q_desired_.segment<8>(15) = tocabi_.q_.segment<8>(15) + q_dot_desired_.segment<8>(15)*(control_time_ - control_time_pre_);
                q_desired_.segment<8>(25) = tocabi_.q_.segment<8>(25) + q_dot_desired_.segment<8>(25)*(control_time_ - control_time_pre_);

                Eigen::MatrixXd kp(8,1);
                Eigen::MatrixXd kv(8,1);
                
                for(int i = 0; i<8; i++)
                {
                    kp(i) = 9;
                    kv(i) = 6;
                }

                for(int i = 0; i<8; i++)
                {
                    torque_task(i+15) += kp(i)*(q_desired_(i+15) - tocabi_.q_(i+15)) + kv(i)*(q_dot_desired_(i+15) - tocabi_.q_dot_(i+15));
                    torque_task(i+25) += kp(i)*(q_desired_(i+25) - tocabi_.q_(i+25)) + kv(i)*(q_dot_desired_(i+25) - tocabi_.q_dot_(i+25));
                }           
                control_time_pre_ = control_time_;
            }
            else if (tc.mode == 8)
            {
                wc_.set_contact(tocabi_, 1, 1);

                task_number = 6 + 6 + 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;
                J_task.block(12, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                tocabi_.link_[COM_id].rot_desired = tocabi_.link_[COM_id].rot_init;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                f_star.segment(6, 6) = wc_.getfstar6d(tocabi_, Left_Hand);
                f_star.segment(12, 6) = wc_.getfstar6d(tocabi_, Right_Hand);

                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);
            }
            else if (tc.mode == 9)
            {
                const int arm_task_number = 6;
                const int arm_dof = 8;
                ///////// Jacobian based ik arm controller (Daegyu, Donghyeon)/////////////////
                Eigen::Matrix<double, 2*arm_task_number, 2*arm_dof> J_task_Arm;
                J_task_Arm.setZero();
                J_task_Arm.block(0, 0, arm_task_number, arm_dof) = tocabi_.link_[Left_Hand].Jac.block(0,21,arm_task_number,arm_dof);
                J_task_Arm.block(arm_task_number, arm_dof, arm_task_number, arm_dof) = tocabi_.link_[Right_Hand].Jac.block(0,31,arm_task_number,arm_dof);
                Eigen::Matrix<double, 2*arm_dof, 2*arm_task_number> J_task_inv;
                J_task_inv = DyrosMath::pinv_SVD(J_task_Arm);

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                
                Eigen::Vector12d x_dot_desired;
                Eigen::Vector6d error_v;
                Eigen::Vector6d error_w;                
                Eigen::Vector6d k_pos;
                Eigen::Vector6d k_rot;

                for (int i = 0; i<6; i++)
                {
                    k_pos(i) = 10;
                    k_rot(i) = 4;
                }

                error_v.segment<3>(0) = tocabi_.link_[Left_Hand].x_traj -  tocabi_.link_[Left_Hand].xpos;
                error_v.segment<3>(3) = tocabi_.link_[Right_Hand].x_traj -  tocabi_.link_[Right_Hand].xpos;

                error_w.segment<3>(0) = -DyrosMath::getPhi(tocabi_.link_[Left_Hand].Rotm, tocabi_.link_[Left_Hand].r_traj);
                error_w.segment<3>(3) = -DyrosMath::getPhi(tocabi_.link_[Right_Hand].Rotm, tocabi_.link_[Right_Hand].r_traj);

                for(int i = 0; i<3; i++)
                {
                    x_dot_desired(i) = tocabi_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
                    x_dot_desired(i+3) = tocabi_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
                    x_dot_desired(i+6) = tocabi_.link_[Right_Hand].v_traj(i) + k_pos(i+3)*error_v(i+3); // linear velocity
                    x_dot_desired(i+9) = tocabi_.link_[Right_Hand].w_traj(i) + k_rot(i+3)*error_w(i+3);
                }
                VectorXd q_dot_arm;
                q_dot_arm = J_task_inv*x_dot_desired;
                for (int i=0; i<arm_dof; i++)
                {
                    q_dot_desired_(15+i) = q_dot_arm(i);
                    q_dot_desired_(25+i) = q_dot_arm(i+arm_dof);
                }
                q_desired_.segment<8>(15) = tocabi_.q_.segment<8>(15) + q_dot_desired_.segment<8>(15)*(control_time_ - control_time_pre_);
                q_desired_.segment<8>(25) = tocabi_.q_.segment<8>(25) + q_dot_desired_.segment<8>(25)*(control_time_ - control_time_pre_);

                Eigen::MatrixXd kp(8,1);
                Eigen::MatrixXd kv(8,1);
                
                for(int i = 0; i<8; i++)
                {
                    kp(i) = 9;
                    kv(i) = 6;
                }
                torque_task.setZero(MODEL_DOF);
                for(int i = 0; i<8; i++)
                {
                    torque_task(i+15) = kp(i)*(q_desired_(i+15) - tocabi_.q_(i+15)) + kv(i)*(q_dot_desired_(i+15) - tocabi_.q_dot_(i+15));
                    torque_task(i+25) = kp(i)*(q_desired_(i+25) - tocabi_.q_(i+25)) + kv(i)*(q_dot_desired_(i+25) - tocabi_.q_dot_(i+25));
                }           
                control_time_pre_ = control_time_;
            }
            else if(tc.mode == 0 && tc.walking_enable == 1)
            {
                if(walkingCallbackOn == true)
                {
                    walkc_.getUiWalkingParameter(wtc, controller_Hz);
                    
                    walkingCallbackOn = false;
                }
                walkc_.walkingCompute();
            }
        }
        else
        {
            wc_.set_contact(tocabi_, 1, 1);
            torque_grav = wc_.gravity_compensation_torque(tocabi_);
            //torque_grav = wc_.task_control_torque_QP_gravity(red_);
        }

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
            TorqueContact = wc_.contact_force_redistribution_torque(tocabi_, TorqueDesiredLocal, fc_redis, fc_ratio);
        }
        else if (cr_mode == 1)
        {
            TorqueContact = wc_.contact_torque_calc_from_QP(tocabi_, TorqueDesiredLocal);
        }

        //acceleration_estimated = (A_matrix_inverse * N_C * Slc_k_T * (TorqueDesiredLocal - torque_grav)).segment(6, MODEL_DOF);
        //acceleration_observed = q_dot_ - q_dot_before_;
        //q_dot_before_ = q_dot_;
        //std::cout << "acceleration_observed : " << std::endl;
        //std::cout << acceleration_observed << std::endl;
        //acceleration_differance = acceleration_observed - acceleration_estimated_before;
        //acceleration_estimated_before = acceleration_estimated;

        ///////////////////////////////////////////////////////////////////////////////////////
        //////////////////              Controller Code End             ///////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        mtx.lock();
        torque_desired = TorqueDesiredLocal + TorqueContact;
        //dc.accel_dif = acceleration_differance;
        //dc.accel_obsrvd = acceleration_observed;
        mtx.unlock();

        //wc_.task_control_torque(J_task,Eigen)
        //wc_.get_contact_force(TorqueDesiredLocal);
        //tocabi_.ZMP_local = wc_.GetZMPpos();

        tocabi_.ContactForce = wc_.get_contact_force(tocabi_, torque_desired);
        tocabi_.ZMP = wc_.GetZMPpos(tocabi_);

        tocabi_.ZMP_ft = wc_.GetZMPpos_fromFT(tocabi_);

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
    tocabi_.com_ = dc.com_;

    mtx_dc.unlock();
}

void TocabiController::initialize()
{
    torque_desired.setZero();
}

void TocabiController::ContinuityChecker(double data)
{
}

void TocabiController::ZMPmonitor()
{
}