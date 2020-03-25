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
    arm_task_command = dc.nh.subscribe("/tocabi/armtaskcommand", 100, &TocabiController::ArmTaskCommandCallback, this);
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
    tc.command_time = control_time_;
    tc.traj_time = msg->time;
    tc.ratio = msg->ratio;
    tc.angle = msg->angle;
    tc.height = msg->height;
    tc.mode = msg->mode;
    tc.task_init = true;

    atc.mode = -1; //turn off atc

    tocabi_.link_[Right_Foot].Set_initpos();
    tocabi_.link_[Left_Foot].Set_initpos();
    tocabi_.link_[Right_Hand].Set_initpos();
    tocabi_.link_[Left_Hand].Set_initpos();
    tocabi_.link_[Pelvis].Set_initpos();
    tocabi_.link_[Upper_Body].Set_initpos();
    tocabi_.link_[COM_id].Set_initpos();

    task_switch = true;

    std::cout << "init set - COM x : " << tocabi_.link_[COM_id].x_init(0) << "\t y : " << tocabi_.link_[COM_id].x_init(1) << std::endl;

    data_out << "###############  COMMAND RECEIVED  ###############" << std::endl;
}

void TocabiController::ArmTaskCommandCallback(const tocabi_controller::ArmTaskCommandConstPtr &msg)
{
    atc.command_time = control_time_;
    control_time_pre_ = control_time_;
    atc.traj_time = msg->time;
    atc.l_x = msg->l_x;
    atc.l_y = msg->l_y;
    atc.l_z = msg->l_z;
    atc.l_roll = msg->l_roll*DEG2RAD;
    atc.l_pitch = msg->l_pitch*DEG2RAD;
    atc.l_yaw = msg->l_yaw*DEG2RAD;
    
    atc.r_x = msg->r_x;
    atc.r_y = msg->r_y;
    atc.r_z = msg->r_z;
    atc.r_roll = msg->r_roll*DEG2RAD;
    atc.r_pitch = msg->r_pitch*DEG2RAD;
    atc.r_yaw = msg->r_yaw*DEG2RAD;

    atc.mode = msg->mode;

    tocabi_.link_[Right_Foot].Set_initpos();
    tocabi_.link_[Left_Foot].Set_initpos();
    tocabi_.link_[Right_Hand].Set_initpos();
    tocabi_.link_[Left_Hand].Set_initpos();
    tocabi_.link_[Pelvis].Set_initpos();
    tocabi_.link_[Upper_Body].Set_initpos();
    tocabi_.link_[COM_id].Set_initpos();

    task_switch = true;

    Eigen::Vector3d TargetDelta_l, TargetDelta_r;
    TargetDelta_l << atc.l_x, atc.l_y, atc.l_z;
    TargetDelta_r << atc.r_x, atc.r_y, atc.r_z;

    tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init + TargetDelta_l;
    tocabi_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(atc.l_roll) * DyrosMath::rotateWithY(atc.l_pitch) * DyrosMath::rotateWithZ(atc.l_yaw) * tocabi_.link_[Left_Hand].rot_init;
    tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Right_Hand].x_init + TargetDelta_r;
    tocabi_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(atc.r_roll) * DyrosMath::rotateWithY(atc.r_pitch) * DyrosMath::rotateWithZ(atc.r_yaw) * tocabi_.link_[Right_Hand].rot_init;

    std::cout << "Arm Command Recieved" << endl;
    std::cout << "Init Pos Left: " << tocabi_.link_[Left_Hand].x_init << endl;
    std::cout << "Init Pos Right: " << tocabi_.link_[Right_Hand].x_init << endl;
    std::cout << "Target Pos Left: " << tocabi_.link_[Left_Hand].x_desired << endl;
    std::cout << "Target Pos Right: " << tocabi_.link_[Right_Hand].x_desired << endl;
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
                // mycontroller.compute_fast();
                // torque_desired = mycontroller.getControl();
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

    Wholebody_controller wc_(dc, tocabi_);

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
        kpa_(i) = 100;
        kda_(i) = 20;
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
            if (tc.mode == 0) //Pelvis position control
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
                torque_grav.segment(0, 12).setZero();
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

                tocabi_.link_[task_link].x_desired(0) = tocabi_.link_[task_link].x_init(0) + 0.05;
                tocabi_.link_[task_link].x_desired(1) = tocabi_.link_[task_link].x_init(1);
                tocabi_.link_[task_link].x_desired(2) = tocabi_.link_[task_link].x_init(2) + 0.06;
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
            else if (tc.mode == 7)
            {
                static int loop_temp;
                static int loop_;
                static bool cgen_init;
                static bool loop_cnged;
                static bool walking_init;
                static double foot_height;

                static bool zmperror_reset;

                //QP_switch = true;
                tocabi_.ee_[0].contact = true;
                tocabi_.ee_[1].contact = true;

                //right_foot_contact_ = true;
                //left_foot_contact_ = true;
                double time_segment_origin = tc.traj_time;
                static double time_segment;
                static double loop_start_time;
                static double loop_end_time;

                if (tc.task_init)
                {
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
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Matrix3d::Identity(), false);

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
                    task_number = 9;
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);
                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                    J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                    wc_.set_contact(tocabi_, 1, 1);
                    torque_grav = wc_.gravity_compensation_torque(tocabi_);

                    tocabi_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                    f_star.segment(6, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
                }
                else if (tocabi_.ee_[0].contact)
                {
                    if (walking_init)
                    {
                        tocabi_.link_[Right_Foot].x_init = tocabi_.link_[Right_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 15;

                    wc_.set_contact(tocabi_, 1, 0);
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Foot].Jac;
                    J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                    torque_grav = wc_.gravity_compensation_torque(tocabi_);

                    tocabi_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    Vector3d lf_desired;
                    lf_desired = tocabi_.link_[Right_Foot].x_init;
                    lf_desired(1) = -0.1024;
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
                    f_star.segment(12, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
                }
                else if (tocabi_.ee_[1].contact) // rightfoot contact
                {

                    Vector3d lf_desired;
                    if (walking_init)
                    {
                        tocabi_.link_[Left_Foot].x_init = tocabi_.link_[Left_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 15;
                    wc_.set_contact(tocabi_, 0, 1);

                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = tocabi_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = tocabi_.link_[Left_Foot].Jac;
                    J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                    torque_grav = wc_.gravity_compensation_torque(tocabi_);

                    tocabi_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);
                    //model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

                    lf_desired = tocabi_.link_[Left_Foot].x_init;

                    lf_desired(1) = 0.1024;
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
                    f_star.segment(12, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
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
                right_cp << 0.0, -0.09;
                left_cp << 0.0, 0.09;

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
                //if (extend_t > 0.0001)
                //{
                //    std::cout << extend_t << std::endl;
                //    time_segment = time_segment + extend_t;
                //}
                // est_cp_ = b_ * (model_.com_.pos.segment(0, 2) + model_.com_.vel.segment(0, 2) / w_) + (1 - b_) * zmp;
                // std::cout << "estimated_cp" << std::endl;
                // std::cout << est_cp_ << std::endl;
                if (cgen_init)
                {
                    std::cout << "1 ZMP : " << zmp(1) << std::endl;
                    cgen_init = false;
                }

                //wc_.set_zmp_control(tocabi_, zmp, 1.0);
                wc_.set_zmp_feedback_control(tocabi_, zmp, zmperror_reset);
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
                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
            }
            else if (tc.mode == 7)
            {
                //left is plus !
                static int phase = 0; //first phase -> left foot contact!
                static bool phase_init = true;
                static bool phase_end = false;

                tocabi_.ee_[LEFT].contact = true;
                tocabi_.ee_[RIGHT].contact = true;

                double step_legth = 0.0;
                double step_time = tc.traj_time;

                int last_phase = 10;

                static double phase_start_time;
                static double phase_end_time;

                static Vector2d cp_desired;
                Vector2d cp_current = tocabi_.com_.CP;

                if (phase_init)
                {
                    phase_start_time = control_time_;
                    phase_end_time = control_time_ + step_time;

                    //init all link in init
                    tocabi_.link_[Left_Foot].Set_initpos();
                    tocabi_.link_[Right_Foot].Set_initpos();
                    tocabi_.link_[COM_id].Set_initpos();

                    phase_init = false;
                }

                //double support phase : start&end
                //single support phase : left&right foot swing.

                if (phase == 0)
                {
                }
                else if (phase == last_phase)
                {
                }
                else
                {
                }

                if (control_time_ >= phase_end_time) //phase ending condition!
                {
                    phase_end = true;
                }

                if (phase_end) //phase end!
                {
                    phase_end = false;
                    phase_init = true;
                }
            }
            else if (tc.mode == 8)
            {
                //Left hand contact
                wc_.set_contact(tocabi_, 1, 1, 1, 0);

                tocabi_.task_force_control = false;

                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 9;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL);
                J_task.block(3, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac;

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                tocabi_.link_[COM_id].x_desired(0) += 0.2;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[Right_Hand].x_desired = tocabi_.link_[Right_Hand].x_init;
                tocabi_.link_[Right_Hand].x_desired(0) += 0.9;
                tocabi_.link_[Right_Hand].x_desired(2) += 0.2;
                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(3.141592) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(3.141592 / 2.0);

                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                f_star.segment(0, 3) = wc_.getfstar_tra(tocabi_, COM_id);
                f_star.segment(3, 6) = wc_.getfstar6d(tocabi_, Right_Hand);

                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);

                //tocabi_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                //f_star = wc_.getfstar6d(red_, COM_id);
                //torque_task = wc_.task_control_torque(red_, J_task, f_star);
                //torque_task = wc_.task_control_torque(J_task, f_star);

                if (1) //control_time_ > tc.command_time + tc.traj_time)
                {
                    std::cout << "##########" << std::endl
                              << "task time : \t" << control_time_ - tc.command_time << std::endl
                              << "dis pos x : \t" << tocabi_.com_.pos(0) - tocabi_.link_[Right_Foot].xpos_contact(0) << std::endl
                              << "lHand f z : \t" << tocabi_.LH_FT(2) - 9.81 << std::endl
                              << std::endl;
                }

                cr_mode = 1;
            }
            else if (tc.mode == 9)
            {
                //Both hand
                wc_.set_contact(tocabi_, 1, 1, 1, 1);

                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[COM_id].Jac;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //tocabi_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                //f_star = wc_.getfstar6d(tocabi_, COM_id);
                //torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);
                //torque_task = wc_.task_control_torque(J_task, f_star);

                cr_mode = 1;
            }
            else if (tc.mode == 10)
            {
                static bool reinit_lh = true;
                if (tc.task_init)
                {
                    reinit_lh = true;
                    tc.task_init = false;
                }

                wc_.set_contact(tocabi_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);

                task_number = 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                //J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                //tocabi_.link_[COM_id].x_desired(0) += 0.1;

                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init;
                tocabi_.link_[Left_Hand].x_desired(1) -= 0.05;
                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(-3.141592 / 2.0) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(0);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                if (control_time_ >= tc.command_time + tc.traj_time)
                {
                    if (reinit_lh)
                    {
                        tocabi_.link_[Left_Hand].Set_initpos();
                        reinit_lh = false;
                    }
                    tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init;
                    tocabi_.link_[Left_Hand].x_desired(2) -= 0.06;
                    tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time + tc.traj_time, tc.command_time + 2 * tc.traj_time);

                    tocabi_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(-3.141592 / 2.0) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(0);
                    tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time + tc.traj_time, tc.command_time + 2 * tc.traj_time, false);
                }

                f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                //f_star.segment(6, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
                f_star.segment(6, 6) = wc_.getfstar6d(tocabi_, Left_Hand);

                if (control_time_ >= tc.command_time + 2 * tc.traj_time)
                {
                    //f_star.segment(6, 6).setZero();
                    MatrixXd task_slc;
                    task_slc.setIdentity(task_number, task_number);

                    VectorXd force_desired;
                    force_desired.setZero(task_number);
                    force_desired(8) = -15.0;
                    task_slc(8, 8) = 0.0;

                    wc_.set_force_control(tocabi_, task_slc, force_desired);
                    //f_star(8) = -10.0;
                    //f_star(11) = 15.0;
                }

                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);

                if (control_time_ >= tc.command_time + 3 * tc.traj_time)
                {
                    tc.mode = 8;
                    tc.command_time = control_time_;
                    tc.task_init = true;
                    tocabi_.link_[Right_Hand].Set_initpos();
                    tocabi_.link_[COM_id].Set_initpos();
                }
            }
            else if (tc.mode == 11)
            {
                wc_.set_contact(tocabi_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(tocabi_, dc.fixedgravity);

                task_number = 3;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                //J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                //J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM_r;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                //tocabi_.link_[COM_id].x_desired(0) += 0.1;

                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                //tocabi_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                //tocabi_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Left_Hand].x_desired = tocabi_.link_[Left_Hand].x_init;
                tocabi_.link_[Left_Hand].x_desired(2); // -= 0.06;
                tocabi_.link_[Left_Hand].x_desired(1) -= 0.05;
                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                tocabi_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(-3.141592 / 2.0) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(0);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                std::cout << "rotation information :::: " << std::endl;
                std::cout << tocabi_.link_[Left_Hand].rot_desired.eulerAngles(0, 1, 2) << std::endl
                          << std::endl;
                std::cout << tocabi_.link_[Left_Hand].Rotm.eulerAngles(0, 1, 2) << std::endl;

                //f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                //f_star.segment(6, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
                //f_star.segment(0, 3) = wc_.getfstar6d(tocabi_, Left_Hand);
                f_star.segment(0, 3) = wc_.getfstar_rot(tocabi_, Left_Hand);

                std::cout << f_star.segment(0, 3) << std::endl
                          << std::endl;

                torque_task = wc_.task_control_torque(tocabi_, J_task, f_star);

                std::cout << "lambda : " << std::endl
                          << tocabi_.lambda << std::endl;
                std::cout << "torque_task : " << std::endl
                          << torque_task << std::endl;
            }
            else if (tc.mode == 12)
            {
                wc_.set_contact(tocabi_, 1, 1);
                //torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 9;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                //tocabi_.link_[COM_id].x_desired(0) += 0.1;

                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithY(tc.angle) * Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                //std::cout << "rotation information :::: " << std::endl;
                //std::cout << tocabi_.link_[Upper_Body].rot_desired.eulerAngles(0, 1, 2) << std::endl
                //          << std::endl;
                //std::cout << tocabi_.link_[Upper_Body].Rotm.eulerAngles(0, 1, 2) << std::endl;

                f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                f_star.segment(6, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
                //f_star.segment(6, 3) = wc_.getfstar6d(red_, Left_Hand);

                //std::cout << f_star.segment(6, 3) << std::endl
                //          << std::endl;
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);
                cr_mode = 2;
                Vector3d lrot_eulr, rrot_eulr;
                lrot_eulr = DyrosMath::rot2Euler(tocabi_.link_[Left_Foot].Rotm);
                rrot_eulr = DyrosMath::rot2Euler(tocabi_.link_[Right_Foot].Rotm);

                double lz_y, rz_y;
                if (control_time_ == tc.command_time)
                {
                    data_out << "t \t ce \t lz \t rz \t lee \t re \t lfz \t rfz \t ly \t ry" << std::endl;
                }
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    data_out << control_time_ << "\t" << tocabi_.link_[COM_id].x_traj(1) - tocabi_.link_[COM_id].xpos(1) << "\t" << tocabi_.ContactForce(3) / tocabi_.ContactForce(2) << "\t" << tocabi_.ContactForce(9) / tocabi_.ContactForce(8) << "\t" << lrot_eulr(0) << "\t" << rrot_eulr(0) << "\t" << tocabi_.ContactForce(2) << "\t" << tocabi_.ContactForce(8) << "\t" << tocabi_.link_[Left_Foot].xpos(1) << "\t" << tocabi_.link_[Right_Foot].xpos(1) << std::endl;
                }

                //std::cout << "cf: " << std::endl
                //          << wc_.get_contact_force(red_, torque_task + torque_grav);
            }
            else if (tc.mode == 13)
            {
                wc_.set_contact(tocabi_, 1, 1);
                //torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 9 + 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac;
                J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Upper_Body].Jac_COM_r;
                J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Left_Hand].Jac_COM;
                J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = tocabi_.link_[Right_Hand].Jac_COM;

                tocabi_.link_[COM_id].x_desired = tc.ratio * tocabi_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos;
                //tocabi_.link_[COM_id].x_desired(0) += 0.1;

                tocabi_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * tocabi_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * tocabi_.link_[Right_Foot].xpos(2);
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                tocabi_.link_[COM_id].rot_desired = Matrix3d::Identity();
                tocabi_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                tocabi_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithY(tc.angle) * Matrix3d::Identity();
                tocabi_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                Vector3d ldi = tocabi_.link_[Left_Hand].x_init - tocabi_.link_[Pelvis].x_init;
                Vector3d rdi = tocabi_.link_[Right_Hand].x_init - tocabi_.link_[Pelvis].x_init;

                tocabi_.link_[Left_Hand].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Pelvis].xpos + ldi, -tocabi_.link_[Pelvis].v, tocabi_.link_[Pelvis].xpos + ldi, -tocabi_.link_[Pelvis].v);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, true);

                tocabi_.link_[Right_Hand].rot_desired = Matrix3d::Identity();
                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, tocabi_.link_[Pelvis].xpos + rdi, -tocabi_.link_[Pelvis].v, tocabi_.link_[Pelvis].xpos + rdi, -tocabi_.link_[Pelvis].v);
                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, true);

                //std::cout << "rotation information :::: " << std::endl;
                //std::cout << tocabi_.link_[Upper_Body].rot_desired.eulerAngles(0, 1, 2) << std::endl
                //          << std::endl;
                //std::cout << tocabi_.link_[Upper_Body].Rotm.eulerAngles(0, 1, 2) << std::endl;

                f_star.segment(0, 6) = wc_.getfstar6d(tocabi_, COM_id);
                f_star.segment(6, 3) = wc_.getfstar_rot(tocabi_, Upper_Body);
                f_star.segment(9, 6) = wc_.getfstar6d(tocabi_, Left_Hand);
                f_star.segment(15, 6) = wc_.getfstar6d(tocabi_, Right_Hand);

                //f_star.segment(6, 3) = wc_.getfstar6d(tocabi_, Left_Hand);

                //std::cout << f_star.segment(6, 3) << std::endl
                //          << std::endl;
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);
                cr_mode = 2;

                //std::cout << "cf: " << std::endl
                //          << wc_.get_contact_force(tocabi_, torque_task + torque_grav);
            }
            else if (tc.mode == 14)
            {
                //example custom controller
                cr_mode = 2; //turn off contact torque redistribution
                torque_grav.setZero();
                torque_task.setZero();

                // mycontroller.compute_slow();
                // torque_task = mycontroller.getControl();
                // cout<<"torque_task:\n"<<torque_task<<endl;
            }
            else if (tc.mode == 15)
            {
                //////////Generalized Biped Walking Controller/////////////////////                
                cr_mode = 2;
                torque_task.setZero(MODEL_DOF);

                getRobotData(wc_);
                walkingStateManager();
                getProcessedRobotData(wc_);

                motionGenerator();
                getCOMTrajectory();
                getSwingFootXYTrajectory(walking_phase_, com_pos_current_, com_vel_current_, com_vel_desired_);


                // torque_task += comVelocityControlCompute(wc_);             //support control for COM velocity control
                torque_task += comVelocityControlCompute2(wc_);             //support control for COM velocity control
                // if(walking_phase_ == 1)
                //     cout<<"torque_task_com_vel: \n" << torque_task<<endl;

                torque_task += jointTrajectoryPDControlCompute(wc_)*first_torque_supplier_;       //upper body motion + swing foot control + knee angle control
                if(walking_phase_ > 0.1)
                {
                    // cout<<"torque_task: \n" << torque_task<<endl;
                    // cout<<"first_torque_supplier_: \n" << first_torque_supplier_<<endl;
                }

                // torque_task += zmpAnkleControl();
                // wc_.set_contact(tocabi_, 0, 0, 0, 0);    
                // if(foot_swing_trigger_ == true)
                // {
                //     if( foot_contact_ == 1) 
                //     {   
                //         torque_grav.segment(6, 6).setZero();
                //         // wc_.set_contact(tocabi_, 1, 0);
                //     }
                //     else if( foot_contact_ == -1 )
                //     {
                //         torque_grav.segment(0, 6).setZero();
                //         // wc_.set_contact(tocabi_, 0, 1);
                //     }
                //     else
                //     {
                //         wc_.set_contact(tocabi_, 1, 1);
                //     }
                // }
                // else
                // {
                //     wc_.set_contact(tocabi_, 1, 1);
                // }
                // if(walking_phase_ < 0.1)
                // {   
                //     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
                //     cout<<"foot_swing_trigger_: \n"<<foot_swing_trigger_<<endl;
                //     cout<<"com_pos_desired_: \n"<<com_pos_desired_<<endl;
                //     cout<<"com_vel_desired_: \n"<<com_vel_desired_<<endl;
                //     cout<<"zmp_measured_: \n"<<zmp_measured_<<endl;
                    // cout<<"l_ft_: \n"<<l_ft_<<endl;
                    // cout<<"r_ft_: \n"<<r_ft_<<endl;
                // }
                // if(walking_phase_ < 0.1)
                // {
                //     cout<<"walking_phase_: " << walking_phase_<<endl;
                //     cout<<"start_time_: " << start_time_<<endl;
                //     cout<<"current_time_: " << current_time_<<endl;
                //     cout<<"foot_swing_trigger_: " << foot_swing_trigger_<<endl;
                //     cout<<"foot_contact_: " << foot_contact_<<endl;
                //     cout<<"swing_foot_vel_trajectory_from_global_(2): " << swing_foot_vel_trajectory_from_global_(2)<<endl;
                //     cout<<"com_pos_desired_: " << com_pos_desired_<<endl;
                //     cout<<"com_vel_desired_: " << com_vel_desired_<<endl;
                //     cout<<"swing_foot_vel_trajectory_from_global_(2): " << swing_foot_vel_trajectory_from_global_(2)<<endl;
                // }
                
                // torque_task = tuneTorqueForZMPSafety( torque_task );        //turn off velocity tuning if the zmp is outside of the foot


                torque_task_ = torque_task;
                torque_grav_ = torque_grav;
                savePreData();

            }



            //////////////////////////////////// Arm Control /////////////////////////////////////////
            if (atc.mode == 0)
            {
                cout<<"tocabi_.q_virtual_: \n"<<tocabi_.q_virtual_<<endl;
                
                cout<<"tocabi_.link_.[Pelvis].Rotm: \n"<<tocabi_.link_[Pelvis].Rotm<<endl;

                cout<<"tocabi_.link_.[Pelvis].xpos: \n"<<tocabi_.link_[Pelvis].xpos<<endl;

                cout<<"tocabi_.link_[COM_id].xpos: \n"<<tocabi_.link_[COM_id].xpos<<endl;

                cout<<"tocabi_.link_[Left_Foot].xpos: \n"<<tocabi_.link_[Left_Foot].xpos<<endl;

                cout<<"tocabi_.link_[Right_Foot].xpos: \n"<<tocabi_.link_[Right_Foot].xpos<<endl;

                const int arm_task_number = 6;
                const int arm_dof = 8;
                ////////// CoM Control //////////////////////
                wc_.set_contact(tocabi_, 1, 1);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = tocabi_.link_[COM_id].Jac;
                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac_COM_p;
                J_task.block(0, 21, 6, arm_dof).setZero(); // Exclude Left Arm Jacobian
                J_task.block(0, 31, 6, arm_dof).setZero(); // Exclude Right Arm Jacobian

                tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
                // tocabi_.link_[COM_id].x_desired(1) += 0.1;
                tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);

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

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, atc.command_time, atc.command_time + atc.traj_time, false);

                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);
                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, atc.command_time, atc.command_time + atc.traj_time, false);
                
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
            else if (atc.mode == 1)
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

                tocabi_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);
                tocabi_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, atc.command_time, atc.command_time + atc.traj_time, false);

                tocabi_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);
                tocabi_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, atc.command_time, atc.command_time + atc.traj_time, false);
                
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
    tocabi_.com_ = dc.com_;

    mtx_dc.unlock();
}

void TocabiController::initialize()
{
    torque_desired.setZero();
    initWalkingParameter();
}

void TocabiController::ContinuityChecker(double data)
{
}

void TocabiController::ZMPmonitor()
{
}

void TocabiController::setWalkingParameter(double walking_duration, double walking_speed, double step_width, double knee_target_angle)
{
    walking_duration_ = walking_duration;
    walking_speed_ = walking_speed;
    step_width_ = step_width;
    knee_target_angle_ = knee_target_angle;

    walking_duration_ = DyrosMath::minmax_cut(walking_duration_, 0.5, 1.5);
    walking_speed_ = DyrosMath::minmax_cut(walking_speed_, -0.5, 1.0);
    step_width_ = DyrosMath::minmax_cut(step_width_, 0.17, 0.25);
    knee_target_angle_ = DyrosMath::minmax_cut(knee_target_angle_, 0.0, 1.5);
}

void TocabiController::initWalkingParameter()
{
    walking_mode_on_ = true;
    stop_vel_threshold_ =  0.10;
    walking_duration_ = 0.6;
    walking_speed_ = 0.2;
    step_width_ = 0.090;
    knee_target_angle_ = 0.1;                               //4.5degree
    // knee_target_angle_ = M_PI/40;                               //4.5degree
    yaw_angular_vel_ = 1;                                       //   rad/s
    swing_foot_height_ = 0.07;
    switching_phase_duration_ = 0.01;

    first_step_flag_ = true;
    first_step_trigger_ = false;
    foot_swing_trigger_ = false;
    stop_walking_trigger_ = false;
    foot_contact_ = -1;
    jac_com_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_com_pos_.setZero(3, MODEL_DOF_VIRTUAL);
    jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    //set init pre data
    com_pos_desired_pre_ = dc.link_[COM_id].xpos;
    com_vel_desired_pre_.setZero();
    pre_time_ = tocabi_.control_time_;
    pre_desired_q_ = tocabi_.q_;

    init_q_ = tocabi_.q_;
    desired_q_ = init_q_;
}

void TocabiController::getRobotData(Wholebody_controller &wc)
{
    // Wholebody_controller wc_(dc_, tocabi_);
    // if(control_time_ == tc.command_time)
    // {
    //     initWalkingParameter();
    // }
    current_time_ = tocabi_.control_time_;
 
    dt_ = current_time_ - pre_time_;


    current_q_ = tocabi_.q_;
    current_q_dot_ = tocabi_.q_dot_;
    current_q_ddot_ = tocabi_.q_ddot_virtual_.segment(6, MODEL_DOF);
    pelv_pos_current_ = tocabi_.link_[Pelvis].xpos;
    pelv_vel_current_ = tocabi_.link_[Pelvis].v;
    pelv_rot_current_ = tocabi_.link_[Pelvis].Rotm;
    pelv_rpy_current_ = DyrosMath::rot2Euler(pelv_rot_current_); //ZYX multiply 
    // pelv_rpy_current_ = (pelv_rot_current_).eulerAngles(2, 1, 0);
    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    pelv_rot_current_yaw_aline_ = pelv_yaw_rot_current_from_global_.transpose()*pelv_rot_current_;

    com_pos_current_ = pelv_yaw_rot_current_from_global_.transpose()*(tocabi_.link_[COM_id].xpos - pelv_pos_current_);
    com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[COM_id].v;

    lfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose()*(tocabi_.link_[Left_Foot].xpos- pelv_pos_current_);
    lfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Left_Foot].Rotm;
    rfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose()*(tocabi_.link_[Right_Foot].xpos- pelv_pos_current_);
    rfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Right_Foot].Rotm;

    lfoot_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Left_Foot].v;
    lfoot_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Left_Foot].w;

    rfoot_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Right_Foot].v;
    rfoot_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Right_Foot].w;

    Matrix6d R_R;
    R_R.setZero();
    R_R.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
    R_R.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();

    jac_com_ = R_R*tocabi_.link_[COM_id].Jac;
    jac_com_pos_ = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[COM_id].Jac_COM_p;
    jac_rhand_ = R_R*tocabi_.link_[Right_Hand].Jac;
    jac_lhand_ = R_R*tocabi_.link_[Left_Hand].Jac;
    jac_rfoot_ = R_R*tocabi_.link_[Right_Foot].Jac;
    jac_lfoot_ = R_R*tocabi_.link_[Left_Foot].Jac;

    tocabi_.link_[Left_Foot].v;
    

    Eigen::Vector3d zmp_local_both_foot;
    // tocabi_.ZMP_ft = wc_.GetZMPpos(tocabi_);
    
    // zmp_local_both_foot = wc.GetZMPpos_fromFT(tocabi_).segment(0, 2);  //get zmp using f/t sensors on both foot
    
    zmp_local_lfoot_(0) = -tocabi_.ContactForce_FT(4) / tocabi_.ContactForce_FT(2);
    zmp_local_lfoot_(1) = -tocabi_.ContactForce_FT(3) / tocabi_.ContactForce_FT(2);
    zmp_local_rfoot_(0) = -tocabi_.ContactForce_FT(10) / tocabi_.ContactForce_FT(8);
    zmp_local_rfoot_(1) = -tocabi_.ContactForce_FT(9) / tocabi_.ContactForce_FT(8);

    zmp_dot_local_lfoot_ = (zmp_local_lfoot_ - zmp_local_lfoot_pre_)/dt_;
    zmp_dot_local_rfoot_ = (zmp_local_rfoot_ - zmp_local_rfoot_pre_)/dt_;

    zmp_measured_lfoot_ = lfoot_transform_current_from_global_.linear()*zmp_local_lfoot_ + lfoot_transform_current_from_global_.translation(); //from global
    
    zmp_measured_rfoot_ = rfoot_transform_current_from_global_.linear()*zmp_local_lfoot_ + rfoot_transform_current_from_global_.translation();

    zmp_measured_ = (zmp_measured_lfoot_*tocabi_.ContactForce_FT(2) + zmp_measured_rfoot_*tocabi_.ContactForce_FT(8))/(tocabi_.ContactForce_FT(2) + tocabi_.ContactForce_FT(8)); //from global
    zmp_dot_measured_ = (zmp_measured_ - zmp_measured_pre_)/dt_;

    l_ft_ = tocabi_.ContactForce_FT.segment(0, 6);
    r_ft_ = tocabi_.ContactForce_FT.segment(6, 6); 

    first_torque_supplier_ = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_+0.1, 0, 1, 0, 0);
}

void TocabiController::walkingStateManager()
{
    if(walking_phase_ <1)
    {
        if(walking_speed_ == 0)
        {
            //first step start
            if(foot_swing_trigger_ == false)
            {
                if(first_step_flag_ == true)
                {
                    if( balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0)) ) //gonna be fall
                    // if(false)
                    {
                        foot_swing_trigger_ = true; 
                        first_step_trigger_ = true;
                        first_step_flag_ = true;
                        cout<<" ################################ Balancing Control ON! ################################"<<endl;
                    }
                    else
                    {
                        stop_walking_trigger_ == false;
                        foot_swing_trigger_ = false;
                        first_step_flag_ = true;
                        first_step_trigger_ = false;
                        start_time_ = current_time_;
                    }
                }
            }
        }
        else
        {
            if(foot_swing_trigger_ == false )
            {
                // if( checkZMPinWhichFoot(zmp_measured_) == true )
                if( (com_pos_current_(0) - support_foot_transform_current_.translation()(0) )/(walking_speed_*walking_duration_)> 0.2 
                || (com_vel_current_(0)/(walking_speed_ + 1e-3) > 0.3))
                {
                    first_step_trigger_ = true;
                    foot_swing_trigger_ = true;
                    // stop_walking_trigger_ = false;
                    cout<<" ################################ First Step Triggered! ################################"<<endl;
                }
                else
                {
                    start_time_ = current_time_;;
                }
            }
            else
            {

                if(foot_contact_ == 1)
                {
                    Vector3d phi_swing_ankle;
                    phi_swing_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), swing_foot_rot_trajectory_from_global_);

                    if( (walking_phase_ > 0.5) && (-r_ft_(2) > tocabi_.com_.mass*GRAVITY/2) 
                    && (abs(phi_swing_ankle.norm())<0.03) && (r_ft_.segment(3,2).norm() < 20) )
                    {
                        start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
                        walking_phase_ =1;
                        cout<<" ################ Early Step Change Occured! #########################"<<endl;
                    }
                }
                else if(foot_contact_ == -1)
                {
                    Vector3d phi_swing_ankle;
                    phi_swing_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), swing_foot_rot_trajectory_from_global_);

                    if( (walking_phase_ > 0.5) && (-l_ft_(2) > tocabi_.com_.mass*GRAVITY/2) 
                    && (abs(phi_swing_ankle.norm())<0.03) && (r_ft_.segment(3,2).norm() < 20) )
                    {
                        start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
                        walking_phase_ =1;
                        cout<<" ################ Early Step Change Occured! #########################"<<endl;
                    }
                }
            }
        }
    }
    else if(walking_phase_ == 1)
    {
        if(walking_speed_ == 0)
        {

            if( balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0)) ) //gonna be fall
            {
                foot_swing_trigger_ = true;         
                foot_contact_ = -foot_contact_;                        //support foot change
                cout<<" ################################ Balancing Control ON! ################################"<<endl;
            }
            else
            {
                foot_swing_trigger_ = false;
                first_step_flag_ = true;
                first_step_trigger_ = false;
                if(stop_walking_trigger_ == false)
                {
                    stop_walking_trigger_ = true;
                }
                stance_start_time_ = current_time_;
            }
        }
        else
        {
            if(first_step_flag_ == true)
            {
                first_step_flag_ == false;
            }
            
            
            foot_contact_ = -foot_contact_;
            stop_walking_trigger_ == false;
            foot_swing_trigger_ = true;         
            cout<<" ################################ Support Foot Changed! ################################"<<endl;                 
        }

        start_time_ = current_time_;
    }
    
    // walking_duration_ = 1;
    walking_phase_ = (current_time_-start_time_)/walking_duration_;
    walking_phase_ = DyrosMath::minmax_cut(walking_phase_, 0, 1);
}

bool TocabiController::balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d)
{
    bool trigger = false;
    Vector2d capture_point_2d;
    Vector2d middle_point_of_foot_2d;
    double omega;
    omega = sqrt(GRAVITY/(com_pos_current_(2) - support_foot_transform_current_.translation()(2)));
    capture_point_2d = com_pos_2d + com_vel_2d/omega;
    middle_point_of_foot_2d = middle_of_both_foot_.segment(0, 2);

    

    // if(capture_point_2d.norm() > stop_vel_threshold_)
    // {
    //     trigger = true;
    //     cout<<"balance swing foot control activated"<<endl;
    // }
    // if( (capture_point_2d(0)>middle_point_of_foot_2d(0) + stop_vel_threshold_)||(capture_point_2d(0)<middle_point_of_foot_2d(0) - stop_vel_threshold_) )
    // {
    //     trigger = true;
    //     cout<<"Catpure point in X axis is over the safety boundary! balance swing foot control activated"<<endl;
    // }
    

    // if( (capture_point_2d(1)>lfoot_transform_current_from_global_.translation()(1) + 0.05)|| (capture_point_2d(1)<rfoot_transform_current_from_global_.translation()(1) - 0.05) ) 
    // {
    //     trigger = true;
    //     cout<<"Catpure point in Y axis is over the safety boundary! balance swing foot control activated"<<endl;
    // }

    if( com_vel_2d.norm() > stop_vel_threshold_)
    {
        trigger = true;
        cout<<"com vel is over the limit ("<< com_vel_2d.norm()<<")"<<endl;
    }
    

    return trigger;
}

int TocabiController::checkZMPinWhichFoot(Eigen::Vector2d zmp_measured)
{
    int flag;
    Eigen::Vector2d diff_zmp_lfoot;
    Eigen::Vector2d diff_zmp_rfoot;
    Eigen::Vector2d foot_size;
    double safe_region_ratio = 0.9;

    
    diff_zmp_lfoot(0) = abs(zmp_measured(0) - lfoot_transform_current_from_global_.translation()(0));
    diff_zmp_lfoot(1) = abs(zmp_measured(1) - lfoot_transform_current_from_global_.translation()(1));

    diff_zmp_rfoot(0) = abs(zmp_measured(0) - rfoot_transform_current_from_global_.translation()(0));
    diff_zmp_rfoot(1) = abs(zmp_measured(1) - rfoot_transform_current_from_global_.translation()(1));

    foot_size(0) = 0.15;
    foot_size(1) = 0.085;
    if( (diff_zmp_lfoot(0)<safe_region_ratio*foot_size(0)) && (diff_zmp_lfoot(1)<safe_region_ratio*foot_size(1)))
    {
        flag = 1; //zmp is in the left foot
    }
    else if( (diff_zmp_rfoot(0)<safe_region_ratio*foot_size(0)) && (diff_zmp_rfoot(1)<safe_region_ratio*foot_size(1)))
    {
        flag = -1; //zmp is in the right foot
    }
    else
    {
        flag = 0; 
    }
    
    return flag;
}

void TocabiController::getProcessedRobotData(Wholebody_controller &wc)
{
    if(walking_mode_on_) //command on
    {
        stance_start_time_ = current_time_;
        start_time_ = current_time_;

        walking_mode_on_ = false;
    }

    bool robot_goes_into_stance_phase =  (current_time_ == stance_start_time_);
    bool robot_start_walking =  ((foot_swing_trigger_ == true)&&(current_time_ == start_time_));
    if( robot_goes_into_stance_phase || robot_start_walking)
    {
        com_pos_init_ = com_pos_current_;
        com_vel_init_ = com_vel_current_;

        pelv_pos_init_ = pelv_pos_current_;
        pelv_vel_init_ = pelv_vel_current_;
        pelv_rot_init_ = pelv_rot_current_;
        pelv_rpy_init_ = pelv_rpy_current_;
        pelv_rot_init_yaw_aline_ = pelv_rot_current_yaw_aline_;

        lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;
        if(foot_contact_ == 1) // left support foot
        {
            swing_foot_transform_init_ = rfoot_transform_current_from_global_;
            support_foot_transform_init_ = lfoot_transform_current_from_global_;
            swing_foot_vel_init_ = lfoot_vel_current_from_global;
        }
        else if(foot_contact_ == -1) //right support foot
        {
            swing_foot_transform_init_ = lfoot_transform_current_from_global_;
            support_foot_transform_init_ = rfoot_transform_current_from_global_;
            swing_foot_vel_init_ = rfoot_vel_current_from_global;
        }

        init_q_ = current_q_;
    }

    if(foot_contact_ == 1) // left support foot
    {

        swing_foot_transform_current_ = rfoot_transform_current_from_global_;
        support_foot_transform_current_ = lfoot_transform_current_from_global_;
        swing_foot_vel_current_ = rfoot_vel_current_from_global;
    }
    else if(foot_contact_ == -1) //right support foot
    {
        swing_foot_transform_current_ = lfoot_transform_current_from_global_;
        support_foot_transform_current_ = rfoot_transform_current_from_global_;
        swing_foot_vel_current_ = lfoot_vel_current_from_global;
    } 
    else if(foot_swing_trigger_ == false)
    {

    }

    zmp_measured_local_ = wc.GetZMPpos_fromFT(tocabi_, true);
    
    middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() + rfoot_transform_current_from_global_.translation())/2;
}



void TocabiController::motionGenerator()
{   


    motion_q_dot_.setZero();
    motion_q_.setZero();
    pd_control_mask_.setZero();


    ///////////////////////LEG/////////////////////////
    //////LEFT LEG///////0 0 0.02 0.15 -0.17 0
    motion_q_(0)   = 0;
    motion_q_(1)   = 0;
    motion_q_(2)   = 0.02;
    // motion_q_(3)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
    motion_q_(3)   = 0.1;
    motion_q_(4)   = -0.12;
    motion_q_(5)   = 0;
    pd_control_mask_(0) = 1;
    pd_control_mask_(1) = 0;
    pd_control_mask_(2) = 0;
    pd_control_mask_(3) = 1;
    pd_control_mask_(4) = 1;
    pd_control_mask_(5) = 1;
    //////////////////////
    /////RIFHT LEG////////0 0 0.02 0.15 -0.17 0 
    motion_q_(6)   = 0;
    motion_q_(7)   = 0;
    motion_q_(8)   = 0.02;
    // motion_q_(9)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
    motion_q_(9)   = 0.1;
    motion_q_(10)  = -0.12;
    motion_q_(11)  = 0;
    pd_control_mask_(6)  = 1;
    pd_control_mask_(7)  = 0;
    pd_control_mask_(8)  = 0;
    pd_control_mask_(9)  = 1;
    pd_control_mask_(10) = 1;
    pd_control_mask_(11) = 1;
    ///////////////////////WAIST/////////////////////////
    motion_q_(12) = 0;//yaw
    motion_q_(13) = 0;//pitch
    motion_q_(14) = 0;//roll
    pd_control_mask_(12) = 1;
    pd_control_mask_(13) = 1;
    pd_control_mask_(14) = 1;
    /////////////////////////////////////////////////////

    ///////////////////////HEAD/////////////////////////
    motion_q_(23) = 0;//yaw                     
    motion_q_(24) = 0;//pitch
    pd_control_mask_(23) = 1;
    pd_control_mask_(24) = 1;
    /////////////////////////////////////////////////////

    ///////////////////////ARM/////////////////////////
    //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
    motion_q_(15) = 0.3;                     
    motion_q_(16) = 0.3;
    motion_q_(17) = 1.5;
    motion_q_(18) = -1.27;                     
    motion_q_(19) = -1.0;
    motion_q_(20) = 0.0;
    motion_q_(21) = -1.0;                     
    motion_q_(22) = 0.0;
    pd_control_mask_(15) = 1;
    pd_control_mask_(16) = 1;
    pd_control_mask_(17) = 1;
    pd_control_mask_(18) = 1;
    pd_control_mask_(19) = 1;
    pd_control_mask_(20) = 1;
    pd_control_mask_(21) = 1;
    pd_control_mask_(22) = 1;
    //////////////////////
    /////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
    motion_q_(25) = -0.3;
    motion_q_(26) = -0.3;
    motion_q_(27) = -1.5;
    motion_q_(28) = 1.27;
    motion_q_(29) = 1.0;
    motion_q_(30) = 0.0;
    motion_q_(31) = 1.0;
    motion_q_(32) = 0.0;
    pd_control_mask_(25) = 1;
    pd_control_mask_(26) = 1;
    pd_control_mask_(27) = 1;
    pd_control_mask_(28) = 1;
    pd_control_mask_(29) = 1;
    pd_control_mask_(30) = 1;
    pd_control_mask_(31) = 1;
    pd_control_mask_(32) = 1;
    /////////////////////////////////////////////////////

    /////////////////FOOT HEIGHT/////////////////////////
    double default_stance_foot_z_from_pelv = -0.349*(cos(0.02) + cos(0.12))-0.1225;
    
    
    if(foot_swing_trigger_ == true)
    {
        if(walking_phase_<0.5)
        {
            // swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(0);
            // swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(1);
            swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1*switching_phase_duration_, 0.5, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(0);
            swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1*switching_phase_duration_, 0.5, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(1);
            swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1*switching_phase_duration_, 0.5, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(2);
        }
        else
        {
            swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2)+0.000, 0, 0)(0);
            swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2)+0.000, 0, 0)(1); 
            swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2)+0.000, 0, 0)(2); 
        }
        
        double current_swing_foot_height_from_ground = swing_foot_transform_current_.translation()(2) - support_foot_transform_current_.translation()(2);
        double swing_foot_z_pos_error = swing_foot_pos_trajectory_from_global_(2) - swing_foot_transform_current_.translation()(2);
        double swing_foot_z_vel_error = swing_foot_vel_trajectory_from_global_(2) - swing_foot_vel_current_(2);

        double kp = 0.0;
        double kv = 0.00;

        double swithching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);
        swing_foot_pos_trajectory_from_global_(2) += swithching*(kp*swing_foot_z_pos_error + kv*swing_foot_z_vel_error);

    }
}

void TocabiController::getCOMTrajectory()
{
    double desired_step_position_in_y;

    com_pos_desired_(2) = com_pos_current_(2);
    com_vel_desired_(2) = com_vel_current_(2);
    com_acc_desired_.setZero();

    if( (walking_speed_ != 0)) // when the robot want to move
    {

        // com_vel_desired_(0) = walking_speed_;
        // com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;


        if(first_step_trigger_ == false)
        {
            com_vel_desired_(0) = walking_speed_;
            com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

            desired_step_position_in_y =  - (step_width_)*foot_contact_ ;

            // com_vel_desired_(1) = 0;
            // com_pos_desired_(1) = com_pos_current_(1);
            // com_vel_desired_(1) = ( desired_step_position_in_y-com_pos_init_(1)+support_foot_transform_init_.translation()(1) )/(walking_duration_);
            // com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1-walking_phase_)*(com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_*desired_step_position_in_y;   
            com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_+0*walking_duration_, stance_start_time_+3*walking_duration_, (com_pos_init_)(0), 0, 0, middle_of_both_foot_(0) + walking_speed_*walking_duration_, 0, 0)(0);
            com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_+0*walking_duration_, stance_start_time_+3*walking_duration_, (com_pos_init_)(0), 0, 0, middle_of_both_foot_(0) + walking_speed_*walking_duration_, 0, 0)(1);
            com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+2*walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1) -0.00*foot_contact_, 0, 0)(0); //-0.03*foot_contact_
            com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+2*walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1) -0.00*foot_contact_, 0, 0)(1);
            // com_acc_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+2*walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1)  , 0, 0)(2);
        }
        else
        {
            com_vel_desired_(0) = walking_speed_;
            // com_pos_desired_(0) = com_pos_current_(0);  
            com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;
 

            desired_step_position_in_y = - (step_width_)*foot_contact_ ;
            double target_com_y_speed = ( support_foot_transform_init_.translation()(1)+desired_step_position_in_y-com_pos_init_(1) )/(walking_duration_);

            // com_vel_desired_(1) = 0;
            // com_pos_desired_(1) = com_pos_current_(1);
            // com_vel_desired_(1) = DyrosMath::cubic(walking_phase_, 0, 5*switching_phase_duration_, 0, target_com_y_speed, 0, 0);
            com_vel_desired_(1) = target_com_y_speed;
            com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1-walking_phase_)*( com_pos_init_(1) - support_foot_transform_init_.translation()(1) ) + walking_phase_*(desired_step_position_in_y );
            // com_pos_desired_(1) = com_pos_current_(1) + com_vel_desired_(1)*dt_;
            // com_pos_desired_(1) = DyrosMath::QuinticSpline(walking_phase_, 0, 1, support_foot_transform_current_.translation()(1)+ com_pos_init_(1)- support_foot_transform_init_.translation()(1), 0, 0, support_foot_transform_current_.translation()(1)+desired_step_position_in_y, 0, 0)(0);
            // com_vel_desired_(1) = DyrosMath::QuinticSpline(walking_phase_, 0, 1, support_foot_transform_current_.translation()(1)+ com_pos_init_(1)- support_foot_transform_init_.translation()(1), 0, 0, support_foot_transform_current_.translation()(1)+desired_step_position_in_y, 0, 0)(1);
        }
        
        

        // com_vel_desired_(1) = ( desired_step_position_in_y-com_pos_init_(1)+support_foot_transform_init_.translation()(1) )/(walking_duration_);
        // com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1-walking_phase_)*(com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_*desired_step_position_in_y;
        //com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+walking_duration_, com_pos_init_(1), 0, 0, com_pos_init_(1)+step_width_*foot_contact_, 0, 0)(0);  
        //com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+walking_duration_, com_pos_init_(1), 0, 0, com_pos_init_(1)+step_width_*foot_contact_, 0, 0)(1);  
    }
    else
    {
        if((foot_swing_trigger_ == true))
        {
            double traj_duraiton = 3.0;

            // com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(0);
            // com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(1);
            com_pos_desired_(0) = com_pos_current_(0);
            com_vel_desired_(0) = 0;
            // com_pos_desired_(0) = middle_of_both_foot_(0);
    
            desired_step_position_in_y = - (step_width_)*foot_contact_ ;
            double target_com_y_speed = ( support_foot_transform_init_.translation()(1)+desired_step_position_in_y-com_pos_init_(1) )/(walking_duration_);

            // com_vel_desired_(1) = 0;
            // com_pos_desired_(1) = com_pos_current_(1);
            // com_vel_desired_(1) = DyrosMath::cubic(walking_phase_, 0, 5*switching_phase_duration_, 0, target_com_y_speed, 0, 0);
            com_vel_desired_(1) = target_com_y_speed;
            com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1-walking_phase_)*( com_pos_init_(1) - support_foot_transform_init_.translation()(1) ) + walking_phase_*(desired_step_position_in_y );        
        }
        else
        {     
            double traj_duraiton = 0.5;

            com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(0);
            com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(1);
            // com_pos_desired_(0) = com_pos_init_(0);
            // com_vel_desired_(0) = 0;
            // com_pos_desired_(0) = middle_of_both_foot_(0);
    
            // com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+5, (com_pos_init_)(1), 0, 0, lfoot_transform_current_from_global_.translation()(1), 0, 0)(0);
            // com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+5, (com_pos_init_)(1), 0, 0, lfoot_transform_current_from_global_.translation()(1), 0, 0)(1);
            com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(1), 0, 0, (middle_of_both_foot_)(1), 0, 0)(0);
            com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(1), 0, 0, (middle_of_both_foot_)(1), 0, 0)(1);    
        }
        
    }
}

void TocabiController::getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired)
{   
    Eigen::Vector2d d;
    Eigen::Vector2d d_prime;
    double d_temp_x;
    double d_temp_y;
    double alpha_x = 0.05;
    double alpha_y = 0.00;

    if(foot_swing_trigger_ == true)
    {
        // x axis
        d_temp_x = ( com_pos_current(2) - support_foot_transform_current_.translation()(2) )/GRAVITY + ( com_vel_current(0)/(2*GRAVITY))*( com_vel_current(0)/(2*GRAVITY));
        
        // if (d_temp<0) d_temp = 0;
            d_temp_x = sqrt(d_temp_x);

        d(0) = com_vel_current(0)*d_temp_x;

        // y axis
        d_temp_y = ( com_pos_current(2) - support_foot_transform_current_.translation()(2) )/GRAVITY + ( com_vel_current(1)/(2*GRAVITY))*( com_vel_current(1)/(2*GRAVITY));
        
        // if (d_temp<0) d_temp = 0;
            d_temp_y = sqrt(d_temp_y);

        d(1) = com_vel_current(1)*d_temp_y;

        if(com_vel_current(0)*walking_speed_ < 0)
        {
            alpha_x = 0.2;
        }

        d_prime(0) = d(0) - alpha_x*com_vel_desired(0);
        // d_prime(1) = d(1) - 0.06*step_width_/walking_duration_*foot_contact_;
        d_prime(1) = d(1) - alpha_y*com_vel_desired(1);

        // d_prime(0) = DyrosMath::minmax_cut(d_prime(0), -0.3, 0.3);

        // if(foot_contact_ == 1) //left support
        // {
        //     d_prime(0) = DyrosMath::minmax_cut(d_prime(0), lfoot_transform_current_from_global_.translation()(0)-0.5, lfoot_transform_current_from_global_.translation()(0) + 0.5);
        //     d_prime(1) = DyrosMath::minmax_cut(d_prime(1), lfoot_transform_current_from_global_.translation()(1)-0.4, lfoot_transform_current_from_global_.translation()(1)-0.22);
        // }
        // else if (foot_contact_ == -1) // right support
        // {
        //     d_prime(0) = DyrosMath::minmax_cut(d_prime(0), rfoot_transform_current_from_global_.translation()(0)-0.5, rfoot_transform_current_from_global_.translation()(0) + 0.5);
        //     d_prime(1) = DyrosMath::minmax_cut(d_prime(1), rfoot_transform_current_from_global_.translation()(1)+0.22, rfoot_transform_current_from_global_.translation()(1)+0.4);
        // }
        

        target_foot_landing_from_pelv_ = com_pos_current.segment<2>(0) + d_prime;
        // target_foot_landing_from_pelv_ = d_prime; //consider pelvis origin(0, 0, 0) as a com
        
        // if(foot_contact_ == 1) //left support
        // {
        //     target_foot_landing_from_pelv_(0) = DyrosMath::minmax_cut(target_foot_landing_from_pelv_(0), lfoot_transform_current_from_global_.translation()(0)-0.5, lfoot_transform_current_from_global_.translation()(0) + 0.5);
        //     target_foot_landing_from_pelv_(1) = DyrosMath::minmax_cut(target_foot_landing_from_pelv_(1), lfoot_transform_current_from_global_.translation()(1)-0.4, lfoot_transform_current_from_global_.translation()(1)-0.20);
        // }
        // else if (foot_contact_ == -1) // right support
        // {
        //     target_foot_landing_from_pelv_(0) = DyrosMath::minmax_cut(target_foot_landing_from_pelv_(0), rfoot_transform_current_from_global_.translation()(0)-0.5, rfoot_transform_current_from_global_.translation()(0) + 0.5);
        //     target_foot_landing_from_pelv_(1) = DyrosMath::minmax_cut(target_foot_landing_from_pelv_(1), rfoot_transform_current_from_global_.translation()(1)+0.20, rfoot_transform_current_from_global_.translation()(1)+0.4);
        // }

        //retrun 
        // if(walking_phase_ < 3*switching_phase_duration_)
        // {
        //     swing_foot_pos_trajectory_from_global_ = swing_foot_transform_init_.translation();
        //     swing_foot_vel_trajectory_from_global_.setZero();
        //     swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
        // }
        // else 
        // {
        //     double ssp = (phase-3*switching_phase_duration_)/(walking_duration_ - 3*switching_phase_duration_);

        //     swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) + 
        //     (1 - ssp)*(swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0)) + 
        //     (ssp)*(target_foot_landing_from_pelv_ - support_foot_transform_current_.translation().segment<2>(0));

        //     swing_foot_vel_trajectory_from_global_.segment(0, 2) = (target_foot_landing_from_pelv_ - swing_foot_transform_init_.translation().segment<2>(0))/(walking_duration_ - 3*switching_phase_duration_);
        //     swing_foot_rot_trajectory_from_global_.setIdentity();  //no orientation
        // }

        double dsp_coeff = 1;

        if(walking_phase_ < dsp_coeff*switching_phase_duration_)
        {
            swing_foot_pos_trajectory_from_global_ = swing_foot_transform_init_.translation();
            swing_foot_vel_trajectory_from_global_.setZero();
            swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
        }
        else 
        {
            double ssp = (phase-dsp_coeff*switching_phase_duration_)/(1 - dsp_coeff*switching_phase_duration_);

            swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) + 
            (1 - ssp)*(swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0)) + 
            (ssp)*(target_foot_landing_from_pelv_ - support_foot_transform_current_.translation().segment<2>(0));

            swing_foot_vel_trajectory_from_global_.segment(0, 2) = (target_foot_landing_from_pelv_ - swing_foot_transform_init_.translation().segment<2>(0))/(walking_duration_ - 3*switching_phase_duration_);
            swing_foot_rot_trajectory_from_global_.setIdentity();  //no orientation
        }
        // if (swing_foot_pos_trajectory_from_global_.norm() > 0.7)
        // {
        //     double xy_proj;
        //     double l_max = 0.7;
        //     xy_proj = (pow(l_max,2) - pow(swing_foot_pos_trajectory_from_global_(2), 2))/(pow(swing_foot_pos_trajectory_from_global_(0), 2) + pow(swing_foot_pos_trajectory_from_global_(1), 2));
        //     swing_foot_pos_trajectory_from_global_(0) = xy_proj*swing_foot_pos_trajectory_from_global_(0);
        //     swing_foot_pos_trajectory_from_global_(1) = xy_proj*swing_foot_pos_trajectory_from_global_(1);
        //     cout<<"####################### swing foot trajectory is out of workspace #####################"<<endl;
        // }

        if(foot_contact_ == 1) //left support
        {
            swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), lfoot_transform_current_from_global_.translation()(0)-0.5, lfoot_transform_current_from_global_.translation()(0)+1.0);
            swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), lfoot_transform_current_from_global_.translation()(1)-0.8, lfoot_transform_current_from_global_.translation()(1)-0.2);
        }
        else if (foot_contact_ == -1) // right support
        {
            swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), rfoot_transform_current_from_global_.translation()(0)-0.5, rfoot_transform_current_from_global_.translation()(0)+1.0);
            swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), rfoot_transform_current_from_global_.translation()(1)+0.2, rfoot_transform_current_from_global_.translation()(1)+0.8);
        }

        // if (swing_foot_pos_trajectory_from_global_.norm() > 0.7)
        // {
        //     swing_foot_pos_trajectory_from_global_ = swing_foot_pos_trajectory_from_global_.normalized()*0.69;
        //     cout<<"####################### swing foot trajectory is out of workspace #####################"<<endl;
        // }

    }
    else
    {   
        swing_foot_pos_trajectory_from_global_ = swing_foot_transform_init_.translation();
        swing_foot_vel_trajectory_from_global_.setZero();
        swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
    }

    // if (swing_foot_pos_trajectory_from_global_.norm() > 0.7)
    // {
    //     swing_foot_pos_trajectory_from_global_ = swing_foot_pos_trajectory_from_global_.normalized()*0.69;
    //     cout<<"####################### swing foot trajectory is out of workspace #####################"<<endl;
    // }

    // double swing_foot_y_deviation = swing_foot_pos_trajectory_from_global_(1) - support_foot_transform_current_.translation()(1);
    // if( abs(swing_foot_y_deviation) < 0.2 )
    // {
    //     double mapping_xy = sqrt( (0.49 -  0.04)/(pow(swing_foot_pos_trajectory_from_global_(0),2) + pow(swing_foot_pos_trajectory_from_global_(2),2)) );
    //     // double mapping_xy =  (L_max - 1e-2)/R_r.norm();
    //     swing_foot_pos_trajectory_from_global_(0) *= mapping_xy;
    //     swing_foot_pos_trajectory_from_global_(2) *= mapping_xy;
    //     swing_foot_pos_trajectory_from_global_(1) = support_foot_transform_current_.translation()(1) - 0.2*foot_contact_;
    // }

    // if( int(current_time_*10)%5 == 0 )
    if(true)
    {
        // cout<<"current_time_: "<<current_time_<<endl;
        // cout<<"stance_start_time_: "<<stance_start_time_<<endl;
        // cout<<"start_time_: "<<start_time_<<endl;
        // cout<<"tc.command_time: "<<tc.command_time<<endl;
        // cout<<"foot_contact_: "<<foot_contact_<<endl;
        // cout<<"walking_phase_: "<<walking_phase_<<endl;
        // cout<<"first_step_flag_: "<<first_step_flag_<<endl;
        // cout<<"first_step_trigger_: "<<first_step_trigger_<<endl;
        // cout<<"foot_swing_trigger_: "<<foot_swing_trigger_<<endl;
        // cout<<"stop_walking_trigger_: "<<stop_walking_trigger_<<endl;
        // cout<<"support_foot_transform_current_.translation()(2):"<< support_foot_transform_current_.translation()(2)<<endl;
        // cout<<"d_temp_y:"<< d_temp_y<<endl;
        // cout<<"d_prime:"<< d_prime<<endl;
        // cout<<"com_vel_desired_:"<< com_vel_desired<<endl;        
        // cout<<"com_pos_desired_:"<< com_pos_desired_<<endl;
        // cout<<"com_vel_current:"<< com_vel_current<<endl;
        // cout<<"com_pos_current_:"<< com_pos_current<<endl;
        // cout<<"com_pos_init_:"<< com_pos_init_<<endl;
        // cout<<"target_foot_landing_from_pelv_:"<< target_foot_landing_from_pelv_<<endl;
        // cout<<"swing_foot_pos_trajectory_from_global_:"<< swing_foot_pos_trajectory_from_global_<<endl;
        // cout<<"swing_foot_transform_current_.translation():"<< swing_foot_transform_current_.translation()<<endl;
    }
}

Eigen::VectorQd TocabiController::comVelocityControlCompute(Wholebody_controller &wc)
{

    /////////////////////////////////////////////////////////////////////////////
    //     const int arm_task_number = 6;
    // const int arm_dof = 8;
    // ////////// CoM Control //////////////////////
    // wc_.set_contact(tocabi_, 1, 1);
    // task_number = 6;
    // J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
    // f_star.setZero(task_number);

    // J_task = tocabi_.link_[COM_id].Jac;
    // J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac_COM_p;
    // J_task.block(0, 21, 6, arm_dof).setZero(); // Exclude Left Arm Jacobian
    // J_task.block(0, 31, 6, arm_dof).setZero(); // Exclude Right Arm Jacobian

    // tocabi_.link_[COM_id].x_desired = tocabi_.link_[COM_id].x_init;
    // tocabi_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);

    // // f_star = wc_.getfstar6d(tocabi_, COM_id);
    // torque_grav.setZero();
    // torque_task = wc_.task_control_torque_QP2(tocabi_, J_task, f_star);
    //////////////////////////////////////////////////////////////////

    Eigen::VectorQd torque;

    Eigen::VectorXd f_star;
    Eigen::MatrixXd J_task;
    const int task_dof = 6;

    J_task.setZero(task_dof, MODEL_DOF_VIRTUAL);
    f_star.setZero(task_dof);

    wc.set_contact(tocabi_, 1, 1);
    if(foot_swing_trigger_ == true)
    {
        if( (foot_contact_ == 1) ) 
        {
            wc.set_contact(tocabi_, 1, 0);
        }
        else if ( (foot_contact_ == -1) )
        {
            wc.set_contact(tocabi_, 0, 1);
        }        
    }

    ////////////// Set J_task  ////////////////
    J_task = tocabi_.link_[COM_id].Jac;
    // J_task = tocabi_.link_[COM_id].Jac.block(0, 0, 2, MODEL_DOF_VIRTUAL);
    J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[COM_id].Jac_COM_p;
    // J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = tocabi_.link_[Head].Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);
    J_task.block(0, 21, 6, 8).setZero(); // Exclude Left hand Jacobian
    J_task.block(0, 31, 6, 8).setZero(); // Exclude Right hand Jacobian
    // J_task.block(0, 29, 6, 2).setZero(); // Exclude Head Jacobian

    if(foot_swing_trigger_ == true)
    {
        if(foot_contact_ == 1) //left support
        {
            J_task.block(0, 12, 6, 8).setZero(); //Exclude Rifht Leg Jacobian
        }
        else if(foot_contact_ == -1) //right support
        {
            J_task.block(0, 6, 6, 8).setZero(); //Exclude Left Leg Jacobian
        }
    }
    ////////////////////////////////////////////

    ////////////// Set f_start  ////////////////
    double kp = 2500;            // 2500(reference paper)
    double kv = 100;             // 100(reference paper)

    // f_star(0) = kv*(com_vel_desired_(0) - com_vel_current_(0)) + kp*(com_pos_desired_(1)- com_pos_current_(1));                                                          //X axis D control
    // f_star(1) = kv*(com_vel_desired_(1) - com_vel_current_(1)) + kp*(com_pos_desired_(1)- com_pos_current_(1));          //Y axis PD control
    // f_star(2) = kv*(com_vel_desired_(2) - com_vel_current_(2)) + kp*(com_pos_desired_(2)- com_pos_current_(2));
    tocabi_.link_[COM_id].x_traj = com_pos_desired_;
    tocabi_.link_[COM_id].v_traj = com_vel_desired_;
    tocabi_.link_[COM_id].a_traj.setZero();
    tocabi_.link_[COM_id].r_traj = Eigen::Matrix3d::Identity();
    tocabi_.link_[COM_id].w_traj.setZero();

    // tocabi_.link_[Head].r_traj = Eigen::Matrix3d::Identity();
    // tocabi_.link_[Head].w_traj = Eigen::Vector3d::Zero();
    // f_star.segment(3, 3) = wc.getfstar_rot(tocabi_, COM_id);
    // f_star.segment(3, 3) = wc.getfstar_rot(tocabi_, Head);
    f_star = wc.getfstar6d(tocabi_, COM_id);
    f_star(2) = 0;
    f_star(5) = 0;
    /////////////////////////////////////////////
    
    double contact_dist_ratio = 0;
    // if( (foot_swing_trigger_ == false) && (walking_speed_ != 0))
    // {
    //     if(foot_contact_ == 1)
    //     {
    //         contact_dist_ratio = 1;
    //     }
    //     else if(foot_contact_ == -1)
    //     {
    //         contact_dist_ratio = -1;
    //     }
    // }

    torque = wc.task_control_torque_QP_dg(tocabi_, J_task, f_star, 1);  //jacobian control + gravity torque
    
    // if(int(control_time_) %2 ==0)
    // {
    //     cout<<"Com Vel torque: \n"<< torque <<endl;
    //     cout<<"f_com: \n"<< f_com <<endl;
    // }
    // torque = tuneTorqueForZMPSafety( torque );        //turn off velocity tuning if the zmp is outside of the foot


    return torque;
}

Eigen::VectorQd TocabiController::comVelocityControlCompute2(Wholebody_controller &wc)
{
    Eigen::VectorQd torque;
    Eigen::VectorQd torque_g;

    Eigen::VectorXd f_star;
    Eigen::MatrixXd J_task;
    VectorQd torque_r_vel_tun;
    VectorQd torque_l_vel_tun;
    const int task_dof = 3;
    torque.setZero();
    torque_g.setZero();
    J_task.setZero(task_dof, MODEL_DOF_VIRTUAL);
    f_star.setZero(task_dof);

    wc.set_contact(tocabi_, 0, 0); // for graviti torque calc
    // if(foot_swing_trigger_ == true)
    // {
    //     if( (foot_contact_ == 1) ) 
    //     {
    //         wc.set_contact(tocabi_, 1, 0);

    //     }
    //     else if ( (foot_contact_ == -1) )
    //     {
    //         wc.set_contact(tocabi_, 0, 1);
    //     }        
    // }
    torque_g = wc.gravity_compensation_torque(tocabi_, true);

    ////////////// Set f_start  ////////////////
    double kp = 3600;            // 2500(reference paper)
    double kv = 120;             // 100(reference paper)

    // com_pos_desired_(1) = com_pos_init_(0) + sin(2*M_PI/8*current_time_-tc.command_time);
    // com_vel_desired_(1) = M_PI/2*cos(2*M_PI/8*current_time_-tc.command_time);
    f_star(0) = kv*(com_vel_desired_(0) - (com_vel_current_)(0)) + kp*(com_pos_desired_(0)- com_pos_current_(0));// + tocabi_.com_.mass*com_acc_desired_(0);                                                          //X axis D control
    f_star(1) = kv*(com_vel_desired_(1) - (com_vel_current_)(1)) + kp*(com_pos_desired_(1)- com_pos_current_(1));// + tocabi_.com_.mass*com_acc_desired_(1);          //Y axis PD control
    f_star(2) = 0;
    // f_star(2) = kv*(com_vel_desired_(2) - com_vel_current_(2)) + kp*(com_pos_desired_(2)- com_pos_current_(2));
    // tocabi_.link_[COM_id].x_traj = com_pos_desired_;
    // tocabi_.link_[COM_id].v_traj = com_vel_desired_;
    // tocabi_.link_[COM_id].a_traj.setZero();
    // tocabi_.link_[COM_id].r_traj = Eigen::Matrix3d::Identity();
    // tocabi_.link_[COM_id].w_traj.setZero();

    // tocabi_.link_[Head].r_traj = Eigen::Matrix3d::Identity();
    // tocabi_.link_[Head].w_traj = Eigen::Vector3d::Zero();
    // f_star.segment(2, 3) = wc.getfstar_rot(tocabi_, COM_id);
    // f_star.segment(3, 3) = wc.getfstar_rot(tocabi_, Head);
    // f_star = wc.getfstar6d(tocabi_, COM_id);

    // f_star_xy_ = 0.3*f_star + 0.7*f_star_xy_pre_;
    /////////////////////////////////////////////
    
    /////////////////////JACOBIAN///////////////////////////////////////


    MatrixXd lfoot_to_com_jac_from_global;
    MatrixXd rfoot_to_com_jac_from_global;
    lfoot_to_com_jac_from_global.setZero(3, MODEL_DOF_VIRTUAL);
    rfoot_to_com_jac_from_global.setZero(3, MODEL_DOF_VIRTUAL);
    Matrix6d adjoint_pelv_to_ankle;
    adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
    adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - lfoot_transform_current_from_global_.translation());
    adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

    lfoot_to_com_jac_from_global = (adjoint_pelv_to_ankle*tocabi_.link_[Left_Foot].Jac).block(0, 0, 3, MODEL_DOF_VIRTUAL) + tocabi_.link_[COM_id].Jac_COM_p;
    lfoot_to_com_jac_from_global.block(0, 12, 3, 6).setZero();
    lfoot_to_com_jac_from_global.block(0, 21, 3, 8).setZero();
    lfoot_to_com_jac_from_global.block(0, 31, 3, 8).setZero();

    adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
    adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - rfoot_transform_current_from_global_.translation());
    adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

    rfoot_to_com_jac_from_global = (adjoint_pelv_to_ankle*tocabi_.link_[Right_Foot].Jac).block(0, 0, 3, MODEL_DOF_VIRTUAL) + tocabi_.link_[COM_id].Jac_COM_p;
    rfoot_to_com_jac_from_global.block(0, 6, 3, 6).setZero();
    rfoot_to_com_jac_from_global.block(0, 21, 3, 8).setZero();
    rfoot_to_com_jac_from_global.block(0, 31, 3, 8).setZero();

    torque_l_vel_tun = (lfoot_to_com_jac_from_global.transpose()*f_star).segment(6, MODEL_DOF);
    torque_r_vel_tun = (rfoot_to_com_jac_from_global.transpose()*f_star).segment(6, MODEL_DOF);

    // J_task = jac_com_pos_;
    /////////////////////////////////////////////////////////////////////////////

    // torque = wc.task_control_torque_QP_dg(tocabi_, J_task, f_star, 0);
    // torque = (J_task.transpose()*f_star).segment(6, MODEL_DOF);
    // if(foot_swing_trigger_ == true)
    // {
    //     wc.set_contact(tocabi_, 1, 0);

    //     torque_l_vel_tun = wc.task_control_torque(tocabi_, J_task, f_star);
    //     torque_l_vel_tun.segment(6, 6).setZero();
    //     torque_l_vel_tun.segment(15, 8).setZero();
    //     torque_l_vel_tun.segment(25, 8).setZero();

    //     wc.set_contact(tocabi_, 0, 1);

    //     torque_r_vel_tun = wc.task_control_torque(tocabi_, J_task, f_star);
    //     torque_r_vel_tun.segment(0, 6).setZero();
    //     torque_r_vel_tun.segment(15, 8).setZero();
    //     torque_r_vel_tun.segment(25, 8).setZero();
    // }
    // else
    // {
    //     wc.set_contact(tocabi_, 1, 1);

    //     torque_l_vel_tun = wc.task_control_torque(tocabi_, J_task, f_star);
    //     torque_l_vel_tun.segment(6, 6).setZero();
    //     torque_l_vel_tun.segment(15, 8).setZero();
    //     torque_l_vel_tun.segment(25, 8).setZero();

    //     torque_r_vel_tun = wc.task_control_torque(tocabi_, J_task, f_star);
    //     torque_r_vel_tun.segment(0, 6).setZero();
    //     torque_r_vel_tun.segment(15, 8).setZero();
    //     torque_r_vel_tun.segment(25, 8).setZero();
    // }
    

   
    ////////////TORQUE CLACULATION/////////////////////////////////
    double lfoot_torque_g_switch;
    double rfoot_torque_g_switch;
    double lfoot_task_torque_switch;
    double rfoot_task_torque_switch;


    if(foot_swing_trigger_ == true)
    {
        double vel_tune_switching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

        if(foot_contact_ == 1) //left support
        {   
            if(first_step_trigger_ == true)
            {
                lfoot_task_torque_switch = 1;
                rfoot_task_torque_switch = 1-vel_tune_switching;

                lfoot_torque_g_switch = 0;
                rfoot_torque_g_switch = vel_tune_switching;
            }
            else
            {
                lfoot_task_torque_switch = vel_tune_switching;
                rfoot_task_torque_switch = 1-vel_tune_switching;

                lfoot_torque_g_switch = 1-vel_tune_switching;
                rfoot_torque_g_switch = vel_tune_switching;
                
            }
        }
        else if(foot_contact_ == -1)
        {
            if(first_step_trigger_ == true)
            {
                lfoot_task_torque_switch = 1-vel_tune_switching;
                rfoot_task_torque_switch = 1;

                lfoot_torque_g_switch = vel_tune_switching;
                rfoot_torque_g_switch = 0;
            }
            else
            {
                lfoot_task_torque_switch = 1-vel_tune_switching;
                rfoot_task_torque_switch = vel_tune_switching;

                lfoot_torque_g_switch = vel_tune_switching;
                rfoot_torque_g_switch = 1-vel_tune_switching;
                
            }
        }
    }
    else
    {
        if(stop_walking_trigger_ == true) 
        {
            if(foot_contact_ == 1) // left support previously
            {
                lfoot_task_torque_switch = 1;
                rfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_,stance_start_time_ + walking_duration_*switching_phase_duration_, 0, 1, 0, 0);

                lfoot_torque_g_switch = 0;
                rfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_*switching_phase_duration_, 1, 0, 0, 0);

            }
            else if(foot_contact_ == -1)
            {
                lfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_*switching_phase_duration_, 0, 1, 0, 0);
                rfoot_task_torque_switch = 1;

                lfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_*switching_phase_duration_, 1, 0, 0, 0);
                rfoot_torque_g_switch = 0;
            }
        }
        else
        {
            lfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1*walking_duration_, 0, 1, 0, 0);
            rfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1*walking_duration_, 0, 1, 0, 0);

            lfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1*walking_duration_, 1, 0, 0, 0);
            rfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1*walking_duration_, 1, 0, 0, 0);
        }
    }    
    
    torque_g.segment(0, 6) = torque_g.segment(0, 6)*lfoot_torque_g_switch;
    torque_g.segment(6, 6) = torque_g.segment(6, 6)*rfoot_torque_g_switch;
    
    
    



    // if( int(walking_duration_*100)%10 == 0 )
    // cout<<"walking_phase_: \n"<<walking_phase_<<endl;
    torque += torque_l_vel_tun*lfoot_task_torque_switch + torque_r_vel_tun*rfoot_task_torque_switch;
    // torque(0) =0;
    // torque(6) =0;
    /////////////////////////////////////////////////////////////////

    // torque = wc.task_control_torque_QP2(tocabi_, jac_com_xy, f_com);  //jacobian control + gravity torque
    
    // if(int(control_time_) %2 ==0)
    // {
    //     cout<<"Com Vel torque: \n"<< torque <<endl;
    //     cout<<"f_com: \n"<< f_com <<endl;
    // }
    
    torque += torque_g;

    // if( walking_phase_> 0.1)
    {
        // cout<<"pelv_rot_current_: \n"<<pelv_rot_current_<<endl;
        // cout<<"pelv_rpy_current_: \n"<<pelv_rpy_current_<<endl;
        // cout<<"f_star: \n"<<f_star<<endl;
        // cout<<"torque: \n"<<torque<<endl;
        // cout<<"torque_g: \n"<<torque_g<<endl;
        // cout<<"lfoot_task_torque_switch: \n"<<lfoot_task_torque_switch<<endl;
        // cout<<"rfoot_task_torque_switch: \n"<<rfoot_task_torque_switch<<endl;
        // cout<<"torque_l_vel_tun: \n"<<torque_l_vel_tun<<endl;
        // cout<<"torque_r_vel_tun: \n"<<torque_r_vel_tun<<endl;
        // cout<<"lfoot_to_com_jac_from_global: \n"<<lfoot_to_com_jac_from_global<<endl;
        // cout<<"test_com_jac: \n"<<test_com_jac<<endl;
    }

    // if( (walking_phase_ > 0.2) && (walking_phase_ < 0.7))
    torque = tuneTorqueForZMPSafety( torque );        //turn off velocity tuning if the zmp is outside of the foot

    return torque;
}

Eigen::VectorQd TocabiController::jointTrajectoryPDControlCompute(Wholebody_controller &wc)
{
    Eigen::VectorQd torque;
    Eigen::Vector12d desired_q_leg;
    Eigen::Isometry3d pelv_transform_from_global;
    Eigen::Isometry3d lleg_transform_from_global;
    Eigen::Isometry3d rleg_transform_from_global;
    Eigen::Isometry3d lleg_transform_target;
    Eigen::Isometry3d rleg_transform_target;

    double default_stance_foot_z_from_pelv = -0.349*(cos(0.02) + cos(0.12))-0.1225;
    // lleg_transform_target.translation()(0) = -0.015;
    // lleg_transform_target.translation()(1) = 0.1025;
    // lleg_transform_target.translation()(2) = default_stance_foot_z_from_pelv;
    lleg_transform_target.linear().setIdentity();
    // rleg_transform_target.translation()(0) = -0.015;
    // rleg_transform_target.translation()(1) = -0.1025;
    // rleg_transform_target.translation()(2) = default_stance_foot_z_from_pelv;
    rleg_transform_target.linear().setIdentity();
    lleg_transform_target.translation() = lfoot_transform_current_from_global_.translation();
    rleg_transform_target.translation() = rfoot_transform_current_from_global_.translation();
    
    // cout<<"lfoot_transform_init_from_global_.translation(): \n"<<lfoot_transform_init_from_global_.translation()<<endl;
    // cout<<"lfoot_transform_init_from_global_.linear(): \n"<<lfoot_transform_init_from_global_.linear()<<endl;
    // cout<<"rfoot_transform_init_from_global_.translation(): \n"<<rfoot_transform_init_from_global_.translation()<<endl;
    // cout<<"rfoot_transform_init_from_global_.linear(): \n"<<rfoot_transform_init_from_global_.linear()<<endl;
    /////////////////////////////////PELVIS/////////////////////////////////////


    pelv_transform_from_global.translation().setZero();
    pelv_transform_from_global.linear() = pelv_rot_current_yaw_aline_; //
    // pelv_transform_from_global.linear() = pelv_yaw_rot_current_from_global_;
    // pelv_transform_from_global.linear().setIdentity();
    Vector3d phi_pelv;
    Vector3d ang_vel_pelv;
    Vector3d torque_pelv;
    VectorQd torque_stance_hip;
    VectorQd torque_swing_assist;

    torque_pelv.setZero();
    torque_stance_hip.setZero();
    torque_swing_assist.setZero();

    phi_pelv = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
    ang_vel_pelv = pelv_yaw_rot_current_from_global_.transpose()*tocabi_.link_[Pelvis].w;
    
    double kpa_pelv = 10000;  //angle error gain
    double kva_pelv = 200;   //angular velocity gain
    torque_pelv = kpa_pelv*phi_pelv - kva_pelv*ang_vel_pelv;
    Vector3d axis;
    double angle;
    // angle = (phi_pelv.norm())*dt_ ;
    // axis = phi_pelv.normalized();

    Eigen::AngleAxisd aa(pelv_rot_current_yaw_aline_.transpose());
    angle = aa.angle();
    axis = aa.axis();
    if( walking_phase_>0.1 )
    {
        // cout <<"phi_pelv: \n"<<phi_pelv<<endl;
        // cout <<"torque_pelv: \n"<<torque_pelv<<endl;
        // cout <<"axis: \n"<<axis<<endl;
    }

    // Matrix3d pelv_rot;
    // pelv_rot = pelv_yaw_rot_current_from_global_.transpose()*pelv_rot_current_*Eigen::AngleAxisd(angle*100*dt_, axis);
    // pelv_transform_from_global.linear() = pelv_rot;


    
    // desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3)*dt_;

    //////////////////////////////////////////////////////////////////////////////////////////





    Eigen::MatrixXd kp_joint(MODEL_DOF,1);
    Eigen::MatrixXd kv_joint(MODEL_DOF,1);

    Eigen::MatrixXd kp_stiff_joint(MODEL_DOF,1);
    Eigen::MatrixXd kv_stiff_joint(MODEL_DOF,1);
    Eigen::MatrixXd kp_soft_joint(MODEL_DOF,1);
    Eigen::MatrixXd kv_soft_joint(MODEL_DOF,1);
    double swing_pd_switch;
    for(int i = 0; i < MODEL_DOF; i++)
    {
        kp_joint(i) = 16;
        kv_joint(i) = 8;
    }

    // for(int i = 0; i<12; i++)      //Leg
    // {
    //     kp_joint(i) = 600;
    //     kv_joint(i) = 50;
    // }
    // kp_joint(1) = 8100;
    // kv_joint(1) = 180;
    // kp_joint(2) = 6400;
    // kv_joint(2) = 160;

    // kp_joint(7) = 8100;
    // kv_joint(7) = 180;
    // kp_joint(8) = 6400;
    // kv_joint(8) = 160;


    for(int i = 0; i<3; i++)      //Waist Joint Gains
    {
        kp_joint(12 + i) = 50;
        kv_joint(12 + i) = 10;
    }
    kp_joint(12) = 50;
    kv_joint(12) = 14;
    kp_joint(13) = 50;
    kv_joint(13) = 14;
    kp_joint(14) = 50;
    kv_joint(14) = 14;

    for(int i = 0; i<2; i++)      //Head Joint Gains
    {
        kp_joint(23 + i) = 16;
        kv_joint(23 + i) = 8;
    }
    
    //stiff
    kp_stiff_joint(0) = 900;   //R hip yaw joint gain
    kv_stiff_joint(0) = 60;
    kp_stiff_joint(1) = 2500;   //L hip roll joint gain
    kv_stiff_joint(1) = 100;
    kp_stiff_joint(2) = 2500;   //L hip pitch joint gain
    kv_stiff_joint(2) = 100;

    kp_stiff_joint(3) = 2500;   //L knee joint gain
    kv_stiff_joint(3) = 100;

    kp_stiff_joint(4) = 900;    //L ankle pitch joint gain
    kv_stiff_joint(4) = 60;
    kp_stiff_joint(5) = 900;    //L ankle roll joint gain
    kv_stiff_joint(5) = 60;

    kp_stiff_joint(6) = 900;   //R hip yaw joint gain
    kv_stiff_joint(6) = 60;
    kp_stiff_joint(7) = 2500;   //R hip roll joint gain
    kv_stiff_joint(7) = 100;
    kp_stiff_joint(8) = 2500;   //R hip pitch joint gain
    kv_stiff_joint(8) = 100;

    kp_stiff_joint(9) = 2500;   //R knee joint gain
    kv_stiff_joint(9) = 100;

    kp_stiff_joint(10) = 900;   //R ankle pitch joint gain
    kv_stiff_joint(10) = 60;
    kp_stiff_joint(11) = 900;   //R ankle roll joint gain
    kv_stiff_joint(11) = 60;


    //soft
    kp_soft_joint(0) = 900;     //L hip yaw joint gain
    kv_soft_joint(0) = 60;
    kp_soft_joint(1) = 400;     //L hip roll joint gain
    kv_soft_joint(1) = 40;
    kp_soft_joint(2) = 400;     //L hip pitch joint gain
    kv_soft_joint(2) = 40;

    kp_soft_joint(3) = 400;     //L knee joint gain
    kv_soft_joint(3) = 40;

    kp_soft_joint(4) = 64;     //L ankle pitch joint gain
    kv_soft_joint(4) = 16;
    kp_soft_joint(5) = 64;     //L ankle roll joint gain
    kv_soft_joint(5) = 16;
    
    kp_soft_joint(6) = 900;     //R hip yaw joint gain
    kv_soft_joint(6) = 60;
    kp_soft_joint(7) = 400;     //R hip roll joint gain
    kv_soft_joint(7) = 40;
    kp_soft_joint(8) = 400;     //R hip pitch joint gain
    kv_soft_joint(8) = 40;

    kp_soft_joint(9) = 400;     //R knee joint gain
    kv_soft_joint(9) = 40;

    kp_soft_joint(10) = 64;    //R ankle pitch joint gain
    kv_soft_joint(10) = 16;
    kp_soft_joint(11) = 64;    //R ankle roll joint gain
    kv_soft_joint(11) = 16;

    for(int i = 0; i<12; i++)      //Leg
    {
        kp_joint(i) = kp_stiff_joint(i);
        kv_joint(i) = kv_stiff_joint(i);
    }
    torque.setZero();
    
    double support_transition_phase = 0.2;
    //////////////////////////SWING FOOT & STANCE FOOT ankle and knee joints///////////////////////
    if(foot_swing_trigger_ == true)
    {
        if(foot_contact_ == -1) //right support
        {
            lleg_transform_from_global.translation() = swing_foot_pos_trajectory_from_global_;
            lleg_transform_from_global.linear() = swing_foot_rot_trajectory_from_global_;
            
            rleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(0), 0, 0, rleg_transform_target.translation()(0), 0, 0)(0);
            rleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(1), 0, 0, rleg_transform_target.translation()(1), 0, 0)(0);
            rleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(2), 0, 0, rleg_transform_target.translation()(2), 0, 0)(0);

            rleg_transform_from_global.linear() = rleg_transform_target.linear();
            // rleg_transform_from_global = rleg_transform_target;
            computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

            for(int i = 1; i<4; i++) //hip and knee
            {
                desired_q_(i) = desired_q_leg(i); //left swing foot
                // kp_joint(i) = 900; //swing foot gain
                // kv_joint(i) = 60;

                // if(walking_phase_ < 0.1)
                // {
                //     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
                //     cout<<"desired_q_leg: \n"<<desired_q_leg<<endl;
                //     cout<<"lleg_transform_from_global.translation(): \n" <<lleg_transform_from_global.translation()<<endl;
                //     cout<<"lleg_transform_from_global.rotation(): \n" <<lleg_transform_from_global.rotation()<<endl;
                //     cout<<"rleg_transform_from_global.translation(): \n" <<rleg_transform_from_global.traznslation()<<endl;
                //     cout<<"rleg_transform_from_global.rotation(): \n" <<rleg_transform_from_global.rotation()<<endl;
                //     cout<<"pelv_transform_from_global.translation(): \n" <<pelv_transform_from_global.translation()<<endl;
                //     cout<<"pelv_transform_from_global.rotation(): \n" <<pelv_transform_from_global.rotation()<<endl;
                // }
            }

            desired_q_(0) = 0.5*motion_q_(6) + 0.5*pre_desired_q_(6);
            Vector3d phi_swing_ankle;
            phi_swing_ankle = -DyrosMath::getPhi(tocabi_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
            // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

            desired_q_dot_(4) = 200*phi_swing_ankle(1); //swing ankle pitch
            desired_q_dot_(5) = 200*phi_swing_ankle(0); //swing ankle roll           
            desired_q_(4) = current_q_(4) + desired_q_dot_(4)*dt_;
            desired_q_(5) = current_q_(5) + desired_q_dot_(5)*dt_;

            Vector3d phi_support_ankle;
            phi_support_ankle = -DyrosMath::getPhi(tocabi_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
            
            desired_q_dot_(10) = 200*phi_support_ankle(1);
            desired_q_dot_(11) = 200*phi_support_ankle(0);
            desired_q_(10) = current_q_(10) + desired_q_dot_(10)*dt_;
            desired_q_(11) = current_q_(11) + desired_q_dot_(11)*dt_;

            // low pass filter for suppport foot target position
            desired_q_(6) = 0.5*motion_q_(6) + 0.5*pre_desired_q_(6); //right support foot hip yaw
            desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, init_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
            // desired_q_(9) = 0.5*motion_q_(9) + 0.5*pre_desired_q_(9); //right support foot
            // desired_q_(9) = desired_q_leg(9); //right support foot knee
            // desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
            // desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
            // desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right support foot ankle pitc
            // desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right support foot anlke roll            
            // desired_q_(10) = desired_q_leg(10) ;
            // desired_q_(11) = desired_q_leg(11) ;

            for(int i = 1; i<6; i++)
            {
                if(kp_joint(i+6) == kp_soft_joint(i+6))
                {
                    kp_joint(i+6) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint(i+6), kp_stiff_joint(i+6), 0, 0); //support foot
                }

                if(kp_joint(i) == kp_stiff_joint(i))
                {
                    kp_joint(i) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint(i), kp_soft_joint(i), 0, 0); //swing foot
                }    
            }
            // kp_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
            // kv_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);    

            // kp_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
            // kv_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
            // kp_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
            // kv_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);

            /////////////////Swing Assist Torque/////////////////////////////////////////
            // left foot swing height assist feed forward torque on hip roll, hip pitch, knee pitch
            Eigen::VectorXd f_star;
            f_star.setZero(3);
            f_star(2) = swing_foot_acc_trajectory_from_global_(2);

            
            // torque_swing_assist.segment(1, 3) = jac_lfoot_.block(2, 7, 1, 3).transpose()*swing_foot_acc_trajectory_from_global_(2); 
            // torque_swing_assist.segment(1, 3) = (jac_lfoot_.transpose()).block(7, 0, 3, 6)*tocabi_.lambda.block(0, 2, 6, 1)*swing_foot_acc_trajectory_from_global_(2); 
            
            torque_swing_assist.segment(1, 3) = wc.task_control_torque(tocabi_, jac_lfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(1, 3);

            /////////////////////////////////////////////////////////////////////////////

            swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

            if(first_step_trigger_ == true)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = swing_pd_switch;
                pd_control_mask_(2) = swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 0;
                pd_control_mask_(8) = 0;

                torque_stance_hip(2) = -torque_pelv(1)*(1-swing_pd_switch); // left hip pitch
                torque_stance_hip(1) = -torque_pelv(0)*(1-swing_pd_switch); // left hip roll

                torque_stance_hip(8) = -torque_pelv(1); // right hip pitch
                torque_stance_hip(7) = -torque_pelv(0); // right hip roll
            }
            else
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = swing_pd_switch;
                pd_control_mask_(2) = swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 1-swing_pd_switch;
                pd_control_mask_(8) = 1-swing_pd_switch;

                torque_stance_hip(2) = -torque_pelv(1)*(1-swing_pd_switch); // left hip pitch
                torque_stance_hip(1) = -torque_pelv(0)*(1-swing_pd_switch); // left hip roll

                torque_stance_hip(8) = -torque_pelv(1)*swing_pd_switch; // right hip pitch
                torque_stance_hip(7) = -torque_pelv(0)*swing_pd_switch; // right hip roll
            }

        }
        else if(foot_contact_ == 1) //left support
        {

            rleg_transform_from_global.translation() = swing_foot_pos_trajectory_from_global_;
            rleg_transform_from_global.linear() = swing_foot_rot_trajectory_from_global_;

            lleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(0), 0, 0, lleg_transform_target.translation()(0), 0, 0)(0);
            lleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(1), 0, 0, lleg_transform_target.translation()(1), 0, 0)(0);
            lleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(2), 0, 0, lleg_transform_target.translation()(2), 0, 0)(0);
            
            lleg_transform_from_global.linear() = lleg_transform_target.linear();
            // lleg_transform_from_global = lleg_transform_target;

            computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

            for(int i = 7; i<10; i++)
            {
                desired_q_(i) = desired_q_leg(i); //right swing foot
                // kp_joint(i) = 900; //swing foot gain
                // kv_joint(i) = 60;

                // desired_q_(i-6) = 0.5*motion_q_(i-6) + 0.5*pre_desired_q_(i-6); //left support foot
                // if(walking_phase_ < 0.1)
                // {
                //     cout<<"desired_q_leg: \n"<<desired_q_leg<<endl;
                //     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
                //     cout<<"lleg_transform_from_global.translation(): \n" <<lleg_transform_from_global.translation()<<endl;
                //     cout<<"lleg_transform_from_global.rotation(): \n" <<lleg_transform_from_global.rotation()<<endl;
                //     cout<<"rleg_transform_from_global.translation(): \n" <<rleg_transform_from_global.translation()<<endl;
                //     cout<<"rleg_transform_from_global.rotation(): \n" <<rleg_transform_from_global.rotation()<<endl;
                //     cout<<"pelv_transform_from_global.translation(): \n" <<pelv_transform_from_global.translation()<<endl;
                //     cout<<"pelv_transform_from_global.rotation(): \n" <<pelv_transform_from_global.rotation()<<endl;                   
                // }
            }

            desired_q_(6) = 0.5*motion_q_(6) + 0.5*pre_desired_q_(6);
            Vector3d phi_swing_ankle;
            phi_swing_ankle = -DyrosMath::getPhi(tocabi_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
            // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

            desired_q_dot_(10) = 200*phi_swing_ankle(1); //swing ankle pitch
            desired_q_dot_(11) = 200*phi_swing_ankle(0); //swing ankle roll           
            desired_q_(10) = current_q_(10) + desired_q_dot_(10)*dt_;
            desired_q_(11) = current_q_(11) + desired_q_dot_(11)*dt_;
            
            Vector3d phi_support_ankle;
            phi_support_ankle = -DyrosMath::getPhi(tocabi_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
            
            desired_q_dot_(4) = 200*phi_support_ankle(1);
            desired_q_dot_(5) = 200*phi_support_ankle(0);
            desired_q_(4) = current_q_(4) + desired_q_dot_(4)*dt_;
            desired_q_(5) = current_q_(5) + desired_q_dot_(5)*dt_;

            desired_q_(0) = 0.5*motion_q_(0) + 0.5*pre_desired_q_(0); //left support foot hip yaw
            desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, init_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
            // desired_q_(3) = 0.5*motion_q_(3) + 0.5*pre_desired_q_(3); //left support foot knee
            // desired_q_(3) = desired_q_leg(3); //left support foot knee
            // desired_q_(4) = 0.5*desired_q_leg(4) + 0.5*pre_desired_q_(4); //left support foot ankle pitch
            // desired_q_(5) = 0.5*desired_q_leg(5) + 0.5*pre_desired_q_(5); //left support foot anlke roll
            // desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //left support foot ankle pitch
            // desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //left support foot anlke roll
            // desired_q_(4) = desired_q_leg(4) ;
            // desired_q_(5) = desired_q_leg(5) ;

            for(int i = 1; i<6; i++)
            {
                if(kp_joint(i) == kp_soft_joint(i))
                {
                    kp_joint(i) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint(i), kp_stiff_joint(i), 0, 0); //support foot
                }

                if(kp_joint(i+6) == kp_stiff_joint(i+6))
                {
                    kp_joint(i+6) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint(i+6), kp_soft_joint(i+6), 0, 0); //swing foot
                }    
            }
            // kp_joint(9) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
            // kv_joint(9) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);

            // kp_joint(10) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
            // kv_joint(10) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
            // kp_joint(11) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
            // kv_joint(11) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);


            /////////////////Swing Assist Torque/////////////////////////////////////////
            // right foot swing height assist feed forward torque on hip roll, hip pitch, knee pitch
            Eigen::VectorXd f_star;
            f_star.setZero(3);
            f_star(2) = swing_foot_acc_trajectory_from_global_(2);


            torque_swing_assist.segment(7, 3) = wc.task_control_torque(tocabi_, jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(7, 3);

            // torque_swing_assist.segment(7, 3) = (jac_lfoot_.transpose()).block(13, 0, 3, 6)*tocabi_.lambda.block(0, 2, 6, 1)*swing_foot_acc_trajectory_from_global_(2); 
            /////////////////////////////////////////////////////////////////////////////

            swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);


            if(first_step_trigger_ == true)
            {

                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 0;
                pd_control_mask_(2) = 0;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = swing_pd_switch;
                pd_control_mask_(8) = swing_pd_switch;


                torque_stance_hip(2) = -torque_pelv(1); // left hip pitch
                torque_stance_hip(1) = -torque_pelv(0); // left hip roll

                torque_stance_hip(8) = -torque_pelv(1)*(1-swing_pd_switch); // right hip pitch
                torque_stance_hip(7) = -torque_pelv(0)*(1-swing_pd_switch); // right hip roll
            }
            else
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 1-swing_pd_switch;
                pd_control_mask_(2) = 1-swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = swing_pd_switch;
                pd_control_mask_(8) = swing_pd_switch;

                torque_stance_hip(2) = -torque_pelv(1)*swing_pd_switch; // left hip pitch
                torque_stance_hip(1) = -torque_pelv(0)*swing_pd_switch; // left hip roll

                torque_stance_hip(8) = -torque_pelv(1)*(1-swing_pd_switch); // right hip pitch
                torque_stance_hip(7) = -torque_pelv(0)*(1-swing_pd_switch); // right hip roll
            }
        }
    }
    else
    {
        lleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.2*walking_duration_, support_foot_transform_init_.translation()(0), 0, 0, lleg_transform_target.translation()(0), 0, 0)(0);
        lleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.2*walking_duration_, support_foot_transform_init_.translation()(1), 0, 0, lleg_transform_target.translation()(1), 0, 0)(0);
        lleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.2*walking_duration_, support_foot_transform_init_.translation()(2), 0, 0, lleg_transform_target.translation()(2), 0, 0)(0);
        lleg_transform_from_global.linear() = lleg_transform_target.linear();

        rleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.2*walking_duration_, support_foot_transform_init_.translation()(0), 0, 0, rleg_transform_target.translation()(0), 0, 0)(0);
        rleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.2*walking_duration_, support_foot_transform_init_.translation()(1), 0, 0, rleg_transform_target.translation()(1), 0, 0)(0);
        rleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.2*walking_duration_, support_foot_transform_init_.translation()(2), 0, 0, rleg_transform_target.translation()(2), 0, 0)(0);
        rleg_transform_from_global.linear() = rleg_transform_target.linear();
        computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

        // for(int i = 4; i<6; i++)
        // {
        //     desired_q_(i) = 0.3*desired_q_leg(i) + 0.7*pre_desired_q_(i); //left ankle
        //     desired_q_(i+6) = 0.3*desired_q_leg(i+6) + 0.7*pre_desired_q_(i+6); //right ankle
        // }
        desired_q_(0) = 0.5*motion_q_(0) + 0.5*pre_desired_q_(0);
        desired_q_(6) = 0.5*motion_q_(6) + 0.5*pre_desired_q_(6);

        desired_q_(3) = 0.5*motion_q_(3) + 0.5*pre_desired_q_(3); //left knee
        desired_q_(9) = 0.5*motion_q_(9) + 0.5*pre_desired_q_(9); //right knee
        
        // desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //lefft ankle pitch
        // desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //lefft ankle roll
        // desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right ankle pitch
        // desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right ankle roll

        // desired_q_(4) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
        // desired_q_(5) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
        // desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
        // desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll

        desired_q_(4)   =   desired_q_leg(4);
        desired_q_(5)   =   desired_q_leg(5);
        desired_q_(10)  =   desired_q_leg(10);
        desired_q_(11)  =   desired_q_leg(11);
        
        for(int i = 1; i<6; i++)
        {
            kp_joint(i+6) = kp_stiff_joint(i+6); //swing foot
            kp_joint(i) = kp_stiff_joint(i); //support foot
        }

        swing_pd_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_*walking_duration_, 0, 1, 0, 0);
        if(stop_walking_trigger_ == true)
        {
            if(foot_contact_ == 1)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 0;
                pd_control_mask_(2) = 0;

                pd_control_mask_(6) = 1-swing_pd_switch;
                pd_control_mask_(7) = 1-swing_pd_switch;
                pd_control_mask_(8) = 1-swing_pd_switch;

                torque_stance_hip(2) = -torque_pelv(1); // left hip pitch
                torque_stance_hip(1) = -torque_pelv(0); // left hip roll
                torque_stance_hip(8) = -torque_pelv(1)*swing_pd_switch; // right hip pitch
                torque_stance_hip(7) = -torque_pelv(0)*swing_pd_switch; // right hip roll
            }
            else if (foot_contact_ == -1)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 1-swing_pd_switch;
                pd_control_mask_(2) = 1-swing_pd_switch;

                pd_control_mask_(6) = 0;
                pd_control_mask_(7) = 0;
                pd_control_mask_(8) = 0;

                torque_stance_hip(2) = -torque_pelv(1)*swing_pd_switch; // left hip pitch
                torque_stance_hip(1) = -torque_pelv(0)*swing_pd_switch; // left hip roll
                torque_stance_hip(8) = -torque_pelv(1); // right hip pitch
                torque_stance_hip(7) = -torque_pelv(0); // right hip roll
            }
        }
        else if(first_step_trigger_ == false)
        {
            // desired_q_(5) = current_q_(5); //aknle roll free for start motion
            // desired_q_(11) = current_q_(11); //aknle roll

            // kp_joint(5) = 600;
            // kv_joint(5) = 40;
            // kp_joint(11)= 600;
            // kv_joint(11)= 40;

            // kp_joint(4) = 400;
            // kv_joint(4) = 40;
            // kp_joint(10)= 400;
            // kv_joint(10)= 40;

            pd_control_mask_(6) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5*walking_duration_, 0, 1, 0, 0);
            pd_control_mask_(7) = 0;
            pd_control_mask_(8) = 0;

            pd_control_mask_(0) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5*walking_duration_, 0, 1, 0, 0);
            pd_control_mask_(1) = 0;
            pd_control_mask_(2) = 0;

            double hip_control_switch;
            hip_control_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5*walking_duration_, 0, 1, 0, 0);
            torque_stance_hip(2) = -torque_pelv(1)*hip_control_switch; // left hip pitch
            torque_stance_hip(1) = -torque_pelv(0)*hip_control_switch; // left hip roll
            torque_stance_hip(8) = -torque_pelv(1)*hip_control_switch; // right hip pitch
            torque_stance_hip(7) = -torque_pelv(0)*hip_control_switch; // right hip roll
        } 
        
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////HIP YAW/////////////////////////////
    // desired_q_(0)=motion_q_(0);
    // desired_q_(6)=motion_q_(6);
    ////////////////////////////////////////////////////////////////////

    //////////////////////////////////LEG Q DOT/////////////////////////////////
    desired_q_dot_.segment(0, 4) = (desired_q_.segment(0, 4) - pre_desired_q_.segment(0, 4))/dt_; //left hip and knee
    desired_q_dot_.segment(6, 4) = (desired_q_.segment(6, 4) - pre_desired_q_.segment(6, 4))/dt_; //left hip and knee
    ///////////////////////////////////////////////////////////////////////////

    /////////////////////////////////WAIST DESIRED JOINT ANGLES//////////////////////////////
    Vector3d phi_trunk;
    phi_trunk = -DyrosMath::getPhi(tocabi_.link_[Upper_Body].Rotm, pelv_yaw_rot_current_from_global_);
    // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

    desired_q_dot_(12) = 20*phi_trunk(2); //waist yaw
    desired_q_dot_(13) = 20*phi_trunk(1); //waist pitch
    desired_q_dot_(14) = -40*phi_trunk(0); //waist roll
    
    desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3)*dt_;

    //////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////HEAD///////////////////////////////////////////////
    for(int i = 15; i<MODEL_DOF; i++)
    {
        desired_q_(i) = motion_q_(i);
    }   
    ////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////ANKLE TUNING FOR VELOCITY TRACKING/////////////////
    desired_q_ += jointComTrackingTuning(); //reference: Lee, Yoonsang, Sungeun Kim, and Jehee Lee. "Data-driven biped control." ACM SIGGRAPH 2010 papers. 2010. 1-8.
    //////////////////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////MOTION CONTROL/////////////////////////////////////
    // torque += stablePDControl(1000, 1000*dt_*16, current_q_, current_q_dot_, current_q_ddot_, desired_q_, desired_q_dot_);
    for(int i = 0; i< MODEL_DOF; i++)
    {
        torque(i) = kp_joint(i)*(desired_q_(i) - current_q_(i)) + kv_joint(i)*(desired_q_dot_(i) - current_q_dot_(i));
        torque(i) = torque(i)*pd_control_mask_(i);   // masking for joint pd control 
    }
    ////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////SWING FOOT ASSIST TORQUE///////////////////////////
    for(int i=0; i<12; i++)
    {
        torque(i) += torque_swing_assist(i)*pd_control_mask_(i);
    }
    //////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////PELVIS ORIENTATION CONTROL TORQUE/////////////////
    torque += torque_stance_hip;
    //////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////PRINT DATA//////////////////////////////////////////////////////////////////
    // if( walking_phase_ > 0.1)
    {
        // cout<<"pelv_rot_current_yaw_aline_: \n"<<pelv_rot_current_yaw_aline_<<endl;
        // // cout<<"pelv_rot_current_yaw_aline_.eulerAngles(2,1,0): \n"<<pelv_rot_current_yaw_aline_.eulerAngles(2,1,0)<<endl;
        // cout<<"desired_q_: \n"<<desired_q_<<endl;
        // cout<<"current_q_: \n"<<current_q_<<endl;
        // cout<<"error_q_: \n"<< desired_q_ - current_q_<<endl;
        // cout<<"joint_pd_torque: \n"<<torque<<endl;
        // cout<<"torque_stance_hip: \n"<<torque_stance_hip<<endl;
        
        // cout<<"swing_foot_acc_trajectory_from_global_(2): \n"<<swing_foot_acc_trajectory_from_global_(2) <<endl;
        // cout<<"torque_swing_assist: \n"<<torque_swing_assist.segment(0, 12) <<endl;
    }

    // torque(4) = DyrosMath::minmax_cut(torque(4), -0.15*tocabi_.com_.mass*GRAVITY, +0.15*tocabi_.com_.mass*GRAVITY);
    // torque(5) = DyrosMath::minmax_cut(torque(5), -0.085*tocabi_.com_.mass*GRAVITY, +0.085*tocabi_.com_.mass*GRAVITY);
    // torque(10) = DyrosMath::minmax_cut(torque(10), -0.15*tocabi_.com_.mass*GRAVITY, +0.15*tocabi_.com_.mass*GRAVITY);
    // torque(11) = DyrosMath::minmax_cut(torque(11), -0.085*tocabi_.com_.mass*GRAVITY, +0.085*tocabi_.com_.mass*GRAVITY);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    return torque;
}

Eigen::VectorQd TocabiController::zmpAnkleControl()
{
    VectorQd zmp_ankle_torque;
    Vector3d zmp_target;
    zmp_ankle_torque.setZero();

    Eigen::Vector3d zmp_desired_lfoot_local;
    Eigen::Vector3d zmp_desired_rfoot_local;
    Eigen::Vector2d zmp_desired_zmp_both;

    double kp_zmp = 1;
    double kv_zmp = 0;
    //////////////////////zmp trajectory///////////////////////



    if( (foot_swing_trigger_ == true)||(walking_speed_ != 0) )
    {
        if(foot_contact_ == 1)
        {
            zmp_target = lfoot_transform_current_from_global_.translation();
        }
        else if(foot_contact_ == -1)
        {
            zmp_target = rfoot_transform_current_from_global_.translation();
        }
    }
    else
    {
        zmp_target = middle_of_both_foot_;
    }
    

    // zmp_desired_from_global_ = 0.1*zmp_target + 0.9*zmp_desired_pre_;
    // zmp_desired_from_global_ = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, );
    zmp_desired_from_global_ = zmp_target;
    ///////////////////////////////////////////////////////////////////

    zmp_desired_lfoot_local = lfoot_transform_current_from_global_.linear().transpose()*(zmp_desired_from_global_ - lfoot_transform_current_from_global_.translation());
    zmp_desired_rfoot_local = rfoot_transform_current_from_global_.linear().transpose()*(zmp_desired_from_global_ - rfoot_transform_current_from_global_.translation());

    if(foot_swing_trigger_ == true)
    {
        if(foot_contact_ == 1)
        {
            // zmp_ankle_torque(4) = -l_ft_(2)*zmp_desired_lfoot_local(0) + kp_zmp*(zmp_desired_lfoot_local(0) - zmp_local_lfoot_(0)) - kv_zmp*zmp_dot_local_lfoot_(0); //ankle pitch
            // zmp_ankle_torque(5) = l_ft_(2)*zmp_desired_lfoot_local(1) - kp_zmp*(zmp_desired_lfoot_local(1) - zmp_local_lfoot_(1)) + kv_zmp*zmp_dot_local_lfoot_(1); //ankle roll
            zmp_ankle_torque(4) =  + kp_zmp*(zmp_desired_lfoot_local(0) - zmp_local_lfoot_(0)) - kv_zmp*zmp_dot_local_lfoot_(0); //ankle pitch
            zmp_ankle_torque(5) = - kp_zmp*(zmp_desired_lfoot_local(1) - zmp_local_lfoot_(1)) + kv_zmp*zmp_dot_local_lfoot_(1); //ankle roll
        }
        else if(foot_contact_ == -1)
        {
            // zmp_ankle_torque(10) = -r_ft_(2)*zmp_desired_rfoot_local(0) + kp_zmp*(zmp_desired_rfoot_local(0) - zmp_local_rfoot_(0)) - kv_zmp*zmp_dot_local_rfoot_(0); //ankle pitch
            // zmp_ankle_torque(11) = r_ft_(2)*zmp_desired_rfoot_local(1) - kp_zmp*(zmp_desired_rfoot_local(1) - zmp_local_rfoot_(1)) + kv_zmp*zmp_dot_local_rfoot_(1); //ankle roll
            zmp_ankle_torque(10) = + kp_zmp*(zmp_desired_rfoot_local(0) - zmp_local_rfoot_(0)) - kv_zmp*zmp_dot_local_rfoot_(0); //ankle pitch
            zmp_ankle_torque(11) = - kp_zmp*(zmp_desired_rfoot_local(1) - zmp_local_rfoot_(1)) + kv_zmp*zmp_dot_local_rfoot_(1); //ankle roll
        }
    }
    else
    {
        // Vector3d total_ankle_torque;
        // total_ankle_torque(0) = l_ft_(2)*(zmp_desired_from_global_(1) - lfoot_transform_current_from_global_.translation()(1))
        // + r_ft_(2)*(zmp_desired_from_global_(1) - rfoot_transform_current_from_global_.translation()(1))
        // -kp_zmp*(zmp_desired_from_global_(1) - zmp_measured_(1))
        // +kv_zmp*(zmp_dot_measured_(1)); //roll

        // total_ankle_torque(1) = -l_ft_(2)*(zmp_desired_from_global_(0) - lfoot_transform_current_from_global_.translation()(0))
        // - r_ft_(2)*(zmp_desired_from_global_(0) - rfoot_transform_current_from_global_.translation()(0)); 
        // +kp_zmp*(zmp_desired_from_global_(0) - zmp_measured_(0))
        // -kv_zmp*(zmp_dot_measured_(0)); //pitch

        // total_ankle_torque(2) = 0;

        // double left_zmp_ratio = abs(zmp_desired_from_global_(1) - lfoot_transform_current_from_global_.translation()(1))
        // /abs(rfoot_transform_current_from_global_.translation()(1) - lfoot_transform_current_from_global_.translation()(1));

        // left_zmp_ratio = DyrosMath::minmax_cut(left_zmp_ratio, 0, 1);

        // zmp_ankle_torque(4) = left_zmp_ratio*(lfoot_transform_current_from_global_.linear().transpose()*total_ankle_torque)(1);
        // zmp_ankle_torque(5) = left_zmp_ratio*(rfoot_transform_current_from_global_.linear().transpose()*total_ankle_torque)(0);

        // zmp_ankle_torque(10) = (1-left_zmp_ratio)*total_ankle_torque(1);
        // zmp_ankle_torque(11) = (1-left_zmp_ratio)*total_ankle_torque(0);

    }

    return zmp_ankle_torque;
}

Eigen::VectorQd TocabiController::jointComTrackingTuning()
{
    Eigen::VectorQd desired_q_tune;
    desired_q_tune.setZero();

    //ankle
    double kp_ank_sag = 0.1; //x direction ankle pitch gain for com position error
    double kv_ank_sag = 0.1; //x direction ankle pitch gain for com velocity error

    double kp_ank_cor = 0.1; //y direction ankle pitch gain for com position error
    double kv_ank_cor = 0.1; //y direction ankle pitch gain for com velocity error

    //hip
    double kp_hip_sag = 0;//0.01; //x direction ankle pitch gain for com position error
    double kv_hip_sag = 0;//0.05; //x direction ankle pitch gain for com velocity error

    double kp_hip_cor = 0;//0.2; //y direction ankle pitch gain for com position error
    double kv_hip_cor = 0;//0.2; //y direction ankle pitch gain for com velocity error

    
    if(foot_swing_trigger_ == true)
    {
        double switching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);
        if(foot_contact_ == 1) //left support
        {   
            
            desired_q_tune(4) -= switching*(kp_ank_sag*(com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag*(com_vel_desired_(0) - com_vel_current_(0)));
            desired_q_tune(5) += switching*(kp_ank_cor*(com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor*(com_vel_desired_(1) - com_vel_current_(1)));

            if(desired_q_(8) < 0)
            {
                desired_q_tune(8) += switching*(kp_hip_sag*(com_pos_desired_(0) - com_pos_current_(0)) + kv_hip_sag*(com_vel_desired_(0) - com_vel_current_(0)));
            }
            desired_q_tune(7) -= switching*(kp_hip_cor*(com_pos_desired_(1) - com_pos_current_(1)) + kv_hip_cor*(com_vel_desired_(1) - com_vel_current_(1)));
        }
        else if(foot_contact_ == -1) //right support
        {
            desired_q_tune(10) -= switching*(kp_ank_sag*(com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag*(com_vel_desired_(0) - com_vel_current_(0)));
            desired_q_tune(11) += switching*(kp_ank_cor*(com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor*(com_vel_desired_(1) - com_vel_current_(1)));

            if(desired_q_(2) > 0)
            {
                desired_q_tune(2) += switching*(kp_hip_sag*(com_pos_desired_(0) - com_pos_current_(0)) + kv_hip_sag*(com_vel_desired_(0) - com_vel_current_(0)));
            }
            
            desired_q_tune(1) -= switching*(kp_hip_cor*(com_pos_desired_(1) - com_pos_current_(1)) + kv_hip_cor*(com_vel_desired_(1) - com_vel_current_(1)));
        }
    }
    else
    {
        desired_q_tune(4) -= kp_ank_sag*(com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag*(com_vel_desired_(0) - com_vel_current_(0));
        desired_q_tune(5) += kp_ank_cor*(com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor*(com_vel_desired_(1) - com_vel_current_(1));
        
        
        desired_q_tune(10) -= kp_ank_sag*(com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag*(com_vel_desired_(0) - com_vel_current_(0));
        desired_q_tune(11) += kp_ank_cor*(com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor*(com_vel_desired_(1) - com_vel_current_(1));
    }
    


    return desired_q_tune;
}

void TocabiController::computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des)
{
  //float = World/ trunk = pelvis
  // 명주 정리 (KAJITA 책 <-> Code 구성)
  // float_trunk_transform.rotation() : float에서 바라본 pelvis rotation -> R1
  // float_trunk_transform.translation() : float에서 바라본 pelvis 좌표 -> P1
  // float_rleg_transform.rotation() : float에서 바라본 발끝 rotation -> R7 
  // float_rleg_transform.translation() : float에서 바라본 발끝 좌표 -> P7
  // float_trunk_transform.translation() + float_trunk_transform.rotation()*D  : float에서 바라본 pelvis 좌표 + float 좌표계로 변환 * pelvis 좌표계에서 바라본 pelvis ~ hip까지 거리-> P2
  
  // R7.transpose * (P2 - P7) , P2 = P1 + R1*D


  Eigen::Vector3d R_r, R_D, L_r, L_D ;

  L_D << 0 , 0.1025, -0.1225;
  R_D << 0 , -0.1025, -0.1225;
  
  L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*L_D - float_lleg_transform.translation());
  R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*R_D - float_rleg_transform.translation());
  
  double R_C = 0, L_C = 0, L_upper = 0.35, L_lower = 0.35 , R_alpha = 0, L_alpha = 0;
  double L_max = L_upper + L_lower;
  L_r(2) = DyrosMath::minmax_cut(L_r(2), 0.2, L_max);
  R_r(2) = DyrosMath::minmax_cut(R_r(2), 0.2, L_max);
  
  R_C = R_r.norm();
  L_C = L_r.norm();

//   L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
  if(foot_swing_trigger_ == true)
  {
    if(foot_contact_ == 1)
    {
        if(R_C> L_max)
        {
            double mapping_xy = sqrt( (pow(L_max, 2) -  pow(R_r(2),2))/(pow(R_r(0),2) + pow(R_r(1),2)) ); //xy mapping
            // double mapping_xy =  (L_max - 1e-2)/R_r.norm();
            R_r(0) *= mapping_xy;
            R_r(1) *= mapping_xy;
            // R_r(2) *= mapping_xy;

            R_C = L_max;
            // cout<<"swing xy projection gain: "<<mapping_xy<<endl;
            // cout<<"swing xy projection point: "<<R_r<<endl;
        }
    }
    else if(foot_contact_ == -1)
    {
        if(L_C> L_max)
        {
            double mapping_xy = sqrt( (pow(L_max, 2) -  pow(L_r(2),2))/(pow(L_r(0),2) + pow(L_r(1),2)) );
            // double mapping_xy =  (L_max - 1e-2)/L_r.norm();
            L_r(0) *= mapping_xy;
            L_r(1) *= mapping_xy;
            // L_r(2) *= mapping_xy;
            
            L_C = L_max;
            // cout<<"swing xy projection gain: "<<mapping_xy<<endl;
            // cout<<"swing xy projection point: "<<L_r<<endl;
        }
    }
  }



  double temp_q_des;
  temp_q_des = (pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2*L_upper*L_lower);
  temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1, 1);
  q_des(3) = (-acos(temp_q_des)+ M_PI) ;
  temp_q_des = (pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2*L_upper*L_lower);
  temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1, 1);
  q_des(9) = (-acos(temp_q_des)+ M_PI) ;
  L_alpha = asin(DyrosMath::minmax_cut(L_upper / L_C * sin(M_PI - q_des(3)), -1, 1));
//   L_alpha = q_des(3)/2;
  R_alpha = asin(DyrosMath::minmax_cut(L_upper / R_C * sin(M_PI - q_des(9)), -1, 1));
//   R_alpha = q_des(9)/2;

  double temp_q_des_4;
  temp_q_des_4  = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2)));
  if(temp_q_des_4> M_PI/2)
  {
      temp_q_des_4 -= M_PI;
  }  
  else if(temp_q_des_4< -M_PI/2)
  {
      temp_q_des_4 += M_PI;
  }
  q_des(4) = temp_q_des_4 -L_alpha;

  double temp_q_des_10;
  temp_q_des_10  = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2)));
  if(temp_q_des_10> M_PI/2)
  {
      temp_q_des_10 -= M_PI;
  }  
  else if(temp_q_des_10< -M_PI/2)
  {
      temp_q_des_10 += M_PI;
  }
  q_des(10) = temp_q_des_10 -R_alpha;
//   q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;

//   if( walking_phase_ > 0.1 )
  {
    // cout<<"L_C: "<<L_C<<endl;
    // cout<<"R_C: "<<R_C<<endl;

    // cout<<"L_r: "<<L_r<<endl;
    // cout<<"R_r: "<<R_r<<endl;
    // cout<<"q_des(4): "<<q_des(4)<<endl;
    // cout<<"L_alpha: "<<L_alpha<<endl;
    // cout<<"q_des(10): "<<q_des(10)<<endl;
  }

  // trunk_lleg_rotation -> R1.transpose * R7 
  // Ryaw * Rroll * Rpitch = R1.transpose * R7 * ~ 
  q_des(11) =  atan2( R_r(1), R_r(2) );
  if(q_des(11)> M_PI/2)
  {
      q_des(11) -= M_PI;
  }
  else if(q_des(11)< -M_PI/2)
  {
      q_des(11) += M_PI;
  }

  q_des(5) =  atan2( L_r(1), L_r(2) ); // Ankle roll
  if(q_des(5)> M_PI/2)
  {
      q_des(5) -= M_PI;
  }  
  else if(q_des(5)< -M_PI/2)
  {
      q_des(5) += M_PI;
  }

  Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
  Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
  Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

  L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3)-q_des(4));
  L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
  R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9)-q_des(10));
  R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11)); 
  
  L_Hip_rot_mat.setZero(); R_Hip_rot_mat.setZero();

  L_Hip_rot_mat = float_trunk_transform.linear().transpose() * float_lleg_transform.linear() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat; 
  R_Hip_rot_mat = float_trunk_transform.linear().transpose() * float_rleg_transform.linear() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

  q_des(0) = -atan2(-L_Hip_rot_mat(0,1),L_Hip_rot_mat(1,1)); // Hip yaw
//   q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
  q_des(1) = asin(DyrosMath::minmax_cut(L_Hip_rot_mat(2, 1), -1, 1));
  q_des(2) =  atan2(-L_Hip_rot_mat(2,0), L_Hip_rot_mat(2,2)) ; // Hip pitch
  q_des(3) =  q_des(3) ; // Knee pitch
  q_des(4) =  q_des(4) ; // Ankle pitch

//   cout<<"L_Hip_rot_mat: \n"<<L_Hip_rot_mat<<endl;
//   cout<<"R_Hip_rot_mat: \n"<<R_Hip_rot_mat<<endl;


  if(q_des(0)> M_PI/2)
  {
      q_des(0) -= M_PI;
  }
  else if(q_des(0)< -M_PI/2)
  {
      q_des(0) += M_PI;
  }

  if(q_des(1)> M_PI/2)
  {
      q_des(1) -= M_PI;
  }
  else if(q_des(1)< -M_PI/2)
  {
      q_des(1) += M_PI;
  }

    if(q_des(2)> M_PI/2)
  {
      q_des(2) -= M_PI;
  }
  else if(q_des(2)< -M_PI/2)
  {
      q_des(2) += M_PI;
  }




  q_des(6) = -atan2(-R_Hip_rot_mat(0,1),R_Hip_rot_mat(1,1));
//   q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
  q_des(7) = asin(DyrosMath::minmax_cut(R_Hip_rot_mat(2, 1), -1, 1));
  q_des(8) = atan2(-R_Hip_rot_mat(2,0), R_Hip_rot_mat(2,2)) ;
  q_des(9) = q_des(9) ;
  q_des(10) = q_des(10) ; 

  if(q_des(6)> M_PI/2)
  {
      q_des(6) -= M_PI;
  }
  else if(q_des(6)< -M_PI/2)
  {
      q_des(6) += M_PI;
  }
  
  if(q_des(7)> M_PI/2)
  {
      q_des(7) -= M_PI;
  }
  else if(q_des(7)< -M_PI/2)
  {
      q_des(7) += M_PI;
  }

    if(q_des(8)> M_PI/2)
  {
      q_des(8) -= M_PI;
  }
  else if(q_des(8)< -M_PI/2)
  {
      q_des(8) += M_PI;
  }


}

Eigen::VectorQd TocabiController::tuneTorqueForZMPSafety(Eigen::VectorQd task_torque)
{
    int flag;
    Eigen::Vector2d diff_zmp_lfoot;
    Eigen::Vector2d diff_zmp_rfoot;
    Eigen::Vector2d diff_zmp_both;
    Eigen::Vector3d diff_btw_both_foot;
    Eigen::Vector2d foot_size;
    double safe_region_ratio = 0.8;
    double edge_region_ratio = 0.95;
    double left_ankle_pitch_tune = 1;
    double left_ankle_roll_tune = 1;
    double right_ankle_pitch_tune = 1;
    double right_ankle_roll_tune = 1;

    foot_size(0) = 0.15;
    foot_size(1) = 0.085;

    if(foot_swing_trigger_ == true)
    {
        if(foot_contact_ == 1)
        {
            // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
            // left_ankle_pitch_tune = DyrosMath::cubic(zmp_local_lfoot_(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
            // left_ankle_roll_tune = DyrosMath::cubic(zmp_local_lfoot_(1)/foot_size(1) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);

            Vector3d phi_support_ankle;
            phi_support_ankle = -DyrosMath::getPhi(tocabi_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
            left_ankle_pitch_tune *= DyrosMath::cubic( abs(phi_support_ankle(1)) , 0.00, 0.05, 1, 0, 0, 0);
            left_ankle_roll_tune *= DyrosMath::cubic( abs(phi_support_ankle(0)) , 0.00, 0.05, 1, 0, 0, 0);
            
            left_ankle_pitch_tune *= DyrosMath::cubic(l_ft_(2) , -tocabi_.com_.mass*GRAVITY/5, -tocabi_.com_.mass*GRAVITY/10, 1, 0, 0, 0);
            left_ankle_roll_tune *= DyrosMath::cubic(l_ft_(2) , -tocabi_.com_.mass*GRAVITY/5, -tocabi_.com_.mass*GRAVITY/10, 1, 0, 0, 0);
            

            task_torque(4) = task_torque(4)*left_ankle_pitch_tune;
            task_torque(5) = task_torque(5)*left_ankle_roll_tune;
        }
        else if (foot_contact_ == -1)
        {
            // diff_zmp_rfoot(0) = abs(zmp_measured_rfoot_(0) - rfoot_transform_current_from_global_.translation()(0));
            // diff_zmp_rfoot(1) = abs(zmp_measured_rfoot_(1) - rfoot_transform_current_from_global_.translation()(1));

            // right_ankle_pitch_tune = DyrosMath::cubic(zmp_local_rfoot_(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
            // right_ankle_roll_tune = DyrosMath::cubic(zmp_local_rfoot_(1)/foot_size(1) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);   
            
            Vector3d phi_support_ankle;
            phi_support_ankle = -DyrosMath::getPhi(tocabi_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);         
            right_ankle_pitch_tune *= DyrosMath::cubic( abs(phi_support_ankle(1)) , 0.00, 0.05, 1, 0, 0, 0);
            right_ankle_roll_tune *= DyrosMath::cubic( abs(phi_support_ankle(0)) , 0.00, 0.05, 1, 0, 0, 0);

            right_ankle_pitch_tune *= DyrosMath::cubic(r_ft_(2) , -tocabi_.com_.mass*GRAVITY/5, -tocabi_.com_.mass*GRAVITY/10, 1, 0, 0, 0);
            right_ankle_roll_tune *= DyrosMath::cubic(r_ft_(2) , -tocabi_.com_.mass*GRAVITY/5, -tocabi_.com_.mass*GRAVITY/10, 1, 0, 0, 0);
        

            task_torque(10) = task_torque(10)*right_ankle_pitch_tune;
            task_torque(11) = task_torque(11)*right_ankle_roll_tune;
        }
    }
    else
    {   
        middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() +lfoot_transform_current_from_global_.translation())/2;
        diff_btw_both_foot = lfoot_transform_current_from_global_.translation() -rfoot_transform_current_from_global_.translation();
        diff_zmp_both(0) = abs(zmp_measured_(0) - middle_of_both_foot_(0));
        diff_zmp_both(1) = abs(zmp_measured_(1) - middle_of_both_foot_(1));
        
        // left_ankle_pitch_tune = DyrosMath::cubic(diff_zmp_both(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
        // right_ankle_pitch_tune = left_ankle_pitch_tune;

        // left_ankle_roll_tune = DyrosMath::cubic(diff_zmp_both(1) , diff_btw_both_foot(1) + safe_region_ratio*foot_size(1), diff_btw_both_foot(1) + edge_region_ratio*foot_size(1), 1, 0, 0, 0);
        // right_ankle_roll_tune = left_ankle_roll_tune;
        
        // if(l_ft_(2) < tocabi_.com_.mass*GRAVITY/5)
        {
            left_ankle_pitch_tune *= DyrosMath::cubic(l_ft_(2) , tocabi_.com_.mass*GRAVITY/10, tocabi_.com_.mass*GRAVITY/5, 0, 1, 0, 0);
            left_ankle_roll_tune *= DyrosMath::cubic(l_ft_(2) , tocabi_.com_.mass*GRAVITY/10, tocabi_.com_.mass*GRAVITY/5, 0, 1, 0, 0);
        }
        
        // if(r_ft_(2) < tocabi_.com_.mass*GRAVITY/2)
        {
            right_ankle_pitch_tune *= DyrosMath::cubic(r_ft_(2) , tocabi_.com_.mass*GRAVITY/10, tocabi_.com_.mass*GRAVITY/5, 0, 1, 0, 0);
            right_ankle_roll_tune *= DyrosMath::cubic(r_ft_(2) , tocabi_.com_.mass*GRAVITY/10, tocabi_.com_.mass*GRAVITY/5, 0, 1, 0, 0);
        }
        

        Vector3d phi_support_ankle;
        phi_support_ankle = -DyrosMath::getPhi(tocabi_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
        left_ankle_pitch_tune = DyrosMath::cubic( abs(phi_support_ankle(1)) , 0.00, 0.05, 1, 0, 0, 0);
        left_ankle_roll_tune = DyrosMath::cubic( abs(phi_support_ankle(0)) , 0.00, 0.05, 1, 0, 0, 0);

        phi_support_ankle = -DyrosMath::getPhi(tocabi_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);         
        right_ankle_pitch_tune = DyrosMath::cubic( abs(phi_support_ankle(1)) , 0.00, 0.05, 1, 0, 0, 0);
        right_ankle_roll_tune = DyrosMath::cubic( abs(phi_support_ankle(0)) , 0.00, 0.05, 1, 0, 0, 0);

        task_torque(4) = task_torque(4)*left_ankle_pitch_tune;
        task_torque(5) = task_torque(5)*left_ankle_roll_tune;

        task_torque(10) = task_torque(10)*right_ankle_pitch_tune;
        task_torque(11) = task_torque(11)*right_ankle_roll_tune;        
    }

    if((left_ankle_pitch_tune*left_ankle_roll_tune*right_ankle_pitch_tune*right_ankle_roll_tune) != 1)
    {
        // if( true )
        // {
        //     cout<<"############### ankle torque tuning! ###############"<<endl;
        //     cout<<"########left_ankle_pitch_tune:"<<left_ankle_pitch_tune<< "############"<<endl;
        //     cout<<"########left_ankle_roll_tune:"<<left_ankle_roll_tune<< "############"<<endl;
        //     cout<<"########right_ankle_pitch_tune:"<<right_ankle_pitch_tune<< "############"<<endl;
        //     cout<<"########right_ankle_roll_tune:"<<right_ankle_roll_tune<< "############"<<endl;
        //     cout<<"############### ankle torque tuning! ###############"<<endl;
        // }
    } 

    return task_torque;
}

Eigen::VectorQd TocabiController::stablePDControl(double kp, double kd, Eigen::VectorQd current_q, Eigen::VectorQd current_q_dot, Eigen::VectorQd estimated_q_ddot, Eigen::VectorQd desired_q_next_step, Eigen::VectorQd desired_q_dot_next_step) // without velocity reference
{
    Eigen::VectorQd torque;

    torque = kp*(desired_q_next_step - current_q - current_q_dot*dt_) + kd*(desired_q_dot_next_step -current_q_dot - estimated_q_ddot*dt_);

    return torque;
}

Eigen::VectorQd TocabiController::stablePDControl(double kp, double kd, Eigen::VectorQd current_q, Eigen::VectorQd current_q_dot, Eigen::VectorQd estimated_q_ddot, Eigen::VectorQd desired_q_next_step) // without velocity reference
{
    Eigen::VectorQd torque;

    torque = kp*(desired_q_next_step - current_q - current_q_dot*dt_) + kd*( -current_q_dot - estimated_q_ddot*dt_);
    
    return torque;
}

void TocabiController::savePreData()
{
    pre_time_ = current_time_;
    pre_q_ = tocabi_.q_;
    pre_desired_q_ = desired_q_;
    zmp_measured_ppre_ = zmp_measured_pre_;
    zmp_measured_pre_ = zmp_measured_;
    com_pos_desired_pre_ = com_pos_desired_;
    com_vel_desired_pre_ = com_vel_desired_;
    f_star_xy_pre_ = f_star_xy_;
    f_star_6d_pre_ = f_star_6d_;
    torque_task_pre_ = torque_task_;
    torque_grav_pre_ = torque_grav_;

    zmp_desired_pre_ = zmp_desired_from_global_;
    zmp_local_lfoot_pre_ = zmp_local_lfoot_;
    zmp_local_rfoot_pre_ = zmp_local_rfoot_;
}