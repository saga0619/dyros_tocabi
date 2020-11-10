#include "tocabi_controller/state_manager.h"
//#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
//#include <tf/transform_datatypes.h>
#include <sstream>

StateManager::StateManager(DataContainer &dc_global) : dc(dc_global)
{
    //signal(SIGINT, StateManager::sigintHandler);

    gui_command = dc.nh.subscribe("/tocabi/command", 100, &StateManager::CommandCallback, this);
    joint_states_pub = dc.nh.advertise<sensor_msgs::JointState>("/tocabi/jointstates", 1);
    time_pub = dc.nh.advertise<std_msgs::Float32>("/tocabi/time", 1);
    motor_acc_dif_info_pub = dc.nh.advertise<tocabi_controller::MotorInfo>("/tocabi/accdifinfo", 100);
    tgainPublisher = dc.nh.advertise<std_msgs::Float32>("/tocabi/torquegain", 100);
    point_pub = dc.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/point", 100);
    ft_viz_pub = dc.nh.advertise<visualization_msgs::MarkerArray>("/tocabi/ft_viz", 0);
    gui_state_pub = dc.nh.advertise<std_msgs::Int32MultiArray>("/tocabi/systemstate", 100);
    support_polygon_pub = dc.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/support_polygon", 100);
    ft_viz_msg.markers.resize(4);
    syspub_msg.data.resize(6);
    imu_lin_acc_lpf.setZero();
    pelv_lin_acc.setZero();
    imu_lin_acc_before.setZero();
    for (int i = 0; i < 4; i++)
    {
        ft_viz_msg.markers[i].header.frame_id = "base_link";
        ft_viz_msg.markers[i].ns = "my_namespace";
        ft_viz_msg.markers[i].id = i;
        ft_viz_msg.markers[i].type = visualization_msgs::Marker::ARROW;
        ft_viz_msg.markers[i].action = visualization_msgs::Marker::ADD;
        ft_viz_msg.markers[i].points.resize(2);
        ft_viz_msg.markers[i].color.a = 1.0;
        ft_viz_msg.markers[i].color.g = 1.0;

        ft_viz_msg.markers[i].scale.x = 0.1;
        ft_viz_msg.markers[i].scale.y = 0.15;
        ft_viz_msg.markers[i].scale.z = 0;
    }

    pointpub_msg.polygon.points.resize(18);

    if (dc.mode == "realrobot")
    {
        motor_info_pub = dc.nh.advertise<tocabi_controller::MotorInfo>("/tocabi/motorinfo", 1);
        motor_info_msg.motorinfo1.resize(MODEL_DOF);
        motor_info_msg.motorinfo2.resize(MODEL_DOF);
    }
    acc_dif_info_msg.motorinfo1.resize(MODEL_DOF);
    acc_dif_info_msg.motorinfo2.resize(MODEL_DOF);

    joint_state_msg.position.resize(MODEL_DOF);
    joint_state_msg.velocity.resize(MODEL_DOF);
    joint_state_msg.effort.resize(MODEL_DOF);

    gravity_.setZero();
    gravity_(2) = GRAVITY;
    yaw_init = 0.0;

    initialize();
    bool verbose = false; //set verbose true for State Manager initialization info
    bool urdfmode;
    ros::param::get("/tocabi_controller/urdfAnkleRollDamping", urdfmode);
    std::string urdf_path, desc_package_path;

    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    if (urdfmode)
    {
        urdf_path = desc_package_path + "/dyros_tocabi_ankleRollDamping.urdf";
    }
    else
    {
        urdf_path = desc_package_path + "/dyros_tocabi.urdf";
    }

    ROS_INFO_COND(verbose, "Loading DYROS TOCABI description from = %s", desc_package_path.c_str());

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_, true, verbose);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_2, true, verbose);

    ROS_INFO_COND(verbose, "Successfully loaded.");
    ROS_INFO_COND(verbose, "MODEL DOF COUNT = %d and MODEL Q SIZE = %d ", model_.dof_count, model_.q_size);

    // model_.mJoints[0].)
    if (model_.dof_count != MODEL_DOF + 6)
    {
        ROS_WARN("The DoF in the model file and the code do not match.");
        ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)MODEL_DOF + 6);
    }
    else
    {
        // ROS_INFO("id:0 name is : %s",model_.GetBodyName(0));
        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_id_[i] = model_.GetBodyId(TOCABI::LINK_NAME[i]);

            //std::cout << TOCABI::LINK_NAME[i] << " mass : " << model_.mBodies[link_id_[i]].mMass << "center of mass? : " << model_.mBodies[link_id_[i]].mCenterOfMass[0] << " " << model_.mBodies[link_id_[i]].mCenterOfMass[1] << " " << model_.mBodies[link_id_[i]].mCenterOfMass[2] << std::endl;

            if (!model_.IsBodyId(link_id_[i]))
            {
                ROS_INFO_COND(verbose, "Failed to get body id at link %d : %s", i, TOCABI::LINK_NAME[i]);
            }
            // ROS_INFO("%s: \t\t id = %d \t parent link = %d",LINK_NAME[i],
            // link_id_[i],model_.GetParentBodyId(link_id_[i]));
            // ROS_INFO("%dth parent
            // %d",link_id_[i],model_.GetParentBodyId(link_id_[i]));
            // std::cout << model_.mBodies[link_id_[i]].mCenterOfMass << std::endl;
            // //joint_name_map_[JOINT_NAME[i]] = i;
        }

        double total_mass = 0;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_[i].initialize(model_2, link_id_[i], TOCABI::LINK_NAME[i], model_2.mBodies[link_id_[i]].mMass, model_2.mBodies[link_id_[i]].mCenterOfMass);
            total_mass += link_[i].Mass;
        }

        link_[Right_Foot].contact_point << 0.03, 0, -0.1585;
        link_[Right_Foot].sensor_point << 0.0, 0.0, -0.09;
        link_[Left_Foot].contact_point << 0.03, 0, -0.1585;
        link_[Left_Foot].sensor_point << 0.0, 0.0, -0.09;

        link_[Right_Hand].contact_point << 0, 0.0, -0.035;
        link_[Right_Hand].sensor_point << 0.0, 0.0, 0.0;
        link_[Left_Hand].contact_point << 0, 0.0, -0.035;
        link_[Left_Hand].sensor_point << 0.0, 0.0, 0.0;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_local[i] = link_[i];
            link_local[i].model = &model_;
        }

        joint_state_msg.name.resize(MODEL_DOF);
        for (int i = 0; i < MODEL_DOF; i++)
        {
            joint_state_msg.name[i] = TOCABI::JOINT_NAME[i];
        }
        // RigidBodyDynamics::Joint bJ_temp;
        // J_temp=RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeEulerXYZ);
        // model_.mJoints[2] = J_temp;
        std::cout << "Total Mass : " << total_mass << std::endl; // mass without head -> 83.6 kg
        dc.tocabi_.total_mass = total_mass;
    }

    ROS_INFO_COND(verbose, "State manager Init complete");
}

void StateManager::stateThread(void)
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    while (!dc.connected && (!shutdown_tocabi_bool))
    {
        //wait for realrobot thread start
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        if (shutdown_tocabi_bool)
            break;
    }
    std::chrono::microseconds cycletime(dc.ctime);
    int cycle_count = 0;
    if (!shutdown_tocabi_bool)
    {
        std::cout << "State thread start! " << std::endl;

        while (!shutdown_tocabi_bool)
        {
            std::this_thread::sleep_until(st_start_time + std::chrono::microseconds(250) + (cycle_count * cycletime));

            try
            {
                updateState();
                //imuCompenstation();
                //q_dot_virtual_ = q_dot_virtual_raw_;
                initYaw();
                qdotLPF();
            }
            catch (exception &e)
            {
                std::cout << "Error ar updateState : " << e.what() << std::endl;
            }

            if (shutdown_tocabi_bool)
            {
                std::cout << "shutdown signal received" << std::endl;
                break;
            }

            if (dc.imu_ignore == true)
            {
                for (int i = 0; i < 6; i++)
                {
                    q_virtual_local_(i) = 0.0;
                    q_dot_virtual_local_(i) = 0.0;
                    q_ddot_virtual_local_(i) = 0.0;
                }
                q_virtual_local_(MODEL_DOF + 6) = 1.0;
            }

            try
            {
                updateKinematics(model_, link_local, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);
                handleFT();
                contactEstimate();
                stateEstimate();
                if (dc.single_foot_only == false)
                {
                    pelvisPosMonitor();
                }

                //lowpass filter for q_dot
                updateKinematics(model_2, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);
                jointVelocityEstimate();
            }
            catch (exception &e)
            {
                std::cout << "Error ar updateKinematics : " << e.what() << std::endl;
            }

            storeState();

            if ((cycle_count % 10) == 0)
            {
                if (control_time_ > 1.0)
                {
                    try
                    {
                        adv2ROS();
                    }
                    catch (exception &e)
                    {
                        std::cout << "Error ar adv2ROS : " << e.what() << std::endl;
                    }
                }
            }

            if ((cycle_count % 200) == 0)
            {
                try
                {
                    sendStateToGui();
                }
                catch (exception &e)
                {
                    std::cout << "Error ar sendStateToGui : " << e.what() << std::endl;
                }
            }

            if (dc.tocabi_.signal_yaw_init)
            {
                dc.tocabi_.signal_yaw_init = false;
            }
            dc.firstcalcdone = true;
            cycle_count++;
        }
    }
    std::cout << cyellow << "State Thread End !" << creset << std::endl;
}

void StateManager::testThread()
{
    std::chrono::high_resolution_clock::time_point StartTime = std::chrono::high_resolution_clock::now();
    std::chrono::seconds sec10(1);
    std::chrono::milliseconds ms(50);

    std::chrono::duration<double> e_s(0);
    ROS_INFO("state Test Thread START");
    int ThreadCount = 0;

    std::chrono::high_resolution_clock::time_point t[4];
    int wait_t = 1;

    std::chrono::duration<double> dur[3];

    while (!shutdown_tocabi_bool)
    {
        t[0] = std::chrono::high_resolution_clock::now();
        updateState();

        q_virtual_ = VectorXd::Random(MODEL_DOF_QVIRTUAL);
        for (int i = 0; i < 6; i++)
            q_virtual_(i) = 0.0;
        q_virtual_(MODEL_DOF_VIRTUAL) = 1.0;
        q_ = q_virtual_.segment(6, MODEL_DOF);

        t[1] = std::chrono::high_resolution_clock::now();
        updateKinematics(model_, link_local, q_virtual_, q_dot_virtual_, q_ddot_virtual_);
        t[2] = std::chrono::high_resolution_clock::now();

        //stateEstimate();
        //updateKinematics(q_virtual_, q_dot_virtual_, q_ddot_virtual_);

        storeSync();

        storeState();

        t[3] = std::chrono::high_resolution_clock::now();

        dur[0] = t[1] - t[0];
        dur[1] = t[2] - t[1];
        dur[2] = t[3] - t[2];

        if (t[3] > (StartTime + sec10 * wait_t))
        {
            printf("state thread calc, up.state %f up.kin. %f st.state %f \n", dur[0].count() * 1000, dur[1].count() * 1000, dur[2].count() * 1000);
            wait_t++;
        }
        dur[0] = std::chrono::high_resolution_clock::now() - StartTime;

        if (dur[0].count() > 5.0)
        {
            shutdown_tocabi = true;
            break;
        }

        if (shutdown_tocabi_bool)
        {
            printf("state end\n");
            break;
        }
        ThreadCount++;
    }
}

void StateManager::adv2ROS(void)
{

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();

    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "Pelvis_Link";
    transformStamped.transform.rotation.x = q_virtual_(3);
    transformStamped.transform.rotation.y = q_virtual_(4);
    transformStamped.transform.rotation.z = q_virtual_(5);
    transformStamped.transform.rotation.w = q_virtual_(MODEL_DOF_VIRTUAL);
    transformStamped.transform.translation.x = q_virtual_(0);
    transformStamped.transform.translation.y = q_virtual_(1);
    transformStamped.transform.translation.z = q_virtual_(2);

    br.sendTransform(transformStamped);

    joint_state_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < MODEL_DOF; i++)
    {
        joint_state_msg.position[i] = q_virtual_local_[i + 6];
        joint_state_msg.velocity[i] = q_dot_virtual_local_[i + 6];
        joint_state_msg.effort[i] = dc.q_dot_virtual_lpf[i + 6];
        acc_dif_info_msg.motorinfo1[i] = dc.q_dot_virtual_lpf[i + 6];
        acc_dif_info_msg.motorinfo2[i] = q_dot_est[i];
    }

    motor_acc_dif_info_pub.publish(acc_dif_info_msg);
    joint_states_pub.publish(joint_state_msg);
    time_msg.data = control_time_;
    time_pub.publish(time_msg);
    if (dc.mode == "realrobot")
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            motor_info_msg.motorinfo1[i] = dc.torqueElmo[i];
            motor_info_msg.motorinfo2[i] = dc.torqueDemandElmo[i];
        }
        motor_info_pub.publish(motor_info_msg);
    }
    tgain_p.data = dc.t_gain;
    tgainPublisher.publish(tgain_p);

    pointpub_msg.header.stamp = ros::Time::now();
    Eigen::Vector3d temp;
    pointpub_msg.polygon.points[0].x = dc.com_.pos(0); //com_pos(0);
    pointpub_msg.polygon.points[0].y = dc.com_.pos(1);
    pointpub_msg.polygon.points[0].z = dc.com_.pos(2);

    pointpub_msg.polygon.points[1].x = link_[Right_Foot].xpos(0);
    pointpub_msg.polygon.points[1].y = link_[Right_Foot].xpos(1);
    pointpub_msg.polygon.points[1].z = link_[Right_Foot].xpos(2); // - dc.tocabi_.yaw;

    pointpub_msg.polygon.points[2].x = link_[Left_Foot].xpos(0);
    pointpub_msg.polygon.points[2].y = link_[Left_Foot].xpos(1);
    pointpub_msg.polygon.points[2].z = link_[Left_Foot].xpos(2);

    Eigen::Vector3d pelv_mod(0.11, 0, 0.02);
    Eigen::Vector3d pelv_pos = dc.tocabi_.link_[Pelvis].xpos + dc.tocabi_.link_[Pelvis].Rotm * pelv_mod;
    //dc.tocabi_.link_[Pelvis].Get_PointPos()

    pointpub_msg.polygon.points[3].x = pelv_pos(0);
    pointpub_msg.polygon.points[3].y = pelv_pos(1);
    pointpub_msg.polygon.points[3].z = pelv_pos(2);

    pointpub_msg.polygon.points[4].x = dc.tocabi_.roll;
    pointpub_msg.polygon.points[4].y = dc.tocabi_.pitch;
    pointpub_msg.polygon.points[4].z = dc.tocabi_.yaw;

    pointpub_msg.polygon.points[5].x = dc.tocabi_.link_[Right_Hand].xpos(0);
    pointpub_msg.polygon.points[5].y = dc.tocabi_.link_[Right_Hand].xpos(1);
    pointpub_msg.polygon.points[5].z = dc.tocabi_.link_[Right_Hand].xpos(2);

    pointpub_msg.polygon.points[6].x = dc.tocabi_.link_[Left_Hand].xpos(0);
    pointpub_msg.polygon.points[6].y = dc.tocabi_.link_[Left_Hand].xpos(1);
    pointpub_msg.polygon.points[6].z = dc.tocabi_.link_[Left_Hand].xpos(2);

    pointpub_msg.polygon.points[7].x = dc.tocabi_.ZMP(0); //from task torque -> contact force -> zmp
    pointpub_msg.polygon.points[7].y = dc.tocabi_.ZMP(1);
    pointpub_msg.polygon.points[7].z = dc.tocabi_.ZMP(2);

    Eigen::Matrix3d tm;
    tm = link_[Left_Foot].Rotm;
    tf2::Matrix3x3 m(tm(0, 0), tm(0, 1), tm(0, 2), tm(1, 0), tm(1, 1), tm(1, 2), tm(2, 0), tm(2, 1), tm(2, 2));
    double ltr, ltp, lty;
    m.getRPY(ltr, ltp, lty);

    pointpub_msg.polygon.points[8].x = ltr;
    pointpub_msg.polygon.points[8].y = ltp;
    pointpub_msg.polygon.points[8].z = lty;

    tm = link_[Right_Foot].Rotm;
    tf2::Matrix3x3 m2(tm(0, 0), tm(0, 1), tm(0, 2), tm(1, 0), tm(1, 1), tm(1, 2), tm(2, 0), tm(2, 1), tm(2, 2));
    double rtr, rtp, rty;
    m2.getRPY(rtr, rtp, rty);

    double mod = 180.0 * 100 / 3.141592;

    pointpub_msg.polygon.points[9].x = rtr;
    pointpub_msg.polygon.points[9].y = rtp;
    pointpub_msg.polygon.points[9].z = rty;

    // use points below :)

    pointpub_msg.polygon.points[10].x = LF_CF_FT(0);
    pointpub_msg.polygon.points[10].y = LF_CF_FT(1);
    pointpub_msg.polygon.points[10].z = LF_CF_FT(2);

    pointpub_msg.polygon.points[11].x = LF_CF_FT(3);
    pointpub_msg.polygon.points[11].y = LF_CF_FT(4);
    pointpub_msg.polygon.points[11].z = LF_CF_FT(5);

    pointpub_msg.polygon.points[12].x = RF_CF_FT(0);
    pointpub_msg.polygon.points[12].y = RF_CF_FT(1);
    pointpub_msg.polygon.points[12].z = RF_CF_FT(2);

    pointpub_msg.polygon.points[13].x = RF_CF_FT(3);
    pointpub_msg.polygon.points[13].y = RF_CF_FT(4);
    pointpub_msg.polygon.points[13].z = RF_CF_FT(5);

    pointpub_msg.polygon.points[14].x = dc.tocabi_.link_[Right_Hand].xpos(0);
    pointpub_msg.polygon.points[14].y = dc.tocabi_.link_[Right_Hand].xpos(1);
    pointpub_msg.polygon.points[14].z = dc.tocabi_.link_[Right_Hand].xpos(2);

    //pointpub_msg.polygon.points[14].x = dc.tocabi_.ZMP_eqn_calc(0); //from zmp dynamics
    //pointpub_msg.polygon.points[14].y = dc.tocabi_.ZMP_eqn_calc(1);
    //pointpub_msg.polygon.points[14].z = dc.tocabi_.ZMP_eqn_calc(2);

    pointpub_msg.polygon.points[15].x = link_local[Right_Foot].v(0);
    pointpub_msg.polygon.points[15].y = link_local[Right_Foot].v(1);
    pointpub_msg.polygon.points[15].z = link_local[Right_Foot].v(2);

    dc.tocabi_.ZMP_command = dc.tocabi_.com_.pos - dc.tocabi_.com_.pos(2) / 9.81 * dc.tocabi_.link_[COM_id].a_traj;

    pointpub_msg.polygon.points[16].x = link_local[Right_Foot].xpos(0);
    pointpub_msg.polygon.points[16].y = link_local[Right_Foot].xpos(1);
    pointpub_msg.polygon.points[16].z = link_local[Right_Foot].xpos(2);

    pointpub_msg.polygon.points[17].x = dc.tocabi_.ContactForce(3) / dc.tocabi_.ContactForce(2);
    pointpub_msg.polygon.points[17].y = dc.tocabi_.ContactForce(3 + 6) / dc.tocabi_.ContactForce(2 + 6);
    pointpub_msg.polygon.points[17].z = RF_CP_est(2);
    point_pub.publish(pointpub_msg);

    for (int i = 0; i < 2; i++)
    {
        ft_viz_msg.markers[i].header.stamp = ros::Time::now();
    }

    ft_viz_msg.markers[0].points[0].x = LF_CF_FT(0);
    ft_viz_msg.markers[0].points[0].y = LF_CF_FT(1);
    ft_viz_msg.markers[0].points[0].z = LF_CF_FT(2);

    ft_viz_msg.markers[0].points[1].x = LF_CF_FT(3);
    ft_viz_msg.markers[0].points[1].y = LF_CF_FT(4);
    ft_viz_msg.markers[0].points[1].z = LF_CF_FT(5);

    ft_viz_msg.markers[1].points[0].x = RF_CF_FT(0);
    ft_viz_msg.markers[1].points[0].y = RF_CF_FT(1);
    ft_viz_msg.markers[1].points[0].z = RF_CF_FT(2);

    ft_viz_msg.markers[1].points[1].x = RF_CF_FT(3);
    ft_viz_msg.markers[1].points[1].y = RF_CF_FT(4);
    ft_viz_msg.markers[1].points[1].z = RF_CF_FT(5);

    ft_viz_pub.publish(ft_viz_msg);
}
void StateManager::initYaw()
{
    tf2::Quaternion q(q_virtual_local_(3), q_virtual_local_(4), q_virtual_local_(5), q_virtual_local_(MODEL_DOF_VIRTUAL));
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    if (dc.tocabi_.signal_yaw_init)
    {
        std::cout << "Yaw Initialized" << std::endl;
        dc.tocabi_.yaw_init = yaw;
    }

    //const tf2Scalar& r_,p_,y_;

    tf2::Quaternion q_mod;
    yaw = yaw - dc.tocabi_.yaw_init;

    q_mod.setRPY(roll, pitch, yaw);
    //tf2::Quaternion q_rot;
    //q_rot.setRPY(0, 0, -yaw_init);
    //q = q * q_rot;

    q_virtual_local_(3) = q_mod.getX();
    q_virtual_local_(4) = q_mod.getY();
    q_virtual_local_(5) = q_mod.getZ();
    q_virtual_local_(MODEL_DOF_VIRTUAL) = q_mod.getW();
}

void StateManager::imuCompenstation()
{
    if (dc.tocabi_.ee_[0].contact && (!dc.tocabi_.ee_[1].contact)) //dc.tocabi_.cont)
    {
    }
    else if (dc.tocabi_.ee_[1].contact && (!dc.tocabi_.ee_[0].contact))
    {
        tf2::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(MODEL_DOF_VIRTUAL));
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_mod;
        q_mod.setRPY(roll + 1.0 / 180.0 * 3.141592, pitch, yaw);

        q_virtual_(3) = q_mod.getX();
        q_virtual_(4) = q_mod.getY();
        q_virtual_(5) = q_mod.getZ();
        q_virtual_(MODEL_DOF_VIRTUAL) = q_mod.getW();
    }
}

void StateManager::connect()
{
    //overrid
}
void StateManager::updateState()
{
    //overrid by simulation or red robot
}

void StateManager::sendCommand(Eigen::VectorQd command, double simt, int control_mode)
{
    //overrid by simulation or red robot
}

void StateManager::pelvisPosMonitor()
{
}

void StateManager::initialize()
{
    data_received_counter_ = 0;

    A_.setZero();
    A_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    Motor_inertia_.setZero();
    q_.setZero();
    q_dot_.setZero();

    q_virtual_.setZero();
    q_dot_virtual_.setZero();
    q_ddot_virtual_.setZero();
    q_dot_virtual_lpf_.setZero();
    q_dot_virtual_lpf_before.setZero();
    q_dot_virtual_raw_.setZero();
    q_ddot_virtual_lpf_.setZero();
    q_dot_before_.setZero();
    q_dot_virtual_before.setZero();

    q_virtual_local_.setZero();
    q_dot_virtual_local_.setZero();
    q_ddot_virtual_local_.setZero();

    torque_desired.setZero();

    dc.torqueElmo.setZero();
}

void StateManager::storeState()
{

    mtx_dc.lock();

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        dc.link_[i] = link_[i];
    }
    dc.com_ = com_;
    dc.time = control_time_;
    dc.sim_time = sim_time_;

    dc.q_ = q_virtual_.segment(6, MODEL_DOF);
    dc.q_dot_ = q_dot_virtual_.segment(6, MODEL_DOF);
    dc.q_dot_virtual_ = q_dot_virtual_;
    dc.q_dot_virtual_lpf = q_dot_virtual_lpf_;
    dc.q_virtual_ = q_virtual_;
    dc.q_ddot_virtual_ = q_ddot_virtual_;
    dc.q_ext_ = q_ext_;
    //dc.q_dot_est_ = q_dot_est;

    dc.tau_nonlinear_ = tau_nonlinear_;

    dc.yaw = yaw;
    dc.roll = roll;
    dc.pitch = pitch;

    dc.A_ = A_;
    dc.A_inv = A_inv;
    dc.Motor_inertia = Motor_inertia_;
    dc.Motor_inertia_inverse = Motor_inertia_inv;

    dc.tocabi_.ContactForce_FT.segment(0, 6) = LF_CF_FT;
    dc.tocabi_.ContactForce_FT.segment(6, 6) = RF_CF_FT;
    dc.tocabi_.ContactForce_FT_raw.segment(0, 6) = LF_FT;
    dc.tocabi_.ContactForce_FT_raw.segment(6, 6) = RF_FT;

    Eigen::Matrix6d Rotm;
    Rotm.setZero();
    Rotm.block(0, 0, 3, 3) = dc.tocabi_.link_[Right_Hand].Rotm;
    Rotm.block(3, 3, 3, 3) = dc.tocabi_.link_[Right_Hand].Rotm;

    dc.tocabi_.RH_FT = Rotm * RH_FT;

    Rotm.block(0, 0, 3, 3) = dc.tocabi_.link_[Left_Hand].Rotm;
    Rotm.block(3, 3, 3, 3) = dc.tocabi_.link_[Left_Hand].Rotm;
    dc.tocabi_.LH_FT = Rotm * LH_FT;

    dc.tocabi_.com_ = com_;

    mtx_dc.unlock();
}
void StateManager::storeSync()
{
    //dc.tocabi_.
}

void StateManager::updateKinematics(RigidBodyDynamics::Model &model_l, Link *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

    //std::cout << control_time_ << " : q_v(0) : " << q_virtual(0) << " : q_v(1) : " << q_virtual(1) << " : q_v(2) : " << q_virtual(2) << std::endl;

    mtx_rbdl.lock();
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);

    A_temp_.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_f, A_temp_, false);

    //Eigen::VectorXd tau_coriolis;
    //RigidBodyDynamics::NonlinearEffects(model_,q_virtual_,q_dot_virtual_,tau_coriolis);
    mtx_rbdl.unlock();
    //tf2::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(MODEL_DOF + 6));

    A_ = A_temp_;
    A_inv = A_.inverse();
    for (int i = 0; i < 6; ++i)
    {
        Motor_inertia_(i, i) = 10.0;
    }
    for (int i = 0; i < 2; ++i)
    {
        Motor_inertia_(6 + 6 * i, 6 + 6 * i) = 0.56;
        Motor_inertia_(7 + 6 * i, 7 + 6 * i) = 0.8;
        Motor_inertia_(8 + 6 * i, 8 + 6 * i) = 1.08;
        Motor_inertia_(9 + 6 * i, 9 + 6 * i) = 1.08;
        Motor_inertia_(10 + 6 * i, 10 + 6 * i) = 1.08;
        Motor_inertia_(11 + 6 * i, 11 + 6 * i) = 0.306;

        Motor_inertia_(21 + 10 * i, 21 + 10 * i) = 0.185;
        Motor_inertia_(22 + 10 * i, 22 + 10 * i) = 0.184;
        Motor_inertia_(23 + 10 * i, 23 + 10 * i) = 0.192;
        Motor_inertia_(24 + 10 * i, 24 + 10 * i) = 0.184;
        Motor_inertia_(25 + 10 * i, 25 + 10 * i) = 0.056;
        Motor_inertia_(26 + 10 * i, 26 + 10 * i) = 0.05;
        Motor_inertia_(27 + 10 * i, 27 + 10 * i) = 0.015;
        Motor_inertia_(28 + 10 * i, 28 + 10 * i) = 0.015;
    }
    Motor_inertia_(18, 18) = 1.01;
    Motor_inertia_(19, 19) = 1.01;
    Motor_inertia_(20, 20) = 1.27;
    Motor_inertia_(29, 29) = 0.015;
    Motor_inertia_(30, 30) = 0.015;

    Motor_inertia_inv = Motor_inertia_.inverse();

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].pos_Update(model_l, q_virtual_f);
    }
    Eigen::Vector3d zero;
    zero.setZero();
    dc.check = true;
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].Set_Jacobian(model_l, q_virtual_f, zero);
    }
    dc.check = false;

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {

        link_p[i].COM_Jac_Update(model_l, q_virtual_f);
    }
    //COM link information update ::
    double com_mass;
    RigidBodyDynamics::Math::Vector3d com_pos;
    RigidBodyDynamics::Math::Vector3d com_vel, com_accel, com_ang_momentum;
    mtx_rbdl.lock();
    RigidBodyDynamics::Utils::CalcCenterOfMass(model_l, q_virtual_f, q_dot_virtual_f, &q_ddot_virtual_f, com_mass, com_pos, &com_vel, &com_accel, &com_ang_momentum, NULL, false);
    mtx_rbdl.unlock();

    RigidBodyDynamics::ConstraintSet CS;

    //CS.AddContactConstraint(link_[Right_Foot].id,)

    //ROS_INFO_ONCE("TOTAL MASS : %f", com_mass);
    com_.mass = com_mass;
    com_.pos = com_pos;

    //ROS_INFO_ONCE("COM POS : %f %f %f", com_pos(0), com_pos(1), com_pos(2));
    //RigidBodyDynamics::Utils::
    /*
    if (com_pos(1) < link_[Right_Foot].xpos(1))
    {
        std::cout << control_time_ << "COM_Y OUT WARNING !!!!!!!!!!!!!!!!" << std::endl;
    }
    else if (com_pos(1) > link_[Left_Foot].xpos(1))
    {
        std::cout << control_time_ << "COM_Y OUT WARNING !!!!!!!!!!!!!!!!" << std::endl;
    }
    if ((com_pos(0) < link_[Right_Foot].xpos(0)) && (com_pos(0) < link_[Left_Foot].xpos(0)))
    {
        std::cout << control_time_ << "COM_Y OUT WARNING !!!!!!!!!!!!!!!!" << std::endl;
    }
    else if (com_pos(0) > link_[Left_Foot].xpos(0))
    {
        std::cout << control_time_ << "COM_Y OUT WARNING !!!!!!!!!!!!!!!!" << std::endl;
    } */

    Eigen::Vector3d foot_ahead_pos(0.15, 0, 0);
    Eigen::Vector3d foot_back_pos(-0.09, 0, 0);
    Eigen::Vector3d RH, RT, LH, LT;

    RH = link_p[Right_Foot].xpos + link_p[Right_Foot].Rotm * foot_ahead_pos;
    RT = link_p[Right_Foot].xpos + link_p[Right_Foot].Rotm * foot_back_pos;

    LH = link_p[Left_Foot].xpos + link_p[Left_Foot].Rotm * foot_ahead_pos;
    LT = link_p[Left_Foot].xpos + link_p[Left_Foot].Rotm * foot_back_pos;

    double s[4];

    s[0] = DyrosMath::check_border(com_.pos(0), com_.pos(1), RH(0), RT(0), RH(1), RT(1), -1.0);
    s[1] = DyrosMath::check_border(com_.pos(0), com_.pos(1), RT(0), LT(0), RT(1), LT(1), -1.0);
    s[2] = DyrosMath::check_border(com_.pos(0), com_.pos(1), LT(0), LH(0), LT(1), LH(1), -1.0);
    s[3] = DyrosMath::check_border(com_.pos(0), com_.pos(1), LH(0), RH(0), LH(1), RH(1), -1.0);
    //std::cout << " com pos : x " << com_.pos(0) << "\t" << com_.pos(1) << std::endl;
    //std::cout << "check sign ! \t" << s[0] << "\t" << s[1] << "\t" << s[2] << "\t" << s[3] << std::endl;

    for (int i = 0; i < 4; i++)
    {
        if (s[i] < 0)
        {
            if (dc.spalarm)
                std::cout << control_time_ << "com is out of support polygon !, line " << i << std::endl;
        }
    }

    /*
    s[1] = DyrosMath::check_border(com_.pos(0), com_.pos(1), RT(0), LT(0), RT(1), LT(1), 1.0);
    std::cout << " s[1] : " << s[1] << std::endl; */
    Eigen::Vector3d vel_temp;
    vel_temp = com_.vel;
    com_.vel = com_vel;

    com_.accel = com_accel;
    com_.angular_momentum = com_ang_momentum;

    double w_ = sqrt(9.81 / com_.pos(2));

    com_.ZMP(0) = com_.pos(0) - com_.accel(0) / pow(w_, 2);
    com_.ZMP(1) = com_.pos(1) - com_.accel(1) / pow(w_, 2);

    //com_.ZMP(0) = (com_.pos(0) * (com_.accel(2) + 9.81) - com_pos(2) * com_accel(0)) / (com_.accel(2) + 9.81) - com_.angular_momentum(2) / com_.mass / (com_.accel(2) + 9.81);
    //com_.ZMP(1) = (com_.pos(1) * (com_.accel(2) + 9.81) - com_pos(2) * com_accel(1)) / (com_.accel(2) + 9.81) - com_.angular_momentum(1) / com_.mass / (com_.accel(2) + 9.81);

    com_.CP(0) = com_.pos(0) + com_.vel(0) / w_;
    com_.CP(1) = com_.pos(1) + com_.vel(1) / w_;

    Eigen::Matrix3Vd jacobian_com;

    jacobian_com.setZero();

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        jacobian_com += link_p[i].Jac_COM_p * link_p[i].Mass;
    }

    link_p[COM_id].Jac.setZero(6, MODEL_DOF + 6);

    //link_p[COM_id].Jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6) / com_.mass;
    //link_p[COM_id].Jac.block(2, 0, 4, MODEL_DOF + 6) = link_p[Pelvis].Jac.block(2, 0, 4, MODEL_DOF + 6);

    link_p[COM_id].Jac.block(0, 0, 3, MODEL_DOF + 6) = jacobian_com.block(0, 0, 3, MODEL_DOF + 6) / com_.mass;
    link_p[COM_id].Jac.block(3, 0, 3, MODEL_DOF + 6) = link_p[Pelvis].Jac.block(3, 0, 3, MODEL_DOF + 6);

    link_p[COM_id].Jac_COM_p = jacobian_com / com_.mass;

    link_p[COM_id].xpos = com_.pos;
    //link_p[COM_id].xpos(2) = link_p[Pelvis].xpos(2);
    link_p[COM_id].Rotm = link_p[Pelvis].Rotm;

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        link_p[i].vw_Update(q_dot_virtual_f);
    }

    RigidBodyDynamics::Math::VectorNd tau_;
    tau_.resize(model_l.qdot_size);
    RigidBodyDynamics::NonlinearEffects(model_l, q_virtual_f, q_dot_virtual_f, tau_);
    tau_nonlinear_ = tau_;

    //contactJacUpdate
    //link_[Right_Foot].Set_Contact(model_, q_virtual_, link_[Right_Foot].contact_point);
    //link_[Left_Foot].Set_Contact(model_, q_virtual_, link_[Left_Foot].contact_point);
    //link_[Right_Hand].Set_Contact(model_, q_virtual_, link_[Right_Hand].contact_point);
    //link_[Left_Hand].Set_Contact(model_, q_virtual_, link_[Left_Hand].contact_point);
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics end ");
}

void StateManager::handleFT()
{

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
    Wrench_foot_plate(2) = -foot_plate_mass * GRAVITY;

    RF_CF_FT = rotrf * adt * RF_FT + adt2 * Wrench_foot_plate;

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
    Wrench_foot_plate(2) = -foot_plate_mass * GRAVITY;

    LF_CF_FT = rotrf * adt * LF_FT + adt2 * Wrench_foot_plate;

    LF_CF_FT_local = rotrf.inverse() * LF_CF_FT;
}

void StateManager::contactEstimate()
{
    RF_FT_ZMP_local(0) = -RF_CF_FT_local(4) / RF_CF_FT_local(2);
    RF_FT_ZMP_local(1) = RF_CF_FT_local(3) / RF_CF_FT_local(2);

    LF_FT_ZMP_local(0) = -LF_CF_FT_local(4) / LF_CF_FT_local(2);
    LF_FT_ZMP_local(1) = LF_CF_FT_local(3) / LF_CF_FT_local(2);

    RF_CP_est(0) = DyrosMath::minmax_cut(RF_FT_ZMP_local(0), -0.12, 0.18);
    RF_CP_est(1) = DyrosMath::minmax_cut(RF_FT_ZMP_local(1), -0.065, 0.065);
    RF_CP_est(2) = 0;

    LF_CP_est(0) = DyrosMath::minmax_cut(LF_FT_ZMP_local(0), -0.12, 0.18);
    LF_CP_est(1) = DyrosMath::minmax_cut(LF_FT_ZMP_local(1), -0.065, 0.065);
    LF_CP_est(2) = 0;

    double Contact_TH = -50;

    if (RF_CF_FT(2) < Contact_TH)
    {
        if (!RF_Contact)
        {
            //std::cout << "RF Contact On" << std::endl;
            RF_Contact = true;
        }
    }
    else
    {
        if (RF_Contact)
        {
            //std::cout << "RF Contact Off" << std::endl;
            RF_Contact = false;
        }
    }

    if (LF_CF_FT(2) < Contact_TH)
    {
        if (!LF_Contact)
        {
            //std::cout << "LF Contact On" << std::endl;
            LF_Contact = true;
        }
    }
    else
    {
        if (LF_Contact)
        {
            //std::cout << "LF Contact Off" << std::endl;
        }
        LF_Contact = false;
    }

    if ((!LF_Contact) && (!RF_Contact))
    {
        //std::cout << "both Contact Disabled " << std::endl;
    }

    //RF_CF_FT
    //FT Sensor Pos : 0,0,-0.075
    //Foot Pos : 0,0,-0.1585
    //Foot Length : x+ 0.18 x- 0.12 y+- 0.065
    //Estimated Contact Point =
    //See zmp for each foot by ft sensor.

    //with ft, contact force Z means,

    //Contact Uncertainty.
}

void StateManager::qdotLPF()
{
    if (dc.enable_lpf)
    {
        if (dc.switch_lpf)
        {
            dc.switch_lpf = false;
            std::cout << "q_dot lowpassfilter Off" << std::endl;
        }
        else
        {
            dc.switch_lpf = true;
            std::cout << "q_dot lowpassfilter On" << std::endl;
        }
        dc.enable_lpf = false;
        q_dot_virtual_lpf_before = q_dot_virtual_local_;
    }

    if (dc.switch_lpf)
    {
        q_dot_virtual_lpf_ = DyrosMath::lpf(q_dot_virtual_local_, q_dot_virtual_lpf_before, 2000, 200);
        q_dot_virtual_lpf_.segment(0, 3).setZero();
        q_dot_virtual_lpf_before = q_dot_virtual_lpf_;

        q_ddot_virtual_local_ = (q_dot_virtual_lpf_ - q_dot_virtual_before) * 2000;

        q_ddot_virtual_lpf_ = DyrosMath::lpf(q_ddot_virtual_local_, q_ddot_virtual_before_, 2000, 200);

        q_ddot_virtual_before_ = q_ddot_virtual_lpf_;

        //q_dot_virtual_local_ = q_dot_virtual_lpf_;
        q_dot_virtual_ = q_dot_virtual_local_;
    }
    else
    {
        q_dot_virtual_ = q_dot_virtual_local_;
    }
}

void StateManager::sendStateToGui()
{
    if (dc.mode == "realrobot")
    {
        syspub_msg.data[0] = dc.imu_state;
        syspub_msg.data[1] = dc.zp_state;
        syspub_msg.data[2] = dc.ft_state;
        syspub_msg.data[3] = dc.ecat_state;
        syspub_msg.data[4] = dc.semode;
        syspub_msg.data[5] = dc.tc_state;
    }
    else if (dc.mode == "simulation")
    {
        syspub_msg.data[0] = 3;
        syspub_msg.data[1] = 3;
        syspub_msg.data[2] = 3;
        syspub_msg.data[3] = 3;
        syspub_msg.data[4] = dc.semode;
        syspub_msg.data[5] = dc.tc_state;
    }
    gui_state_pub.publish(syspub_msg);
}

void StateManager::stateEstimate()
{
    if (dc.semode && (!dc.tocabi_.signal_yaw_init))
    {
        static bool contact_right, contact_left;
        static Eigen::Vector3d RF_contact_pos_holder, LF_contact_pos_holder;
        static Eigen::Vector3d RF_contact_pos_mod, LF_contact_pos_mod;
        static Eigen::Vector3d RF_CP_est_holder, LF_CP_est_holder;
        static Eigen::Vector3d RF_CP_est_holder_before, LF_CP_est_holder_before;
        static Eigen::Vector3d RF_CP_est_before, LF_CP_est_before;
        static Eigen::Vector3d imu_init;
        RF_CP_est.setZero();
        LF_CP_est.setZero();

        static Eigen::Vector3d pelv_v_before;
        static Eigen::Vector3d pelv_v;
        static Eigen::Vector3d pelv_x_before;
        static Eigen::Vector3d pelv_x;
        RF_contact_pos_mod = RF_CP_est - RF_CP_est_before;
        LF_contact_pos_mod = LF_CP_est - LF_CP_est_before;

        RF_CP_est_before = RF_CP_est;
        LF_CP_est_before = LF_CP_est;

        Eigen::Vector3d RF_contactpoint_base_pos = link_local[Right_Foot].contact_point;
        Eigen::Vector3d LF_contactpoint_base_pos = link_local[Left_Foot].contact_point;
        Eigen::Vector3d RF_contactpoint_internal_pos = link_local[Right_Foot].contact_point + RF_CP_est;
        Eigen::Vector3d LF_contactpoint_internal_pos = link_local[Left_Foot].contact_point + LF_CP_est;
        Eigen::Vector3d mod_base_pos;
        Eigen::Vector3d mod_base_vel;
        Eigen::Vector3d rf_cp_m, lf_cp_m;
        Eigen::Vector3d RF_fixed_contact_pos, LF_fixed_contact_pos, RF_global_contact_pos, LF_global_contact_pos;
        Eigen::Vector3d RF_global_hold_pos, LF_global_hold_pos;
        Eigen::Vector6d RF_global_contact_vel, LF_global_contact_vel;
        Eigen::Vector6d RF_fixed_contact_vel, LF_fixed_contact_vel;
        Eigen::Vector3d RF_P_cpm, LF_P_cpm;

        link_[Right_Foot].Get_PointPos(q_virtual_, q_dot_virtual_, RF_contactpoint_internal_pos, RF_global_contact_pos, RF_global_contact_vel);
        link_[Left_Foot].Get_PointPos(q_virtual_, q_dot_virtual_, LF_contactpoint_internal_pos, LF_global_contact_pos, LF_global_contact_vel);

        link_local[Right_Foot].Get_PointPos(q_virtual_local_, q_dot_virtual_local_, RF_contactpoint_internal_pos, RF_fixed_contact_pos, RF_fixed_contact_vel);
        link_local[Left_Foot].Get_PointPos(q_virtual_local_, q_dot_virtual_local_, LF_contactpoint_internal_pos, LF_fixed_contact_pos, LF_fixed_contact_vel);

        bool local_RF_Contact, local_LF_contact;

        if (dc.sebyft)
        {
            local_LF_contact = LF_Contact;
            local_RF_Contact = RF_Contact;
        }
        else
        {
            local_LF_contact = dc.tocabi_.ee_[0].contact;
            local_RF_Contact = dc.tocabi_.ee_[1].contact;
        }

        bool left_change, right_change;
        left_change = false;
        right_change = false;
        if (contact_right != local_RF_Contact)
        {
            right_change = true;
            if (local_RF_Contact)
            {
                std::cout << control_time_ << "  right foot contact initialized" << std::endl;
                RF_contact_pos_holder = RF_global_contact_pos;
                RF_CP_est_holder_before = RF_CP_est_holder;
                RF_CP_est_holder = RF_CP_est;
            }
            else
            {
                std::cout << control_time_ << "  right foot contact disabled" << std::endl;
            }
        }
        if (contact_left != local_LF_contact)
        {
            left_change = true;
            if (local_LF_contact)
            {
                std::cout << control_time_ << "  left foot contact initialized" << std::endl;
                LF_contact_pos_holder = LF_global_contact_pos;
                LF_CP_est_holder_before = LF_CP_est_holder;
                LF_CP_est_holder = LF_CP_est;
            }
            else
            {
                std::cout << control_time_ << "  left foot contact disabled" << std::endl;
            }
        }

        if (dc.semode_init)
        {
            std::cout << "state Estimation Initialized" << std::endl;
            RF_contact_pos_holder(2) = 0.0; // - RF_contactpoint_internal_pos(2);
            LF_contact_pos_holder(2) = 0.0; // - LF_contactpoint_internal_pos(2);
            RF_contact_pos_mod.setZero();
            LF_contact_pos_mod.setZero();
            RF_CP_est_before.setZero();
            LF_CP_est_before.setZero();
            dc.semode_init = false;
            pelv_v_before.setZero();
            pelv_x_before.setZero();
            imu_init = link_local[Pelvis].Rotm * imu_lin_acc;
        }

        // imu pos estimation part (useless for now... )
        imu_lin_acc_lpf = DyrosMath::lpf(imu_lin_acc, imu_lin_acc_before, 2000, 20);
        imu_lin_acc_before = imu_lin_acc_lpf;
        pelv_lin_acc = dc.link_[Pelvis].Rotm.inverse() * imu_lin_acc_lpf;
        double dt_i = 1.0 / 2000.0;
        Vector3d temp;
        temp = dc.tocabi_.imu_vel_ + dt_i * pelv_lin_acc;
        dc.tocabi_.imu_vel_ = temp;
        temp = dc.tocabi_.imu_pos_ + (dt_i * dt_i / 0.5) * pelv_lin_acc + dc.tocabi_.imu_vel_ * dt_i;
        dc.tocabi_.imu_pos_ = temp;
        // imu estimate end

        RF_P_cpm = link_local[Right_Foot].Rotm * (RF_CP_est - RF_CP_est_holder);
        LF_P_cpm = link_local[Left_Foot].Rotm * (LF_CP_est - LF_CP_est_holder);

        RF_contact_pos_holder = RF_contact_pos_holder + RF_P_cpm;
        LF_contact_pos_holder = LF_contact_pos_holder + LF_P_cpm;

        //Vector3d es_zmp;

        //es_zmp = dc.tocabi_.com_.pos - dc.tocabi_.com_.pos(2)/9.81*

        contact_right = local_RF_Contact;
        contact_left = local_LF_contact;

        rf_cp_m = RF_fixed_contact_pos - RF_contact_pos_holder;
        lf_cp_m = LF_fixed_contact_pos - LF_contact_pos_holder;

        double dr, dl;
        //dr =

        dr = DyrosMath::minmax_cut(RF_CF_FT(2) / (-com_.mass * GRAVITY), 0, 1);
        dl = DyrosMath::minmax_cut(LF_CF_FT(2) / (-com_.mass * GRAVITY), 0, 1);

        if (dr == 1)
        {
            dl = 0;
        }
        else if (dr == 0)
        {
            dl = 1;
        }

        if (dl == 1)
        {
            dr = 0;
        }
        else if (dl == 0)
        {
            dr = 1;
        }
        rf_s_ratio = dr / (dr + dl);
        lf_s_ratio = dl / (dl + dr);

        lf_s_ratio = DyrosMath::minmax_cut(lf_s_ratio, 0, 1);
        if (lf_s_ratio == 0)
        {
            rf_s_ratio = 1;
        }
        else if (lf_s_ratio == 1)
        {
            rf_s_ratio = 0;
        }
        else
        {
            rf_s_ratio = DyrosMath::minmax_cut(rf_s_ratio, 0, 1);
        }
        //std::cout << " dr : " << dr << "  dl : " << dl << "  rf_s_ratio : " << rf_s_ratio << "  lf_s_ratio : " << lf_s_ratio << std::endl;
        if (contact_right && contact_left)
        {
            //std::cout << control_time_ << " : base pos calc ! " << std::endl;
            mod_base_pos = rf_cp_m * rf_s_ratio + lf_cp_m * lf_s_ratio;
            //mod_base_pos(2) = mod_base_pos(2) + ((link_[Right_Foot].xpos(2) + link_[Right_Foot].contact_point(2)) * rf_s_ratio/ (rf_s_ratio + lf_s_ratio) + (link_[Left_Foot].xpos(2) + link_[Left_Foot].contact_point(2)) * lf_s_ratio / (rf_s_ratio + lf_s_ratio));
            mod_base_vel = -RF_fixed_contact_vel.segment(3, 3) * rf_s_ratio - LF_fixed_contact_vel.segment(3, 3) * lf_s_ratio;
        }
        else if (contact_right && (!contact_left))
        {
            mod_base_pos = rf_cp_m;
            mod_base_vel = -RF_fixed_contact_vel.segment(3, 3);
        }
        else if (contact_left && (!contact_right))
        {
            mod_base_pos = lf_cp_m;
            mod_base_vel = -LF_fixed_contact_vel.segment(3, 3);
        }
        else
        {
            std::cout << "whatthefuck" << std::endl;
        }

        if (dc.single_foot_only)
        {
            mod_base_pos = rf_cp_m;
            mod_base_vel = -RF_fixed_contact_vel.segment(3, 3);
        }

        //Pelvis Velocity Complementary filter
        //v = alpha *(pelv_imu_acc * dt + v_before) + (1-alpha)*mb_v

        Vector3d imu_acc_dat;
        imu_acc_dat = link_local[Pelvis].Rotm * imu_lin_acc;

        imu_acc_dat = imu_acc_dat - imu_init;

        double dt = 0.0005;
        double tau = 0.6;
        double alpha = tau / (tau + dt);

        pelv_v = alpha * (imu_acc_dat * dt + pelv_v_before) + (1 - alpha) * mod_base_vel;
        pelv_v_before = pelv_v;
        q_virtual_ = q_virtual_local_;
        //q_dot_virtual_ = q_dot_virtual_local_;

        //std::cout<<"pelv_v es : "<<pelv_v_before.transpose()<<" imu acc : "<<imu_acc_dat.transpose()<<"  imu init : "<<imu_init.transpose() <<" imu lin acc : "<<imu_lin_acc.transpose() <<" mod base : "<<mod_base_vel.transpose()<<std::endl;

        pelv_x = alpha * (pelv_v * dt + imu_acc_dat * dt * dt * 0.5 + pelv_x_before) + (1 - alpha) * (-mod_base_pos);
        pelv_x_before = pelv_x;
        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = -mod_base_pos(i);
            q_dot_virtual_(i) = mod_base_vel(i);
        }

        //acceleration calculation!
        //q_ddot_virtual_ = (q_dot_virtual_ - q_dot_virtual_before) / ((double)dc.ctime / 1000000.0);
        //q_dot_virtual_before = q_dot_virtual_;

        static Vector3d pelv_pos_before;

        Vector3d currentPelvPos = q_virtual_.segment(0, 3);

        Vector3d pos_err = currentPelvPos - pelv_pos_before;
        static Vector3d rf1, rf2, rf3, rf4;
        static bool rf_b, lf_b;
        bool problem_is_here = false;
        static VectorQVQd q_v_before;
        static bool err_before = true;
        if (dc.torqueOn && (control_time_ > (dc.torqueOnTime + 5.0)))
        {
            if (((currentPelvPos(0) == 0) && (currentPelvPos(1) == 0) && (currentPelvPos(2) == 0)) || ((pelv_pos_before(0) == 0) && (pelv_pos_before(1) == 0) && (pelv_pos_before(2) == 0)))
            {
            }
            else
            {

                for (int i = 0; i < 3; i++)
                {
                    if (pos_err(i) * 2000 > 50.0)
                        problem_is_here = true;
                }
            }
        }
        if (problem_is_here)
        {
            std::cout << cred << "WARNING :: PELV POSITION TRACKING ERROR :: BEFORE : " << pelv_pos_before.transpose() << "  NOW : " << currentPelvPos.transpose() << creset << std::endl;

            std::cout << "INFO : " << -mod_base_pos.transpose() << " RF ratio : " << rf_s_ratio << " LF ratio : " << lf_s_ratio << " RF ratio bf : " << rf_b << " LF ratio bf : " << lf_b << std::endl;
            std::cout << " RF fix cp : " << RF_fixed_contact_pos.transpose() << " RF cp hd : " << RF_contact_pos_holder.transpose() << " LF fix cp : " << LF_fixed_contact_pos.transpose() << " RF cp hdl : " << LF_contact_pos_holder.transpose() << std::endl;

            std::cout << " RF fix cp : " << rf1.transpose() << " RF cp hd : " << rf2.transpose() << " LF fix cp : " << rf3.transpose() << " RF cp hdl : " << rf4.transpose() << std::endl;

            std::cout << q_virtual_local_.transpose();
            std::cout << q_v_before.transpose();
            //q_virtual_.segment(0, 3) = pelv_pos_before;
            //currentPelvPos = pelv_pos_before;
        }
        q_v_before = q_virtual_local_;
        rf_b = rf_s_ratio;
        lf_b = lf_s_ratio;
        rf1 = RF_fixed_contact_pos;
        rf2 = RF_contact_pos_holder;
        rf3 = LF_fixed_contact_pos;
        rf4 = LF_contact_pos_holder;
        pelv_pos_before = currentPelvPos;
    }
    else
    {
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;
        q_ddot_virtual_ = q_ddot_virtual_local_;
    }
}

void StateManager::jointVelocityEstimate()
{
    //Estimate joint velocity using state observer
    double dt;
    dt = 1 / 2000;
    Eigen::MatrixXd A_t, A_dt, B_t, B_dt, C, I, I_t;
    I.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    I.setIdentity();
    I_t.setZero(MODEL_DOF, MODEL_DOF);
    I_t.setIdentity();
    A_t.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    A_dt.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    B_t.setZero(MODEL_DOF * 2, MODEL_DOF);
    B_dt.setZero(MODEL_DOF * 2, MODEL_DOF);
    C.setZero(MODEL_DOF, MODEL_DOF * 2);

    A_t.topRightCorner(MODEL_DOF, MODEL_DOF);
    A_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = A_inv.bottomRightCorner(MODEL_DOF, MODEL_DOF) * dc.tocabi_.Cor_;
    A_t.topRightCorner(MODEL_DOF, MODEL_DOF) = I_t;
    B_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = A_inv.bottomRightCorner(MODEL_DOF, MODEL_DOF);
    C.bottomLeftCorner(MODEL_DOF, MODEL_DOF) = I_t * dt;
    B_dt = B_t * dt;
    A_dt = I - dt * A_dt;

    double L, L1;
    L = 0.002;
    L1 = 0.1;

    if (velEst == false)
    {
        q_est = q_;
        q_dot_est = q_dot_;
        velEst = true;
    }

    if (velEst = true)
    {
        Eigen::VectorQd q_temp;
        Eigen::VectorVQd q_dot_virtual;

        q_dot_virtual = q_dot_virtual_;

        q_temp = q_est;

        q_est = q_est + dt * q_dot_est + L * (q_ - q_est);

        q_dot_virtual.segment<MODEL_DOF>(6) = q_dot_est;

        q_dot_est = (q_temp - q_est) * 2000;

        RigidBodyDynamics::Math::VectorNd tau_;
        tau_.resize(model_.qdot_size);

        RigidBodyDynamics::NonlinearEffects(model_, q_virtual_, q_dot_virtual, tau_);

        q_dot_est = -(q_dot_est + B_dt.bottomRightCorner(MODEL_DOF, MODEL_DOF) * (dc.torque_desired + L1 * (q_ - q_est) - tau_.segment<MODEL_DOF>(6)));
    }

    dc.tocabi_.q_dot_est = q_dot_est;
}

void StateManager::SetPositionPDGainMatrix()
{
    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc.tocabi_.Kps[i] = dc.tocabi_.vector_kp[i];
        dc.tocabi_.Kvs[i] = dc.tocabi_.vector_kv[i];
    }
}

void StateManager::CommandCallback(const std_msgs::StringConstPtr &msg)
{
    //std::cout << "msg from gui : " << msg->data << std::endl;
    dc.command = msg->data;

    if (msg->data == "torqueon")
    {
        if (dc.torqueOn)
        {
            std::cout << "torque is already enabled, command duplicated, ignoring command!" << std::endl;
        }
        else
        {
            dc.semode = true;
            dc.tocabi_.signal_yaw_init = true;
            std::cout << "torque ON !" << std::endl;
            dc.torqueOnTime = control_time_;
            dc.torqueOn = true;
            dc.torqueOff = false;
            dc.tocabi_.ee_[0].contact = true;
            dc.tocabi_.ee_[1].contact = true;
        }
    }
    else if (msg->data == "positioncontrol")
    {
        if (!dc.positionControl)
        {
            std::cout << "Joint position control : on " << std::endl;
            dc.commandTime = control_time_;
            dc.positionDesired = q_;
            dc.set_q_init = true;
            if (dc.position_command_ext)
            {
                std::cout << "Joint position control by positionCommandExt disabled" << std::endl;
                dc.position_command_ext = false;
            }
        }
        else
        {
            std::cout << "Joint position control : off " << std::endl;
        }

        dc.positionControl = !dc.positionControl;
    }
    else if (msg->data == "positiongravcontrol")
    {
        if (!dc.positionGravControl)
        {
            std::cout << "Joint Grav position control : on " << std::endl;
            dc.commandTime = control_time_;
            dc.positionDesired = q_;
        }
        else
        {
            std::cout << "Joint Grav position control : off " << std::endl;
        }
        dc.positionGravControl = !dc.positionGravControl;
    }
    else if (msg->data == "torqueoff")
    {
        if (dc.torqueOn)
        {
            std::cout << "Torque OFF ! " << std::endl;
            dc.torqueOffTime = control_time_;
            dc.torqueOn = false;
            dc.torqueOff = true;
            dc.signal_gravityCompensation = true;

            dc.tocabi_.contact_redistribution_mode = 0;

            dc.tc_state = 3;
        }
        else if (dc.torqueOff)
        {
            std::cout << "Torque is already disabled, command duplicated, ignoring command! " << std::endl;
        }
    }
    else if (msg->data == "gravity")
    {
        std::cout << "gravity compensation mode is on! " << std::endl;
        dc.commandTime = control_time_;
        dc.signal_gravityCompensation = true;

        dc.tocabi_.contact_redistribution_mode = 0;

        dc.tc_state = 3;
    }
    else if (msg->data == "emergencyoff")
    {
        std::cout << "!!!!!! RED DISABLED BY EMERGENCY OFF BUTTON !!!!!!" << std::endl;
        dc.emergencyoff = true;
        dc.torqueOn = false;
        dc.torqueOff = true;
    }
    else if (msg->data == "tunereset")
    {
        std::cout << "custom gain disabled, defualt gain enabled!" << std::endl;
        dc.customGain = false;
    }
    else if (msg->data == "tunecurrent")
    {
        std::cout << "default motor gain is : " << std::endl;
        for (int i = 0; i < MODEL_DOF; i++)
        {
            std::cout << dc.currentGain(i) << "\t";
        }
        std::cout << std::endl;
    }
    else if (msg->data == "fixedgravity")
    {
        std::cout << "fixed based gravity compensation mode" << std::endl;
        dc.fixedgravity = true;
    }
    else if (msg->data == "spalarm")
    {
        if (dc.spalarm)
            std::cout << "Support Polygon alarm : Off" << std::endl;
        else
            std::cout << "Support Polygon alarm : On" << std::endl;

        dc.spalarm = !dc.spalarm;
    }
    else if (msg->data == "torqueredis")
    {
        dc.signal_contactTorqueRedistribution = true;
    }
    else if (msg->data == "stateestimation")
    {
        if (dc.semode)
        {
            std::cout << "State Estimation mode off" << std::endl;
        }
        else
        {
            std::cout << "State Estimation mode on" << std::endl;
            dc.semode_init = true;
        }
        dc.semode = !dc.semode;
    }
    else if (msg->data == "qp2nd")
    {
        if (dc.tocabi_.qp2nd)
        {
            std::cout << "qp2nd mode off" << std::endl;
        }
        else
        {
            std::cout << "qp2nd mode on" << std::endl;
        }
        dc.tocabi_.qp2nd = !dc.tocabi_.qp2nd;
    }
    else if (msg->data == "ecatinit")
    {
        dc.start_initialize_sequence = true;
    }
    else if (msg->data == "ecatinitlower")
    {
        dc.start_initialize_lower = true;
    }
    else if (msg->data == "safetyreset")
    {
        dc.disableSafetyLock = true;
        dc.safetycheckdisable = false;
        dc.safetyison = false;
        std::cout << "Reset Safety mode" << std::endl;
    }
    else if (msg->data == "safetydisable")
    {
        dc.safetycheckdisable = true;
    }
    else if (msg->data == "ftcalib")
    {
        dc.ftcalib = true;
    }
    else if (msg->data == "showdata")
    {
        dc.tocabi_.showdata = true;
    }
    else if (msg->data == "terminate")
    {
        shutdown_tocabi_bool = true;
    }
    else if (msg->data == "inityaw")
    {
        dc.tocabi_.signal_yaw_init = true;
    }
    else if (msg->data == "imureset")
    {
        dc.signal_imu_reset = true;
    }
    else if (msg->data == "simvirtualjoint")
    {
        dc.use_virtual_for_mujoco = !dc.use_virtual_for_mujoco;
        if (dc.use_virtual_for_mujoco)
        {
            std::cout << "Use mujoco virtual info : On " << std::endl;
        }
        else
        {
            std::cout << "Use mujoco virtual info : Off " << std::endl;
        }
    }
    else if (msg->data == "printdata")
    {
        dc.open_file_for_print = true;
    }
    else if (msg->data == "enablelpf")
    {
        dc.enable_lpf = !dc.enable_lpf;
        if (dc.enable_lpf)
        {
            std::cout << "qdot Lowpass : On" << std::endl;
        }
        else
        {
            std::cout << "qdot lowpass : Off" << std::endl;
        }
    }
    else if (msg->data == "imuignore")
    {
        dc.imu_ignore = !dc.imu_ignore;
        if (dc.imu_ignore)
        {
            std::cout << "imu ignore : on" << std::endl;
        }
        else
        {
            std::cout << "Imu Ignore : Off" << std::endl;
        }
    }
    else if (msg->data == "sebyft")
    {
        dc.sebyft = !dc.sebyft;
        if (dc.sebyft)
        {
            std::cout << "State Estimate by FT : ON" << std::endl;
        }
        else
        {
            std::cout << "State Estimate by Ft : OFF" << std::endl;
        }
    }
    else if (msg->data == "disablelower")
    {
        dc.disableLowerBody = !dc.disableLowerBody;
        if (dc.disableLowerBody)
        {
            std::cout << "Disable LowerBody" << std::endl;
        }
        else
        {
            std::cout << "Enable LowerBody" << std::endl;
        }
    }
    else if (msg->data == "singlefootonly")
    {
        dc.single_foot_only = !dc.single_foot_only;
        if (dc.single_foot_only)
        {
            std::cout << "Single foot only disable" << std::endl;
        }
        else
        {
            std::cout << "single foot only enabled" << std::endl;
        }
    }
}
