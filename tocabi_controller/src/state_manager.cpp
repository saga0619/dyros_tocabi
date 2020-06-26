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
    motor_acc_dif_info_pub = dc.nh.advertise<tocabi_controller::MotorInfo>("/tocabi/accdifinfo", 1);
    tgainPublisher = dc.nh.advertise<std_msgs::Float32>("/tocabi/torquegain", 100);
    point_pub = dc.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/point", 100);
    ft_viz_pub = dc.nh.advertise<visualization_msgs::MarkerArray>("/tocabi/ft_viz", 0);
    gui_state_pub = dc.nh.advertise<std_msgs::Int32MultiArray>("/tocabi/systemstate", 100);
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

    ROS_INFO_COND(verbose, "Loading DYROS TOCABI description from = %s", urdf_path.c_str());

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

        Eigen::Vector3d lf_c, rf_c, lh_c, rh_c;
        lf_c << 0.0317, 0, -0.154;
        rf_c << 0.0317, 0, -0.154;

        link_[Right_Foot].contact_point = rf_c;
        link_[Right_Foot].sensor_point << 0.0, 0.0, -0.1098;
        link_[Left_Foot].contact_point = lf_c;
        link_[Left_Foot].sensor_point << 0.0, 0.0, -0.1098;

        link_[Right_Hand].contact_point << 0, 0.092, 0.0;
        link_[Right_Hand].sensor_point << 0.0, 0.0, 0.0;
        link_[Left_Hand].contact_point << 0, 0.092, 0.0;
        link_[Left_Hand].sensor_point << 0.0, 0.0, 0.0;

        joint_state_msg.name.resize(MODEL_DOF);
        for (int i = 0; i < MODEL_DOF; i++)
        {
            joint_state_msg.name[i] = TOCABI::JOINT_NAME[i];
        }
        // RigidBodyDynamics::Joint J_temp;
        // J_temp=RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeEulerXYZ);
        // model_.mJoints[2] = J_temp;
        std::cout << "Total Mass : " << total_mass << std::endl; // mass without head -> 83.6 kg
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
    q_dot_virtual_before.setZero();
    std::chrono::microseconds cycletime(dc.ctime);
    int cycle_count = 0;
    if (!shutdown_tocabi_bool)
    {
        std::cout << "State thread start! " << std::endl;

        while (!shutdown_tocabi_bool)
        {
            std::this_thread::sleep_until(st_start_time + std::chrono::microseconds(250) + (cycle_count * cycletime));
            updateState();
            //imuCompenstation();

            //q_dot_virtual_ = q_dot_virtual_raw_;
            initYaw();

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
            }

            if (dc.switch_lpf)
            {
                q_dot_virtual_ = DyrosMath::lpf(q_dot_virtual_raw_, q_dot_virtual_before, 2000, 15);
                q_dot_virtual_.segment(0, 6) = q_dot_virtual_raw_.segment(0, 6);
                q_dot_virtual_before = q_dot_virtual_;
            }
            else
            {
                q_dot_virtual_ = q_dot_virtual_raw_;
            }

            if (shutdown_tocabi_bool)
            {
                std::cout << "shutdown signal received" << std::endl;
                break;
            }
            updateKinematics(model_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

            stateEstimate();

            //lowpass filter for q_dot

            updateKinematics(model_2, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

            handleFT();

            storeState();

            if ((cycle_count % 10) == 0)
            {
                if (control_time_ > 1.0)
                {
                    adv2ROS();
                }
            }
            if ((cycle_count % 200) == 0)
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
        updateKinematics(model_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);
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
        joint_state_msg.position[i] = q_[i];
        joint_state_msg.velocity[i] = q_dot_[i];
        joint_state_msg.effort[i] = dc.torque_desired[i];
    }
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

    pointpub_msg.polygon.points[3].x = dc.tocabi_.link_[Pelvis].xpos(0);
    pointpub_msg.polygon.points[3].y = dc.tocabi_.link_[Pelvis].xpos(1);
    pointpub_msg.polygon.points[3].z = dc.tocabi_.link_[Pelvis].xpos(2);

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

    //pointpub_msg.polygon.points[10].x = rtr * mod / (dc.tocabi_.torque_grav[7] + dc.tocabi_.torque_contact[7]);
    //pointpub_msg.polygon.points[10].y = dc.q_(7) - dc.q_ext_(7);
    //pointpub_msg.polygon.points[10].z = rtr * mod / dc.torque_desired[7];

    pointpub_msg.polygon.points[10].x = dc.tocabi_.link_[Pelvis].v(0);
    pointpub_msg.polygon.points[10].y = dc.tocabi_.link_[Pelvis].v(1);
    pointpub_msg.polygon.points[10].z = dc.tocabi_.link_[Pelvis].v(2);

    Matrix6d adt;

    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-dc.tocabi_.link_[Right_Foot].contact_point) * Matrix3d::Identity();

    Vector6d cf_t;
    cf_t = dc.tocabi_.ContactForce_FT.segment(6, 6);

    Matrix6d rotrf;
    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = dc.tocabi_.link_[Right_Foot].Rotm;
    rotrf.block(3, 3, 3, 3) = dc.tocabi_.link_[Right_Foot].Rotm;

    Vector6d cf_t_res;
    cf_t_res = rotrf * adt * cf_t;

    pointpub_msg.polygon.points[10].x = dc.torque_desired[7];
    pointpub_msg.polygon.points[10].y = -dc.tocabi_.ContactForce[9] / dc.tocabi_.ContactForce[8];

    Vector3d p1, p2;

    p1 = dc.tocabi_.link_[Left_Foot].xpos;//_contact;// - dc.tocabi_.link_[Pelvis].xpos;
    p2 = dc.tocabi_.link_[Right_Foot].xpos;//_contact;// - dc.tocabi_.link_[Pelvis].xpos;

    MatrixXd Rs_;

    Rs_.setZero(6, 12);

    Rs_.block(0, 0, 6, 6) = Matrix6d::Identity();
    Rs_.block(0, 6, 6, 6) = Matrix6d::Identity();
    Rs_.block(3, 0, 3, 3) = DyrosMath::skm(p1);
    Rs_.block(3, 6, 3, 3) = DyrosMath::skm(p2);

    Vector6d RsForce;

    RsForce = Rs_ * dc.tocabi_.ContactForce;

    pointpub_msg.polygon.points[10].z = RsForce(3) / RsForce(2);// + dc.tocabi_.link_[Pelvis].xpos(1);
    pointpub_msg.polygon.points[10].x = 0;
    pointpub_msg.polygon.points[10].y = dc.tocabi_.link_[COM_id].xpos(1);
    //pointpub_msg.polygon.points[10].z = dc.tocabi_.link_[COM_id].v(1);

    //pointpub_msg.polygon.points[11].x = dc.tocabi_.link_[COM_id].v_traj(2);
    //pointpub_msg.polygon.points[11].y = dc.tocabi_.link_[COM_id].v_traj(2);
    //pointpub_msg.polygon.points[11].z = dc.tocabi_.link_[COM_id].v_traj(2);

    /*

    pointpub_msg.polygon.points[8].x = dc.tocabi_.link_[COM_id].xpos(0);
    pointpub_msg.polygon.points[8].y = dc.tocabi_.link_[COM_id].xpos(1);
    pointpub_msg.polygon.points[8].z = dc.tocabi_.link_[COM_id].xpos(2);

    pointpub_msg.polygon.points[9].x = dc.tocabi_.link_[COM_id].v(2);
    pointpub_msg.polygon.points[9].y = dc.tocabi_.link_[COM_id].v(2);
    pointpub_msg.polygon.points[9].z = dc.tocabi_.link_[COM_id].v(2);*/

    //pointpub_msg.polygon.points[10].x = dc.tocabi_.link_[COM_id].x_traj(0);
    //pointpub_msg.polygon.points[10].y = dc.tocabi_.link_[COM_id].x_traj(1);
    //pointpub_msg.polygon.points[10].z = dc.tocabi_.link_[COM_id].x_traj(2);

    //pointpub_msg.polygon.points[11].x = dc.q_(7);
    //pointpub_msg.polygon.points[11].y = dc.q_ext_(7);
    //pointpub_msg.polygon.points[11].z = dc.tocabi_.imu_pos_(2);

    pointpub_msg.polygon.points[11].x = dc.tocabi_.link_[COM_id].v_traj(1);
    pointpub_msg.polygon.points[11].y = dc.tocabi_.link_[COM_id].x_traj(1);
    pointpub_msg.polygon.points[11].z = dc.tocabi_.link_[COM_id].a_traj(1);

    /*
    temp = DyrosMath::rotateWithZ(-dc.tocabi_.yaw) * link_[Left_Foot].xpos;

    pointpub_msg.polygon.points[9].x = dc.tocabi_.ZMP(0); //from task torque -> contact force -> zmp
    pointpub_msg.polygon.points[9].y = dc.tocabi_.ZMP(1);
    pointpub_msg.polygon.points[9].z = dc.tocabi_.ZMP(2);

    pointpub_msg.polygon.points[10].x = dc.tocabi_.ZMP_local(0); //from acceleration trajecoty -> tasktorque -> contactforce -> zmp
    pointpub_msg.polygon.points[10].y = dc.tocabi_.ZMP_local(1);
    pointpub_msg.polygon.points[10].z = dc.tocabi_.ZMP_local(2);

    pointpub_msg.polygon.points[11].x = dc.tocabi_.ZMP_desired(0); //from acceleration trajectory
    pointpub_msg.polygon.points[11].y = dc.tocabi_.ZMP_desired(1);
    pointpub_msg.polygon.points[11].z = dc.tocabi_.ZMP_desired(2);

*/
    temp = DyrosMath::rotateWithZ(-dc.tocabi_.yaw) * dc.tocabi_.ZMP_ft;

    pointpub_msg.polygon.points[12].x = temp(0); //calc from ft sensor
    pointpub_msg.polygon.points[12].y = temp(1);
    pointpub_msg.polygon.points[12].z = dc.tocabi_.ZMP_ft(2);

    pointpub_msg.polygon.points[13].x = rf_s_ratio;
    pointpub_msg.polygon.points[13].y = lf_s_ratio;

    if (dc.tocabi_.TaskForce.size() > 0)
    {
        pointpub_msg.polygon.points[13].z = dc.tocabi_.TaskForce(1);
    }
    else
    {
        pointpub_msg.polygon.points[13].z = 0;
    }

    pointpub_msg.polygon.points[14].x = RF_CF_FT(2) / (com_.mass * GRAVITY);
    pointpub_msg.polygon.points[14].y = LF_CF_FT(2) / (com_.mass * GRAVITY);
    pointpub_msg.polygon.points[14].z = dc.tocabi_.link_[COM_id].a_traj(2);

    //pointpub_msg.polygon.points[14].x = dc.tocabi_.ZMP_eqn_calc(0); //from zmp dynamics
    //pointpub_msg.polygon.points[14].y = dc.tocabi_.ZMP_eqn_calc(1);
    //pointpub_msg.polygon.points[14].z = dc.tocabi_.ZMP_eqn_calc(2);

    pointpub_msg.polygon.points[15].x = dc.tocabi_.com_.accel(0);
    pointpub_msg.polygon.points[15].y = dc.tocabi_.com_.accel(1);
    pointpub_msg.polygon.points[15].z = dc.tocabi_.com_.accel(2);

    dc.tocabi_.ZMP_command = dc.tocabi_.com_.pos - dc.tocabi_.com_.pos(2) / 9.81 * dc.tocabi_.link_[COM_id].a_traj;

    pointpub_msg.polygon.points[16].x = dc.tocabi_.ZMP_command(0);
    pointpub_msg.polygon.points[16].y = dc.tocabi_.ZMP_command(1);
    pointpub_msg.polygon.points[16].z = dc.tocabi_.ZMP_command(2);

    pointpub_msg.polygon.points[17].x = dc.tocabi_.ContactForce(3) / dc.tocabi_.ContactForce(2);
    pointpub_msg.polygon.points[17].y = dc.tocabi_.ContactForce_FT(3) / (dc.tocabi_.ContactForce_FT(2) - 2.36 * 9.81);
    pointpub_msg.polygon.points[17].z = pointpub_msg.polygon.points[8].x * 180 / 3.141592;
    point_pub.publish(pointpub_msg);

    for (int i = 0; i < 2; i++)
    {
        ft_viz_msg.markers[i].header.stamp = ros::Time::now();
    }

    ft_viz_msg.markers[0].points[0].x = link_[Right_Foot].xpos(0);
    ft_viz_msg.markers[0].points[0].y = link_[Right_Foot].xpos(1);
    ft_viz_msg.markers[0].points[0].z = link_[Right_Foot].xpos(2);

    ft_viz_msg.markers[0].points[1].x = RF_FT(0);
    ft_viz_msg.markers[0].points[1].y = RF_FT(1);
    ft_viz_msg.markers[0].points[1].z = RF_FT(2) / 100.0;

    ft_viz_msg.markers[1].points[0].x = link_[Left_Foot].xpos(0);
    ft_viz_msg.markers[1].points[0].y = link_[Left_Foot].xpos(1);
    ft_viz_msg.markers[1].points[0].z = link_[Left_Foot].xpos(2);

    ft_viz_msg.markers[1].points[1].x = LF_FT(0);
    ft_viz_msg.markers[1].points[1].y = LF_FT(1);
    ft_viz_msg.markers[1].points[1].z = LF_FT(2) / 100.0;

    ft_viz_pub.publish(ft_viz_msg);
}
void StateManager::initYaw()
{
    tf2::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(MODEL_DOF_VIRTUAL));
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

    q_virtual_(3) = q_mod.getX();
    q_virtual_(4) = q_mod.getY();
    q_virtual_(5) = q_mod.getZ();
    q_virtual_(MODEL_DOF_VIRTUAL) = q_mod.getW();
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

void StateManager::initialize()
{
    data_received_counter_ = 0;
    A_.setZero();
    A_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    q_.setZero();
    q_dot_.setZero();

    q_virtual_.setZero();
    q_dot_virtual_.setZero();
    q_ddot_virtual_.setZero();
    q_dot_virtual_lpf_.setZero();
    q_dot_virtual_raw_.setZero();
    q_ddot_virtual_lpf_.setZero();
    q_dot_before_.setZero();
    q_dot_virtual_before.setZero();

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

    dc.q_ = q_;
    dc.q_dot_ = q_dot_;
    dc.q_dot_virtual_ = q_dot_virtual_;
    dc.q_virtual_ = q_virtual_;
    dc.q_ddot_virtual_ = q_ddot_virtual_;
    dc.q_ext_ = q_ext_;

    dc.tau_nonlinear_ = tau_nonlinear_;

    dc.yaw = yaw;
    dc.roll = roll;
    dc.pitch = pitch;

    dc.A_ = A_;
    dc.A_inv = A_inv;

    dc.tocabi_.ContactForce_FT.segment(0, 6) = LF_FT;
    dc.tocabi_.ContactForce_FT.segment(6, 6) = RF_FT;

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

void StateManager::updateKinematics(RigidBodyDynamics::Model &model_l, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual)
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
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual, &q_dot_virtual, &q_ddot_virtual);

    A_temp_.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_, A_temp_, false);

    //Eigen::VectorXd tau_coriolis;
    //RigidBodyDynamics::NonlinearEffects(model_,q_virtual_,q_dot_virtual_,tau_coriolis);
    mtx_rbdl.unlock();
    //tf2::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(MODEL_DOF + 6));

    A_ = A_temp_;
    A_inv = A_.inverse();

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_[i].pos_Update(model_l, q_virtual_);
    }
    Eigen::Vector3d zero;
    zero.setZero();
    dc.check = true;
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_[i].Set_Jacobian(model_l, q_virtual_, zero);
    }
    dc.check = false;

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {

        link_[i].COM_Jac_Update(model_l, q_virtual_);
    }
    //COM link information update ::
    double com_mass;
    RigidBodyDynamics::Math::Vector3d com_pos;
    RigidBodyDynamics::Math::Vector3d com_vel, com_accel, com_ang_momentum;
    mtx_rbdl.lock();
    RigidBodyDynamics::Utils::CalcCenterOfMass(model_l, q_virtual_, q_dot_virtual_, &q_ddot_virtual, com_mass, com_pos, &com_vel, &com_accel, &com_ang_momentum, NULL, false);
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

    RH = link_[Right_Foot].xpos + link_[Right_Foot].Rotm * foot_ahead_pos;
    RT = link_[Right_Foot].xpos + link_[Right_Foot].Rotm * foot_back_pos;

    LH = link_[Left_Foot].xpos + link_[Left_Foot].Rotm * foot_ahead_pos;
    LT = link_[Left_Foot].xpos + link_[Left_Foot].Rotm * foot_back_pos;

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
        jacobian_com += link_[i].Jac_COM_p * link_[i].Mass;
    }

    link_[COM_id].Jac.setZero(6, MODEL_DOF + 6);

    link_[COM_id].Jac.block(0, 0, 3, MODEL_DOF + 6) = jacobian_com.block(0, 0, 3, MODEL_DOF + 6) / com_.mass;
    link_[COM_id].Jac.block(3, 0, 3, MODEL_DOF + 6) = link_[Pelvis].Jac.block(3, 0, 3, MODEL_DOF + 6);

    link_[COM_id].Jac_COM_p = jacobian_com / com_.mass;

    link_[COM_id].xpos = com_.pos;
    link_[COM_id].xpos(2) = link_[Pelvis].xpos(2);
    link_[COM_id].Rotm = link_[Pelvis].Rotm;

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        link_[i].vw_Update(q_dot_virtual_);
    }

    RigidBodyDynamics::Math::VectorNd tau_;
    tau_.resize(model_l.qdot_size);
    RigidBodyDynamics::NonlinearEffects(model_l, q_virtual_, q_dot_virtual_, tau_);
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
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-link_[Right_Foot].contact_point) * Matrix3d::Identity();
    Matrix6d rotrf;
    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = link_[Right_Foot].Rotm;
    rotrf.block(3, 3, 3, 3) = link_[Right_Foot].Rotm;
    Vector3d RF_com(-0.0162, 0.00008, -0.1209);

    Vector3d com2cp = link_[Right_Foot].contact_point - RF_com;

    Matrix6d adt2;
    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();

    Vector6d Wrench_foot_plate;
    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = -foot_plate_mass * GRAVITY;

    RF_CF_FT = rotrf * adt * RF_FT + adt2 * Wrench_foot_plate;

    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-link_[Left_Foot].contact_point) * Matrix3d::Identity();

    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = link_[Left_Foot].Rotm;
    rotrf.block(3, 3, 3, 3) = link_[Left_Foot].Rotm;

    Vector3d LF_com(-0.0162, -0.00008, -0.1209);

    com2cp = link_[Left_Foot].contact_point - LF_com;

    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();
    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = -foot_plate_mass * GRAVITY;

    LF_CF_FT = rotrf * adt * LF_FT + adt2 * Wrench_foot_plate;
}

void StateManager::contactEstimate()
{
    //See zmp for each foot by ft sensor.

    //with ft, contact force Z means,
    //Oh my fucking Goddddddd
}

void StateManager::stateEstimate()
{
    if (dc.semode && (!dc.tocabi_.signal_yaw_init))
    {
        static bool contact_right, contact_left;
        static Eigen::Vector3d rf_cp, lf_cp;

        static Eigen::Vector3d mod_base_pos;
        static Eigen::Vector3d mod_base_vel;

        static Eigen::Vector3d rf_cp_m, lf_cp_m;

        if (contact_right != dc.tocabi_.ee_[1].contact)
        {
            if (dc.tocabi_.ee_[1].contact)
            {
                std::cout << "right foot contact point initialized" << std::endl;
                rf_cp = dc.link_[Right_Foot].xpos;
                lf_cp = dc.link_[Left_Foot].xpos;
            }
            else
            {
                std::cout << "right foot contact disabled" << std::endl;
            }
        }
        if (contact_left != dc.tocabi_.ee_[0].contact)
        {
            if (dc.tocabi_.ee_[0].contact)
            {
                std::cout << "left foot contact point initialized" << std::endl;
                rf_cp = dc.link_[Right_Foot].xpos;
                lf_cp = dc.link_[Left_Foot].xpos;
            }
            else
            {
                std::cout << "left foot contact disabled" << std::endl;
            }
        }

        if (dc.semode_init)
        {
            rf_cp(2) = 0.0 - dc.link_[Right_Foot].contact_point(2);
            lf_cp(2) = 0.0 - dc.link_[Left_Foot].contact_point(2);
            dc.semode_init = false;

            imu_lin_acc_before = imu_lin_acc;
        }

        imu_lin_acc_lpf = DyrosMath::lpf(imu_lin_acc, imu_lin_acc_before, 2000, 20);
        imu_lin_acc_before = imu_lin_acc_lpf;
        pelv_lin_acc = dc.link_[Pelvis].Rotm.inverse() * imu_lin_acc_lpf;

        double dt_i = 1.0 / 2000.0;

        Vector3d temp;

        temp = dc.tocabi_.imu_vel_ + dt_i * pelv_lin_acc;

        dc.tocabi_.imu_vel_ = temp;

        temp = dc.tocabi_.imu_pos_ + (dt_i * dt_i / 0.5) * pelv_lin_acc + dc.tocabi_.imu_vel_ * dt_i;

        dc.tocabi_.imu_pos_ = temp;

        //Vector3d es_zmp;

        //es_zmp = dc.tocabi_.com_.pos - dc.tocabi_.com_.pos(2)/9.81*

        contact_right = dc.tocabi_.ee_[1].contact;
        contact_left = dc.tocabi_.ee_[0].contact;

        rf_cp_m = link_[Right_Foot].xpos - rf_cp;
        lf_cp_m = link_[Left_Foot].xpos - lf_cp;

        double dr, dl;

        Vector2d zmpc = dc.com_.pos.segment(0, 2) - dc.com_.pos(2) / 9.81 * dc.tocabi_.link_[COM_id].a_traj.segment(0, 2);

        dr = (zmpc - dc.link_[Right_Foot].xpos.segment(0, 2)).norm();
        dl = (zmpc - dc.link_[Left_Foot].xpos.segment(0, 2)).norm();

        rf_s_ratio = dl / (dr + dl);
        lf_s_ratio = dr / (dr + dl);

        //std::cout << " dr : " << dr << "  dl : " << dl << "  rf_s_ratio : " << rf_s_ratio << "  lf_s_ratio : " << lf_s_ratio << std::endl;
        if (contact_right && contact_left)
        {
            link_[Right_Foot].Set_Contact(q_virtual_, q_dot_virtual_, link_[Right_Foot].contact_point);
            link_[Left_Foot].Set_Contact(q_virtual_, q_dot_virtual_, link_[Left_Foot].contact_point);
            //std::cout << control_time_ << " : base pos calc ! " << std::endl;
            mod_base_pos = (rf_cp_m * rf_s_ratio / (rf_s_ratio + lf_s_ratio) + lf_cp_m * lf_s_ratio / (rf_s_ratio + lf_s_ratio));
            //mod_base_pos(2) = mod_base_pos(2) + ((link_[Right_Foot].xpos(2) + link_[Right_Foot].contact_point(2)) * rf_s_ratio/ (rf_s_ratio + lf_s_ratio) + (link_[Left_Foot].xpos(2) + link_[Left_Foot].contact_point(2)) * lf_s_ratio / (rf_s_ratio + lf_s_ratio));
            mod_base_vel = link_[Right_Foot].v_contact * rf_s_ratio / (rf_s_ratio + lf_s_ratio) + link_[Left_Foot].v_contact * lf_s_ratio / (rf_s_ratio + lf_s_ratio);
        }
        else if (contact_right)
        {
            link_[Right_Foot].Set_Contact(q_virtual_, q_dot_virtual_, link_[Right_Foot].contact_point);
            mod_base_pos = rf_cp_m;
            mod_base_vel = link_[Right_Foot].v_contact;
            //mod_base_pos(2) = mod_base_pos(2) + link_[Right_Foot].xpos(2) + link_[Right_Foot].contact_point(2);
        }
        else if (contact_left)
        {
            link_[Left_Foot].Set_Contact(q_virtual_, q_dot_virtual_, link_[Left_Foot].contact_point);
            mod_base_pos = lf_cp_m;
            mod_base_vel = link_[Left_Foot].v_contact;
            //mod_base_pos(2) = mod_base_pos(2) + link_[Left_Foot].xpos(2) + link_[Left_Foot].contact_point(2);
        }

        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = -mod_base_pos(i);
            q_dot_virtual_(i) = -mod_base_vel(i);
        }

        q_ddot_virtual_ = (q_dot_virtual_ - q_dot_virtual_before) / ((double)dc.ctime / 1000000.0);
        q_dot_virtual_before = q_dot_virtual_;
    }
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
        dc.use_virtual_for_mujoco = true;
    }
    else if (msg->data == "printdata")
    {
        dc.open_file_for_print = true;
    }
    else if (msg->data == "enablelpf")
    {
        dc.enable_lpf = true;
    }
}