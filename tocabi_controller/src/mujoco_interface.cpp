#include "tocabi_controller/mujoco_interface.h"

MujocoInterface::MujocoInterface(DataContainer &dc_global) : dc(dc_global), StateManager(dc_global)
{
    ros::NodeHandle nh = dc.nh;
    if (dc.pubmode)
    {
        mujoco_sim_status_sub_ = nh.subscribe("/mujoco_ros_interface/sim_status", 1, &MujocoInterface::simStatusCallback, this, ros::TransportHints().tcpNoDelay(true));
    }
    else
    {
        //mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states", 1, &MujocoInterface::jointStateCallback, this, ros::TransportHints().tcpNoDelay(true));
        mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time", 1, &MujocoInterface::simTimeCallback, this, ros::TransportHints().tcpNoDelay(true));
        //mujoco_sensor_state_sub_ = nh.subscribe("/mujoco_ros_interface/sensor_states", 1, &MujocoInterface::sensorStateCallback, this, ros::TransportHints().tcpNoDelay(true));
    }

    mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 100);
    mujoco_sim_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);
    mujoco_sim_command_sub_ = nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 100, &MujocoInterface::simCommandCallback, this);
    mujoco_joint_set_msg_.position.resize(MODEL_DOF);
    mujoco_joint_set_msg_.torque.resize(MODEL_DOF);
}

void MujocoInterface::updateState()
{
    while (ros::ok() && (!shutdown_tocabi_bool))
    {
        ros::spinOnce();
        if (new_state_trigger)
        {
            new_state_trigger = false;
            break;
        }
        if (dc.mode == "testmode" || dc.mode == "testmode2")
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void MujocoInterface::sendCommand(Eigen::VectorQd command, double simt, int control_mode)
{
    if (control_mode == Torquemode)
    {
        if (dc.sim_mode == "torque")
        {
            mujoco_joint_set_msg_.MODE = 1;

            for (int i = 0; i < MODEL_DOF; i++)
            {
                for (int j = 0; j < MODEL_DOF; j++)
                {
                    if (TOCABI::ACTUATOR_NAME[i] == joint_name_mj[j])
                    {
                        mujoco_joint_set_msg_.torque[j] = command[i];
                    }
                }
            }
        }
        else
        {
            std::cout << "COMMAND DISMATCH! -- mujoco mode : positon, controller mode : torque" << std::endl;
        }
    }
    else if (control_mode == Positionmode)
    {
        if (dc.sim_mode == "position")
        {
            mujoco_joint_set_msg_.MODE = 0;

            for (int i = 0; i < MODEL_DOF; i++)
            {
                for (int j = 0; j < MODEL_DOF; j++)
                {
                    if (TOCABI::ACTUATOR_NAME[i] == joint_name_mj[j])
                    {
                        mujoco_joint_set_msg_.position[j] = command[i];
                    }
                }
            }
        }
        else
        {
            std::cout << "COMMAND DISMATCH! -- mujoco mode : torque, controller mode : position" << std::endl;
        }
    }

    mujoco_joint_set_msg_.header.stamp = ros::Time::now();
    mujoco_joint_set_msg_.time = simt;
    mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
    mujoco_sim_last_time = mujoco_sim_time;
}

void MujocoInterface::connect()
{
    //std::cout << "________________________________________________________________________________\n\n";
    std::cout << "\tConnecting to Mujoco ..." << std::flush;

    printf("Press any key to stop \n");
    printf("Connecting to Mujoco .... \n");
    ros::Rate r(100);
    ros::Time start_time = ros::Time::now();
    int cnt = 0;
    int loading = 0;
    while (!mujoco_ready && ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        if ((ros::Time::now().toSec() - start_time.toSec()) > 5.0)
        {
            printf("No response from Mujoco for 5 seconds ... Stop Connecting Mujoco\n");
            break;
        }
    }

    start_time = ros::Time::now();

    if (mujoco_ready)
    {
        while (!mujoco_init_receive & ros::ok())
        {
            r.sleep();
            ros::spinOnce();
            if ((ros::Time::now().toSec() - start_time.toSec()) > 1.0)
                break;
        }
    }

    if ((!mujoco_init_receive) && (!mujoco_ready))
    {
        mujoco_init_receive = false;
        mujoco_ready = false;
    }
    else if (mujoco_init_receive && mujoco_ready)
    {
        mujoco_init_receive = false;
        mujoco_ready = false;
        printf("Connected!\n");
        dc.connected = true;
    }
}

void MujocoInterface::playMujoco()
{
    std_msgs::String rst_msg_;
    rst_msg_.data = "PAUSE";
    mujoco_sim_command_pub_.publish(rst_msg_);
}

void MujocoInterface::simTimeCallback(const std_msgs::Float32ConstPtr &msg)
{
    mujoco_sim_time = msg->data;
    control_time_ = mujoco_sim_time;
    sim_time_ = mujoco_sim_time;
}

void MujocoInterface::simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg)
{
    mujoco_sim_time = msg->time;
    control_time_ = mujoco_sim_time;
    sim_time_ = mujoco_sim_time;

    static bool first = true;

    if (first)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            for (int j = 0; j < msg->name.size(); j++)
            {
                if (TOCABI::ACTUATOR_NAME[i] == msg->name[j].data())
                {
                    jointmap[i] = j;
                }
            }
        }
        first = false;
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        q_(i) = msg->position[jointmap[i]];
        q_virtual_local_(i + 6) = msg->position[jointmap[i]];
        q_dot_virtual_local_(i + 6) = msg->velocity[jointmap[i]];
        q_ddot_virtual_local_(i + 6) = msg->effort[jointmap[i]];
        torque_(i) = msg->effort[jointmap[i]];
        joint_name_mj[i] = msg->name[i + 6].data();
    }

    Real_Pos(0) = msg->position[0];
    Real_Pos(1) = msg->position[1];
    Real_Pos(2) = msg->position[2];

    Real_Vel(0) = msg->velocity[0];
    Real_Vel(1) = msg->velocity[1];
    Real_Vel(2) = msg->velocity[2];

    //virtual joint
    if (dc.semode)
    {
        for (int i = 3; i < 6; i++)
        {
            q_virtual_local_(i) = msg->position[i];
            q_dot_virtual_local_(i) = msg->velocity[i];
            q_ddot_virtual_local_(i) = msg->effort[i];
        }
        q_virtual_local_(MODEL_DOF + 6) = msg->position[MODEL_DOF + 6];

        for (int i = 0; i < 3; i++)
        {
            q_virtual_local_(i) = 0.0;
            q_dot_virtual_local_(i) = 0.0;
            q_ddot_virtual_local_(i) = 0.0;
        }

        //TEMP
        /*     q_virtual_(3) = 0.0;
        q_virtual_(4) = 0.0;
        q_virtual_(5) = 0.0;
        q_virtual_(MODEL_DOF + 6) = 1.0;*/
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            q_virtual_local_(i) = msg->position[i];
            q_dot_virtual_local_(i) = msg->velocity[i];
            q_ddot_virtual_local_(i) = msg->effort[i];
        }
        q_virtual_local_(MODEL_DOF + 6) = msg->position[MODEL_DOF + 6];
        q_virtual_local_(0) = 0.0;
        q_virtual_local_(1) = 0.0;
        q_virtual_local_(2) = 0.0;

        if (dc.use_virtual_for_mujoco)
        {
            for (int i = 0; i < 3; i++)
                q_virtual_local_(i) = msg->position[i];
        }
    }

    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "Acc_Pelvis_IMU")
        {
            for (int j = 0; j < 3; j++)
            {
                imu_lin_acc(j) = msg->sensor[i].data[j];
            }
        }
    }

    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "Gyro_Pelvis_IMU")
        {
            for (int j = 0; j < 3; j++)
            {
                //q_dot_virtual_(j+3)=msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RF_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                RF_FT(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RF_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                RF_FT(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LF_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                LF_FT(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LF_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                LF_FT(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LH_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                LH_FT(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LH_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                LH_FT(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RH_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                RH_FT(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RH_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                RH_FT(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    data_received_counter_++;
    new_state_trigger = true;
}

void MujocoInterface::simCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        //parameterInitialize();
        mujoco_sim_last_time = 0.0;

        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        mujoco_sim_time = 0.0;
        control_time_ = 0.0;
        dc.semode_init = true;
        mujoco_reset = true;
    }

    if (buf == "terminate")
    {
        shutdown_tocabi_bool = true;
    }
}