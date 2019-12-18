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
        mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states", 1, &MujocoInterface::jointStateCallback, this, ros::TransportHints().tcpNoDelay(true));
        mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time", 1, &MujocoInterface::simTimeCallback, this, ros::TransportHints().tcpNoDelay(true));
        mujoco_sensor_state_sub_ = nh.subscribe("/mujoco_ros_interface/sensor_states", 1, &MujocoInterface::sensorStateCallback, this, ros::TransportHints().tcpNoDelay(true));
    }

    mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 1);
    mujoco_sim_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);
    mujoco_sim_command_sub_ = nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 100, &MujocoInterface::simCommandCallback, this);
    mujoco_joint_set_msg_.position.resize(MODEL_DOF);
    mujoco_joint_set_msg_.torque.resize(MODEL_DOF);
}

void MujocoInterface::updateState()
{
    while (ros::ok())
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
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}

void MujocoInterface::sendCommand(Eigen::VectorQd command, double simt)
{

    mujoco_joint_set_msg_.MODE = 1;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < MODEL_DOF; j++)
        {
            if (RED::ACTUATOR_NAME[i] == joint_name_mj[j])
            {
                mujoco_joint_set_msg_.torque[j] = command[i];
            }
        }
    }

    torque_desired = command;

    mujoco_joint_set_msg_.header.stamp = ros::Time::now();
    mujoco_joint_set_msg_.time = simt;
    mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
    mujoco_sim_last_time = mujoco_sim_time;
}

void MujocoInterface::connect()
{
    //std::cout << "________________________________________________________________________________\n\n";
    std::cout << "\tConnecting to Mujoco ..." << std::flush;

    int w_y;

    w_y = 12;
    rprint(dc, 14, 10, "Press any key to stop ");
    rprint(dc, 13, 10, "Connecting to Mujoco .... ");
    ros::Rate r(100);
    ros::Time start_time = ros::Time::now();
    int cnt = 0;
    int kbhit = -1;
    int loading = 0;
    while (!mujoco_ready & ros::ok())
    {
        kbhit = getch();
        cnt++;
        r.sleep();
        ros::spinOnce();
        if ((cnt % 10) == 0)
        {
            if (loading == 0)
                rprint(dc, 13, 31, ":    ");
            else if (loading == 1)
                rprint(dc, 13, 31, " :   ");
            else if (loading == 2)
                rprint(dc, 13, 31, "  :  ");
            else if (loading == 3)
                rprint(dc, 13, 31, "   : ");
            else if (loading == 4)
                rprint(dc, 13, 31, "    :");
            else if (loading == 5)
                rprint(dc, 13, 31, "   : ");
            else if (loading == 6)
                rprint(dc, 13, 31, "  :  ");
            else if (loading == 7)
                rprint(dc, 13, 31, " :  ");

            loading++;
            if (loading > 7)
                loading = 0;
        }
        if (!(kbhit == -1))
        {
            rprint(dc, 13, 31, "::::");
            rprint(dc, 13, 36, "Stopping");
            break;
        }
        if ((ros::Time::now().toSec() - start_time.toSec()) > 5.0)
        {
            rprint(dc, 13, 31, "::::");
            rprint(dc, 13, 36, "Stopping");
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
        //std::cout << "\tConnection failed. \n"                  << std::flush;
    }
    else if (mujoco_init_receive && mujoco_ready)
    {
        mujoco_init_receive = false;
        mujoco_ready = false;
        rprint(dc, 13, 31, "::::");
        rprint(dc, 13, 36, "Connected");
        dc.connected = true;
        //return 1;
        //std::cout << "\tConnected! \n"                  << std::flush;
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

    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < msg->name.size(); j++)
        {
            if (RED::ACTUATOR_NAME[i] == msg->name[j].data())
            {
                q_(i) = msg->position[j];
                q_virtual_(i + 6) = msg->position[j];
                q_dot_(i) = msg->velocity[j];
                q_dot_virtual_(i + 6) = msg->velocity[j];
                q_ddot_virtual_(i + 6) = msg->effort[j];
                torque_(i) = msg->effort[j];
            }
        }

        joint_name_mj[i] = msg->name[i + 6].data();
    }

    //virtual joint
    if (dc.semode)
    {
        for (int i = 3; i < 6; i++)
        {
            q_virtual_(i) = msg->position[i];
            q_dot_virtual_(i) = msg->velocity[i];
            q_ddot_virtual_(i) = msg->effort[i];
        }
        q_virtual_(MODEL_DOF + 6) = msg->position[MODEL_DOF + 6];

        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = 0.0;
            q_dot_virtual_(i) = 0.0;
            q_ddot_virtual_(i) = 0.0;
        }
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            q_virtual_(i) = msg->position[i];
            q_dot_virtual_(i) = msg->velocity[i];
            q_ddot_virtual_(i) = msg->effort[i];
        }
        q_virtual_(MODEL_DOF + 6) = msg->position[MODEL_DOF + 6];
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

void MujocoInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < msg->name.size(); j++)
        {
            if (RED::ACTUATOR_NAME[i] == msg->name[j].data())
            {
                q_(i) = msg->position[j];
                q_virtual_(i + 6) = msg->position[j];
                q_dot_(i) = msg->velocity[j];
                q_dot_virtual_(i + 6) = msg->velocity[j];
                q_ddot_virtual_(i + 6) = msg->effort[j];
                torque_(i) = msg->effort[j];
            }
        }

        joint_name_mj[i] = msg->name[i + 6].data();
    }

    //virtual joint
    if (dc.semode == false)
    {
        for (int i = 0; i < 6; i++)
        {
            q_virtual_(i) = msg->position[i];
            q_dot_virtual_(i) = msg->velocity[i];
            q_ddot_virtual_(i) = msg->effort[i];
        }
        q_virtual_(MODEL_DOF + 6) = msg->position[MODEL_DOF + 6];
    }

    data_received_counter_++;
    //tf::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(total_dof_ + 6));
    //q.normalize();
}

void MujocoInterface::sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr &msg)
{
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
                //left_hand_ft_(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LH_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //left_hand_ft_(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RH_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //right_hand_ft_(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RH_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //right_hand_ft_(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
}

void MujocoInterface::simCommandCallback(const std_msgs::StringConstPtr &msg)
{
    

    std::string buf;
    buf = msg->data;

    //ROS_INFO("CB from simulator : %s", buf.c_str());
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
        mujoco_reset = true;
    }
}