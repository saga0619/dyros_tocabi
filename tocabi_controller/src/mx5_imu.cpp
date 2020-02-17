#include "tocabi_controller/mx5_imu.h"

using namespace std;

MX5IMU::MX5IMU(DataContainer &dc_global, mscl::InertialNode &node_gl) : dc(dc_global), node(node_gl)
{
    ef_state = 55;
    ef_state_flag = 55;
    ef_state_before = 55;
    ef_state_flag_before = 55;
}

void MX5IMU::initIMU()
{
    imu_pub = dc.nh.advertise<sensor_msgs::Imu>("/tocabi/imu", 1);

    cout << "IMU : connection success " << std::endl;
    /*
    cout << "Node Info : " << endl;
    cout << "Model Name : " << node.modelName() << endl;
    cout << "Model number : " << node.modelNumber() << endl;
    cout << "Serial : " << node.serialNumber() << endl;
    cout << "Firmware : " << node.firmwareVersion().str() << endl
    << endl;*/
    node.setAutoInitialization(false);

    //cout << "Starting Sampling ... " << endl;
    startSampling(node);

    //cout << "Sending Reset Command ... " << endl;
    node.resetFilter();
    node.setInitialHeading(0.0);

    //getCurrentConfig(node);
    //node.setAutoInitialization(false);
    //node.setInitialHeading(0.0);
    //node.setAutoInitialization(true);
}

sensor_msgs::Imu MX5IMU::getIMU()
{
    mscl::MipDataPackets packets = node.getDataPackets(1);

    for (mscl::MipDataPacket packet : packets)
    {
        //print out the data
        //cout << "Packet Received: " <<packet.collectedTimestamp().str()<<packets.size()<< endl;

        //get the data in the packet
        mscl::MipDataPoints data = packet.data();
        mscl::MipDataPoint dataPoint;

        //loop through all the data points in the packet
        for (unsigned int itr = 0; itr < data.size(); itr++)
        {
            mscl::Timestamp time_data = packet.collectedTimestamp();

            //time_data.seconds();

            static int seconds;
            static int count;
            if (seconds != time_data.seconds())
            {
                count = 0;
            }
            seconds = time_data.seconds();

            dataPoint = data[itr];
            if (dataPoint.channelName() == "estFilterState")
            {
                ef_state = dataPoint.as_int16();
            }

            if (dataPoint.channelName() == "estFilterStatusFlags")
            {
                ef_state_flag = dataPoint.as_int16();
                /*
                if (dataPoint.as_int32())
                {
                    if (rst_pub_once)
                    {
                        cout << "############ resetting EF filter ############" << endl;
                        rst_pub_once = false;
                    }
                }
                else if (dataPoint.as_int16() == 0)
                {
                    if (!rst_pub_once)
                    {
                        if (rst_pub_once2)
                        {
                            cout << "############ EF filter reset complete ############" << endl;
                            rst_pub_once2 = false;
                        }
                    }
                }
                if (!rst_pub_once2)
                {
                    if (dataPoint.as_int16() != 0)
                    {
                        //ROS_INFO("WARNING ::::: estFilterStatusFlags : %d", dataPoint.as_int16());
                    }
                }*/
            }

            if (ef_state >= 2)
            {
                if (dataPoint.channelName() == "estOrientQuaternion")
                {
                    count++;
                    tf2::Quaternion q(dataPoint.as_Vector().as_floatAt(1), dataPoint.as_Vector().as_floatAt(2), dataPoint.as_Vector().as_floatAt(3), dataPoint.as_Vector().as_floatAt(0));
                    tf2::Quaternion q_rot, q_rot2;

                    q_rot.setRPY(M_PI, 0, M_PI / 2);

                    q_rot2.setRPY(M_PI, M_PI, M_PI / 2);
                    q = q_rot * q * q_rot2;

                    imu_pub_msg.orientation = tf2::toMsg(q);
                    imu_pub_msg.header.stamp = ros::Time::now();
                    imu_pub_msg.header.frame_id = "imu";
                }

                if (dataPoint.channelName() == "estLinearAccelX")
                {
                    imu_pub_msg.linear_acceleration.y = dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estLinearAccelY")
                {
                    imu_pub_msg.linear_acceleration.x = -dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estLinearAccelZ")
                {
                    imu_pub_msg.linear_acceleration.z = dataPoint.as_float();
                }

                if (dataPoint.channelName() == "estAngularRateX")
                {
                    imu_pub_msg.angular_velocity.y = dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estAngularRateY")
                {
                    imu_pub_msg.angular_velocity.x = -dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estAngularRateZ")
                {
                    imu_pub_msg.angular_velocity.z = dataPoint.as_float();
                }
            }
            //cout << dataPoint.channelName() << ": ";

            //print out the channel data
            //Note: The as_string() function is being used here for simplicity.
            //      Other methods (ie. as_float, as_uint16, as_Vector) are also available.
            //      To determine the format that a dataPoint is stored in, use dataPoint.storedAs().

            //if the dataPoint is invalid
            if (!dataPoint.valid())
            {
                //cout << "IMU : Invalid dataPoint received. reconnect IMU! ";
            }
        }
    }

    if ((ef_state_before != ef_state) || (ef_state_before != ef_state))
    {

        if (ef_state == 0)
        {
            std::cout << "IMU : start up" << std::endl;
        }
        else if (ef_state == 1)
        {
            if (ef_state_flag == 4096)
            {
                std::cout << cyellow << "IMU : initialization, Attitude not initialized" << creset << std::endl;
            }
            else if (ef_state_flag == 8192)
            {
                std::cout << cyellow << "IMU : initialization, Position & Velocity not initialized " << creset << std::endl;
            }
            else
            {
                std::cout << cyellow << "IMU : initialization, flags : " << std::hex << ef_state_flag << std::dec << creset << std::endl;
            }
        }
        else if (ef_state == 2)
        {
            std::cout << cgreen << "IMU : running, solution valid" << creset << std::endl;
            pub_to_gui(dc, "imuvalid");
        }
        else if (ef_state == 3)
        {
            std::cout << cyellow << "IMU : running, solution error" << std::hex << ef_state_flag << std::dec << creset << std::endl;
            pub_to_gui(dc, "imunotvalid");
        }
        else
        {
            std::cout << cred << "IMU : unknown ef state : " << std::hex << ef_state << std::dec << creset << std::endl;
        }
    }

    ef_state_flag_before = ef_state_flag;
    ef_state_before = ef_state;

    imu_pub.publish(imu_pub_msg);
    return imu_pub_msg;
}

void MX5IMU::startIMU()
{
}

void MX5IMU::endIMU()
{
    cout << "IMU : Setting IMU to Idle " << std::endl;
    node.setToIdle();
}

void MX5IMU::setCurrentConfig(mscl::InertialNode &node)
{ //many other settings are available than shown below
    //reference the documentation for the full list of commands

    //if the node supports AHRS/IMU
    if (node.features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        mscl::MipChannels ahrsImuChs;
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl::SampleRate::Hertz(500)));
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl::SampleRate::Hertz(100)));

        //apply to the node
        node.setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);
    }

    //if the node supports Estimation Filter
    if (node.features().supportsCategory(mscl::MipTypes::CLASS_ESTFILTER))
    {
        mscl::MipChannels estFilterChs;
        estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_GYRO_BIAS, mscl::SampleRate::Hertz(100)));

        //apply to the node
        node.setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);
    }

    //if the node supports GNSS
    if (node.features().supportsCategory(mscl::MipTypes::CLASS_GNSS))
    {
        mscl::MipChannels gnssChs;
        gnssChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_GNSS_LLH_POSITION, mscl::SampleRate::Hertz(1)));

        //apply to the node
        node.setActiveChannelFields(mscl::MipTypes::CLASS_GNSS, gnssChs);
    }

    node.setPitchRollAid(true);

    node.setAltitudeAid(false);

    mscl::PositionOffset offset(0.0f, 0.0f, 0.0f);
    node.setAntennaOffset(offset);
}
void MX5IMU::getCurrentConfig(mscl::InertialNode &node)
{
    //many other settings are available than shown below
    //reference the documentation for the full list of commands

    //if the node supports AHRS/IMU
    if (node.features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        //get a list of the AHRS/IMU channels currently active on the Node
        mscl::MipChannels ahrsImuActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU);

        cout << "AHRS/IMU Channels" << endl;
        cout << "-----------------" << endl;
        for (mscl::MipChannel ch : ahrsImuActiveChs)
        {
            cout << "Channel Field: " << std::hex << ch.channelField() << endl;
            cout << "Sample Rate: " << ch.sampleRate().prettyStr() << endl
                 << endl;
        }
    }

    //if the node supports Estimation Filter
    if (node.features().supportsCategory(mscl::MipTypes::CLASS_ESTFILTER))
    {
        //get a list of the Estimation Filter channels currently active on the Node
        mscl::MipChannels estFilterActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER);

        cout << endl;
        cout << "Estimation Filter Channels" << endl;
        cout << "--------------------------" << endl;
        for (mscl::MipChannel ch : estFilterActiveChs)
        {
            cout << "Channel Field: " << std::hex << ch.channelField() << endl;
            cout << "Sample Rate: " << ch.sampleRate().prettyStr() << endl
                 << endl;
        }
    }

    //if the node supports GNSS
    if (node.features().supportsCategory(mscl::MipTypes::CLASS_GNSS))
    {
        //get a list of the GNSS channels currently active on the Node
        mscl::MipChannels gnssActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_GNSS);

        cout << endl;
        cout << "GNSS Channels" << endl;
        cout << "-------------" << endl;
        for (mscl::MipChannel ch : gnssActiveChs)
        {
            cout << "Channel Field: " << std::hex << ch.channelField() << endl;
            cout << "Sample Rate: " << ch.sampleRate().prettyStr() << endl
                 << endl;
        }
    }
    //cout << "Altitude Aiding enabled?: " << node.getAltitudeAid() << endl;

    //mscl::PositionOffset offset = node.getAntennaOffset();
    //cout << "Antenna Offset: x=" << offset.x() << " y=" << offset.y() << " z=" << offset.z() << endl;

    cout << "Pitch/Roll Aiding enabled?: " << node.getPitchRollAid() << endl;
}

void MX5IMU::startSampling(mscl::InertialNode &node)
{ //each class/category is separated into its own command.
    //you can enable them individually if, say, you only wanted the Estimation Filter channels to be streamed

    if (node.features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        node.enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);
    }

    if (node.features().supportsCategory(mscl::MipTypes::CLASS_ESTFILTER))
    {
        node.enableDataStream(mscl::MipTypes::CLASS_ESTFILTER);
    }

    if (node.features().supportsCategory(mscl::MipTypes::CLASS_GNSS))
    {
        node.enableDataStream(mscl::MipTypes::CLASS_GNSS);
    }
}

void MX5IMU::parseData(mscl::InertialNode &node)
{ //endless loop of reading in data
    while (ros::ok())
    {
        //get all the data packets from the node, with a timeout of 500 milliseconds
        mscl::MipDataPackets packets = node.getDataPackets(500);

        for (mscl::MipDataPacket packet : packets)
        {
            //print out the data
            cout << "Packet Received: ";

            //get the data in the packet
            mscl::MipDataPoints data = packet.data();
            mscl::MipDataPoint dataPoint;

            //loop through all the data points in the packet
            for (unsigned int itr = 0; itr < data.size(); itr++)
            {
                dataPoint = data[itr];
                cout << " ######## " << endl;
                cout << dataPoint.channelName() << ": ";

                //print out the channel data
                //Note: The as_string() function is being used here for simplicity.
                //      Other methods (ie. as_float, as_uint16, as_Vector) are also available.
                //      To determine the format that a dataPoint is stored in, use dataPoint.storedAs().
                cout << dataPoint.as_string() << " ";

                //if the dataPoint is invalid
                if (!dataPoint.valid())
                {
                    cout << "[Invalid] ";
                }
            }
            cout << endl;
        }
    }
}

void MX5IMU::parseData_custum(mscl::InertialNode &node)
{

    //get all the data packets from the node, with a timeout of 1 milliseconds
    mscl::MipDataPackets packets = node.getDataPackets(1);

    for (mscl::MipDataPacket packet : packets)
    {
        //print out the data
        //cout << "Packet Received: " <<packet.collectedTimestamp().str()<<packets.size()<< endl;

        //get the data in the packet
        mscl::MipDataPoints data = packet.data();
        mscl::MipDataPoint dataPoint;

        //loop through all the data points in the packet
        for (unsigned int itr = 0; itr < data.size(); itr++)
        {
            mscl::Timestamp time_data = packet.collectedTimestamp();

            //time_data.seconds();

            static int seconds;
            static int count;
            if (seconds != time_data.seconds())
            {
                count = 0;
            }
            seconds = time_data.seconds();

            dataPoint = data[itr];
            if (dataPoint.channelName() == "estFilterStatusFlags")
            {
                if (dataPoint.as_int16() == 4096)
                {
                    if (rst_pub_once)
                    {
                        cout << "############ resetting EF filter ############" << endl;
                        rst_pub_once = false;
                    }
                }
                else if (dataPoint.as_int16() == 0)
                {
                    if (!rst_pub_once)
                    {
                        if (rst_pub_once2)
                        {
                            cout << "############ EF filter reset complete ############" << endl;
                            rst_pub_once2 = false;
                        }
                    }
                }

                if (!rst_pub_once2)
                {
                    if (dataPoint.as_int16() != 0)
                    {
                        ROS_INFO("WARNING ::::: estFilterStatusFlags : %d", dataPoint.as_int16());
                    }
                }
            }

            if (!rst_pub_once2)
            {
                if (dataPoint.channelName() == "estOrientQuaternion")
                {
                    count++;
                    tf2::Quaternion q(dataPoint.as_Vector().as_floatAt(1), dataPoint.as_Vector().as_floatAt(2), dataPoint.as_Vector().as_floatAt(3), dataPoint.as_Vector().as_floatAt(0));
                    tf2::Quaternion q_rot, q_rot2;
                    tf2::Transform transform;

                    q_rot.setRPY(M_PI, 0, M_PI / 2);

                    q_rot2.setRPY(0, M_PI, 0);
                    q = q_rot * q * q_rot2;

                    transform.setRotation(q);

                    static tf2_ros::TransformBroadcaster br;
                    geometry_msgs::TransformStamped transformStamped;

                    transformStamped.header.stamp = ros::Time::now();
                    transformStamped.header.frame_id = "world";
                    transformStamped.child_frame_id = "Pelvis_Link";
                    transformStamped.transform.rotation.x = q.x();
                    transformStamped.transform.rotation.y = q.y();
                    transformStamped.transform.rotation.z = q.z();
                    transformStamped.transform.rotation.w = q.w();
                    br.sendTransform(transformStamped);

                    imu_pub_msg.orientation = tf2::toMsg(q);
                    imu_pub_msg.header.stamp = ros::Time::now();
                    imu_pub_msg.header.frame_id = "imu";
                }

                if (dataPoint.channelName() == "estLinearAccelX")
                {
                    imu_pub_msg.linear_acceleration.x = dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estLinearAccelY")
                {
                    imu_pub_msg.linear_acceleration.y = dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estLinearAccelZ")
                {
                    imu_pub_msg.linear_acceleration.z = dataPoint.as_float();
                }

                if (dataPoint.channelName() == "estAngularRateX")
                {
                    imu_pub_msg.angular_velocity.x = dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estAngularRateY")
                {
                    imu_pub_msg.angular_velocity.y = dataPoint.as_float();
                }
                if (dataPoint.channelName() == "estAngularRateZ")
                {
                    imu_pub_msg.angular_velocity.z = dataPoint.as_float();
                }
            }
            //cout << dataPoint.channelName() << ": ";

            //print out the channel data
            //Note: The as_string() function is being used here for simplicity.
            //      Other methods (ie. as_float, as_uint16, as_Vector) are also available.
            //      To determine the format that a dataPoint is stored in, use dataPoint.storedAs().

            //if the dataPoint is invalid
            if (!dataPoint.valid())
            {
                //cout << "[Invalid] ";
            }
        }
        //cout << endl;
    }

    imu_pub.publish(imu_pub_msg);
}