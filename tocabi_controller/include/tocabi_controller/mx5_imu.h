
#include "tocabi_controller/data_container.h"
#include "tocabi_controller/terminal.h"
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "mscl/mscl.h"

class MX5IMU
{
public:
    MX5IMU(DataContainer &dc_global, mscl::InertialNode &node);
    DataContainer &dc;
    mscl::InertialNode &node;

    ros::Publisher imu_pub;
    sensor_msgs::Imu imu_pub_msg;

    mscl::InertialNode *np;

    unsigned short int ef_state, ef_state_flag, ef_state_before, ef_state_flag_before;

    bool init_end = false;
    bool rst_pub_once = true;
    bool rst_pub_once2 = true;
    tf2_ros::TransformBroadcaster br;

    void initIMU();
    void resetEFIMU();

    void startIMU();

    sensor_msgs::Imu getIMU();

    void endIMU();

    void setCurrentConfig(mscl::InertialNode &node);

    void getCurrentConfig(mscl::InertialNode &node);

    void startSampling(mscl::InertialNode &node);

    void parseData(mscl::InertialNode &node);

    void parseData_custum(mscl::InertialNode &node);
};