#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <chrono>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <pthread.h>
#include <mutex>
#include <future>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <mujoco_ros_msgs/SensorState.h>
#include <mujoco_ros_msgs/JointSet.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "tocabi_controller/link.h"
#include <fstream>
#include <atomic>

#include <stdlib.h>
#include <signal.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
//robot definition variables here

namespace TOCABI
{
const std::string JOINT_NAME[MODEL_DOF] = {
    "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
    "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint",
    "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
    "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint",
    "Waist1_Joint", "Waist2_Joint", "Upperbody_Joint",
    "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
    "L_Elbow_Joint", "L_Forearm_Joint", "L_Wrist1_Joint", "L_Wrist2_Joint",
    "Neck_Joint", "Head_Joint",
    "R_Shoulder1_Joint", "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Armlink_Joint",
    "R_Elbow_Joint", "R_Forearm_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint"};

const std::string ELMO_NAME[MODEL_DOF] = {
    "Head_Joint", "Neck_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint", "L_Wrist2_Joint", "L_Wrist1_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
    "R_Armlink_Joint", "R_Shoulder3_Joint", "R_Elbow_Joint", "R_Forearm_Joint", "L_Forearm_Joint", "L_Elbow_Joint", "L_Shoulder1_Joint", "L_Shoulder2_Joint",
    "R_Shoulder2_Joint", "R_Shoulder1_Joint", "Upperbody_Joint", "Waist2_Joint", "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint",
    "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "Waist1_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint",
    "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"};

enum
{
  Head_Joint,
  Neck_Joint,
  R_Wrist1_Joint,
  R_Wrist2_Joint,
  L_Wrist2_Joint,
  L_Wrist1_Joint,
  L_Shoulder3_Joint,
  L_Armlink_Joint,
  R_Armlink_Joint,
  R_Shoulder3_Joint,
  R_Elbow_Joint,
  R_Forearm_Joint,
  L_Forearm_Joint,
  L_Elbow_Joint,
  L_Shoulder1_Joint,
  L_Shoulder2_Joint,
  R_Shoulder2_Joint,
  R_Shoulder1_Joint,
  Upperbody_Joint,
  Waist2_Joint,
  R_HipYaw_Joint,
  R_HipRoll_Joint,
  R_HipPitch_Joint,
  R_Knee_Joint,
  R_AnklePitch_Joint,
  R_AnkleRoll_Joint,
  Waist1_Joint,
  L_HipYaw_Joint,
  L_HipRoll_Joint,
  L_HipPitch_Joint,
  L_Knee_Joint,
  L_AnklePitch_Joint,
  L_AnkleRoll_Joint
};

const std::string ACTUATOR_NAME[MODEL_DOF] = {
    "L_HipYaw_Motor", "L_HipRoll_Motor", "L_HipPitch_Motor",
    "L_Knee_Motor", "L_AnklePitch_Motor", "L_AnkleRoll_Motor",
    "R_HipYaw_Motor", "R_HipRoll_Motor", "R_HipPitch_Motor",
    "R_Knee_Motor", "R_AnklePitch_Motor", "R_AnkleRoll_Motor",
    "Waist1_Motor", "Waist2_Motor", "Upperbody_Motor",
    "L_Shoulder1_Motor", "L_Shoulder2_Motor", "L_Shoulder3_Motor", "L_Armlink_Motor",
    "L_Elbow_Motor", "L_Forearm_Motor", "L_Wrist1_Motor", "L_Wrist2_Motor",
    "Neck_Motor", "Head_Motor",
    "R_Shoulder1_Motor", "R_Shoulder2_Motor", "R_Shoulder3_Motor", "R_Armlink_Motor",
    "R_Elbow_Motor", "R_Forearm_Motor", "R_Wrist1_Motor", "R_Wrist2_Motor"};

static constexpr const char *LINK_NAME[LINK_NUMBER] = {
    "Pelvis_Link", "Waist1_Link", "Waist2_Link", "Upperbody_Link",
    "L_HipRoll_Link", "L_HipCenter_Link", "L_Thigh_Link", "L_Knee_Link", "L_AnkleCenter_Link", "L_AnkleRoll_Link",
    "R_HipRoll_Link", "R_HipCenter_Link", "R_Thigh_Link", "R_Knee_Link", "R_AnkleCenter_Link", "R_AnkleRoll_Link",
    "L_Shoulder1_Link", "L_Shoulder2_Link", "L_Shoulder3_Link", "L_Armlink_Link", "L_Elbow_Link", "L_Forearm_Link", "L_Wrist1_Link", "L_Wrist2_Link",
    "R_Shoulder1_Link", "R_Shoulder2_Link", "R_Shoulder3_Link", "R_Armlink_Link", "R_Elbow_Link", "R_Forearm_Link", "R_Wrist1_Link", "R_Wrist2_Link",
    "Neck_Link", "Head_Link"};
} // namespace TOCABI

const int Pelvis = 0;
const int Upper_Body = 3;

const int Left_Foot = 9;
const int Right_Foot = 15;

const int Left_Hand = 23;
const int Right_Hand = 31;

const int COM_id = 32;

const int Head = 34;

const int LEFT = 0;
const int RIGHT = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//Terminal Data Que
class TQue
{
public:
  bool update;
  //std::string text;
  char text[256];
};

//Robot data Storage
class DataContainer
{
public:
  ros::NodeHandle nh;
  //Basic var
  bool simulation = true;
  bool connected = false;
  bool firstcalcdone = false;
  bool statemanager_ready = false;
  bool print_delay_info = false;
  bool print_elmo_info_tofile = false;
  bool start_initialize_sequence = false;

  bool torquezeroByTerminal = false;
  bool disableSafetyLock = false;
  bool ftcalib = false;

  std::string mode;

  //Tui Var..
  bool state_end;
  bool dynamics_end;
  int t_que = 0;
  
  std::chrono::steady_clock::time_point start_time_point;

  double time;
  double com_time;
  double sim_time;

  int dym_hz;
  std::chrono::microseconds dym_timestep;

  int stm_hz;
  std::chrono::microseconds stm_timestep;

  bool check = false;

  Eigen::VectorQd q_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;

  //Kinematics Information :
  Link link_[LINK_NUMBER + 1];

  double roll, pitch, yaw;

  Eigen::MatrixVVd A_;
  Eigen::MatrixVVd A_inv;

  Com com_;

  Eigen::VectorVQd tau_nonlinear_;
  Eigen::VectorQd torque_;

  //Command Var
  Eigen::VectorQd torque_desired;

  //Kinematics Information :
  //MODEL Tocabi;
  RobotData tocabi_;

  //Model var

  std::string homedir;

  //For real robot
  std::string ifname;
  int ctime;

  Eigen::VectorQd q_init_;
  Eigen::VectorQd q_elmo_;
  Eigen::VectorQd torqueElmo;
  Eigen::VectorQd torqueDemandElmo;
  Eigen::VectorQd positionDesired;
  Eigen::VectorQd accel_dif;
  Eigen::VectorQd accel_obsrvd;
  Eigen::VectorQd currentGain;

  int elmo_cnt;

  std::ofstream f_out;

  //Gui Command
  std::string command;

  double commandTime = 0.0;
  double commandTimeLock = -1.0;

  //Hardware switch

  bool torqueOn = false;
  bool torqueOff = false;
  bool emergencyoff = false;
  double t_gain = 0.0;
  double torqueOnTime = 0.0;
  double torqueOffTime = 0.0;

  bool elmo_Ready = false;
  //Simulation switch

  bool pubmode = false;      // Publish mode of mujoco, integrated mode(basic), detached mode.
  bool checkfreqency = true; // check running frequency of state thread and dynamics thread.

  bool testmode = false; // switch for controller test mode.

  //Controller switch

  bool positionControl = false;
  bool gravityMode = false;
  bool customGain = false;
  bool fixedgravity = false;
  bool torqueredis = false;
  bool qp2nd = false;

  bool spalarm = false; // support polygon alarm bool
  bool semode = true;  // state estimation running or not.

  bool initialize_request = false;



  ros::Publisher statusPub;
  std_msgs::String statusPubMsg;
};

static volatile sig_atomic_t shutdown_tocabi = 0;

//static volatile bool shutdown_tocabi_bool = false;


//static atomic
const std::string cred("\033[0;31m");
const std::string creset("\033[0m");
const std::string cblue("\033[0;34m");
const std::string cgreen("\033[0;32m");
const std::string cyellow("\033[0;33m");

#endif