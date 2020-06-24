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
#include <std_msgs/Int32MultiArray.h>
#include <mujoco_ros_msgs/SensorState.h>
#include <mujoco_ros_msgs/JointSet.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <atomic>

#include <stdlib.h>
#include <signal.h>

#include "tocabi_controller/walking_controller.h"
#include "tocabi_controller/wholebody_controller.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//robot definition variables here

struct TaskCommand
{
  double command_time;
  double traj_time;
  bool task_init;
  int mode;
  // COM Related
  double ratio;
  double height;
  double pelv_pitch;
  double roll;
  double pitch;
  double yaw;
  // Arm Related
  double l_x;
  double l_y;
  double l_z;
  double l_roll;
  double l_pitch;
  double l_yaw;
  double r_x;
  double r_y;
  double r_z;
  double r_roll;
  double r_pitch;
  double r_yaw;

  int solver;
  int contactredis;

  double init_com_height;

  //Walking Related
  int walking_enable;
  int ik_mode;
  int walking_pattern;
  int foot_step_dir;
  double target_x;
  double target_y;
  double target_z;
  double theta;
  double walking_height;
  double step_length_x;
  double step_length_y;
  bool dob;

  //taskgain
  bool custom_taskgain;
  double pos_p;
  double pos_d;
  double ang_p;
  double ang_d;
  double acc_p;
};

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
  bool start_initialize_lower = false;
  bool signal_imu_reset = false;
  bool operation_ready = false;
  bool torquezeroByTerminal = false;
  bool disableSafetyLock = false;
  bool safetycheckdisable = false;
  bool ftcalib = false;




  bool print_data_to_file = false;
  bool open_file_for_print = false;
  bool print_data_ready = false;
  std::string sim_mode;
  std::string mode;
  std::string print_file_name;
  std::ofstream data_out;

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
  Eigen::VectorQd q_ext_;

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
  WholebodyController wbc_;
  Walking_controller wkc_;

  //Model var

  std::string homedir;

  //For real robot
  std::string ifname;
  std::string ifname2;
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
  bool showdata = false;
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
  bool positionGravControl = false;
  bool signal_gravityCompensation = false;
  bool customGain = false;
  bool fixedgravity = false;
  bool signal_contactTorqueRedistribution = false;
  bool qp2nd = false;
  bool enable_lpf = false;
  bool switch_lpf = false;

  bool spalarm = false; // support polygon alarm bool
  bool semode = false;  // state estimation running or not.
  bool semode_init = true;

  bool safetyison = false;

  bool initialize_request = false;

  bool use_virtual_for_mujoco = false;

  int imu_state = 0;  //0 not ok 1 not valid 2 valid
  int zp_state = 0;   //0 not ok 1 ready 2 ok
  int ft_state = 0;   //0 not ok 1 init req 2 ok
  int ecat_state = 0; //0 not ok 1 ok 2 commutation
  int se_state = 0; //se state. 0: off 1: on
  int tc_state = 3; //task control state 0 : on 1 : warn 2: error 3: off

  //Simulation mode
  bool simulationMode;

  bool use_virtual_joint = false;

  ros::Publisher statusPub;
  std_msgs::String statusPubMsg;
  ros::Publisher rgbPub;
  std_msgs::Int32MultiArray rgbPubMsg;
};

static volatile sig_atomic_t shutdown_tocabi = 0;

//static volatile bool shutdown_tocabi_bool = false;

#endif