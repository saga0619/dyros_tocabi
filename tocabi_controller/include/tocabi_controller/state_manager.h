#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "tocabi_controller/data_container.h"
#include "tocabi_controller/terminal.h"
#include "tocabi_controller/MotorInfo.h"
#include "geometry_msgs/PolygonStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "tocabi_controller/TaskCommand.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

extern std::mutex mtx;
extern std::mutex mtx_rbdl;
extern std::mutex mtx_dc;
extern volatile bool shutdown_tocabi_bool;

class StateManager
{
public:
  StateManager(DataContainer &dc_global);
  virtual ~StateManager() {}
  DataContainer &dc;
  virtual void connect();
  virtual void stateThread(); //main thread managing state
  virtual void stateThread2(); //main thread managing state
  virtual void updateState();
  //virtual void sendCommand(Eigen::VectorQd command);
  virtual void sendCommand(Eigen::VectorQd command, double sim_time);

  //initialize variables
  virtual void initialize();
  //store data at container
  void storeState();

  void stateEstimate();
  //private functions


  //advertise informations to ROS
  void adv2ROS();


  //update kinematic information with RBDL
  void updateKinematics(const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);

  //testThread to test multithread
  void testThread();


  void initYaw();
  //private variables

  unsigned int link_id_[40];

  double control_time_;
  double control_time_before_;
  double sim_time_;

  int data_received_counter_;

  Eigen::VectorQd q_;
  Eigen::VectorQd q_init_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorQd torque_;
  Eigen::VectorQd torque_desired;
  Eigen::VectorVQd tau_nonlinear_;

  double roll, pitch, yaw;
  double yaw_radian;
  double yaw_init;

  Eigen::MatrixVVd A_;
  Eigen::MatrixVVd A_inv;
  Eigen::MatrixXd A_temp_;

  Eigen::Vector3d gravity_;

  RigidBodyDynamics::Model model_;

  Link link_[LINK_NUMBER + 1];
  Com com_;

  Eigen::Vector6d RF_FT, LF_FT, LH_FT, RH_FT;

  std::chrono::steady_clock::time_point st_start_time;


  //Communication Subscriber!

  ros::Subscriber gui_command;
  //ros::Subscriber task_command;
  ros::Publisher joint_states_pub;
  ros::Publisher time_pub;
  ros::Publisher motor_info_pub;
  ros::Publisher motor_acc_dif_info_pub;
  ros::Publisher point_pub;
  ros::Publisher point_pub2;

  ros::Publisher ft_viz_pub;
  visualization_msgs::MarkerArray ft_viz_msg;

  ros::Publisher tgainPublisher;
  std_msgs::Float32 tgain_p;

  sensor_msgs::JointState joint_state_msg;
  std_msgs::Float32 time_msg;
  tocabi_controller::MotorInfo motor_info_msg;
  tocabi_controller::MotorInfo acc_dif_info_msg;
  geometry_msgs::PolygonStamped pointpub_msg;

  void CommandCallback(const std_msgs::StringConstPtr &msg);
  //void TaskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr &msg);



  //Terminate Signal Handler
  //static void sigintHandler(int sig);
};

#endif