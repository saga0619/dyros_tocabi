#include "tocabi_controller/dynamics_manager.h"
#include "tocabi_controller/mujoco_interface.h"
#include "tocabi_controller/realrobot_interface.h"
#include "custom_controller.h"

#define Kp_Yaw1s 2000  //Hip
#define Kp_Roll1s 2000  //Hip
#define Kp_Pitch1s 2000 //Hip
#define Kp_Pitch2s 2000 //Knee
#define Kp_Pitch12s 2000 //Ankle
#define Kp_Roll2s 2000  //Ankle

#define Kp_Waist1 2000  //Waist
#define Kp_Waist2 2000  //Waist
#define Kp_Upper 2000  //Waist

#define Kp_Shoulder1 2000  //Shoulder
#define Kp_Shoulder2 2000  //Shoulder
#define Kp_Shoulder12 2000  //Shoulder
#define Kp_Arm 2000  //Arm
#define Kp_Elbow 2000  //Elbow
#define Kp_Forearm 2000  //Elbow
#define Kp_Wrist1 2000  //Wrist
#define Kp_Wrist2 2000  //Wrist

#define Kp_Neck 2000  //Head
#define Kp_Head 2000  //Head

#define Kv_Yaw1s 12   //Hip
#define Kv_Roll1s 12  //Hip
#define Kv_Pitch1s 12 //Hip
#define Kv_Pitch2s 12 //Knee
#define Kv_Pitch12s 12 //Ankle
#define Kv_Roll2s 12  //Ankle

#define Kv_Waist1 12  //Waist
#define Kv_Waist2 12  //Waist
#define Kv_Upper 12  //Waist

#define Kv_Shoulder1 12  //Shoulder
#define Kv_Shoulder2 12  //Shoulder
#define Kv_Shoulder12 12  //Shoulder
#define Kv_Arm 12  //Arm
#define Kv_Elbow 12  //Elbow
#define Kv_Forearm 12  //Elbow
#define Kv_Wrist1 12  //Elbow
#define Kv_Wrist2 12  //Elbow

#define Kv_Neck 12  //Head
#define Kv_Head 12  //Head

extern volatile bool shutdown_tocabi_bool;
const double Kps[MODEL_DOF] =
    {
        Kp_Yaw1s,
        Kp_Roll1s,
        Kp_Pitch1s,
        Kp_Pitch2s,
        Kp_Pitch12s,
        Kp_Roll2s,
        Kp_Yaw1s,
        Kp_Roll1s,
        Kp_Pitch1s,
        Kp_Pitch2s,
        Kp_Pitch12s,
        Kp_Roll2s,
        Kp_Waist1,
        Kp_Waist2,
        Kp_Upper,
        Kp_Shoulder1,
        Kp_Shoulder2,
        Kp_Shoulder12,
        Kp_Arm,
        Kp_Elbow,
        Kp_Forearm,
        Kp_Wrist1,
        Kp_Wrist2,
        Kp_Neck,
        Kp_Head,
        Kp_Shoulder1,
        Kp_Shoulder2,
        Kp_Shoulder12,
        Kp_Arm,
        Kp_Elbow,
        Kp_Forearm,
        Kp_Wrist1,
        Kp_Wrist2};

const double Kvs[MODEL_DOF] =
    {
        Kv_Yaw1s,
        Kv_Roll1s,
        Kv_Pitch1s,
        Kv_Pitch2s,
        Kv_Pitch12s,
        Kv_Roll2s,
        Kv_Yaw1s,
        Kv_Roll1s,
        Kv_Pitch1s,
        Kv_Pitch2s,
        Kv_Pitch12s,
        Kv_Roll2s,
        Kv_Waist1,
        Kv_Waist2,
        Kv_Upper,
        Kv_Shoulder1,
        Kv_Shoulder2,
        Kv_Shoulder12,
        Kv_Arm,
        Kv_Elbow,
        Kv_Forearm,
        Kv_Wrist1,
        Kv_Wrist2,
        Kv_Neck,
        Kv_Head,
        Kv_Shoulder1,
        Kv_Shoulder2,
        Kv_Shoulder12,
        Kv_Arm,
        Kv_Elbow,
        Kv_Forearm,
        Kv_Wrist1,
        Kv_Wrist2};

class TocabiController
{
public:
  TocabiController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm);

  DataContainer &dc;
  CustomController &mycontroller;
  TaskCommand tc;

  WholebodyController wbc_;
  Walking_controller wkc_;

  void stateThread();
  void dynamicsThreadLow();
  void dynamicsThreadHigh();
  void tuiThread();
  void TaskCommandCallback(const tocabi_controller::TaskCommandConstPtr &msg);
  void ContinuityChecker(double data);
  void ZMPmonitor();
  void pubfromcontroller();

  ros::Subscriber task_command;
  std::ofstream data_out;

  ros::Publisher point_pub;
  ros::Publisher point_pub2;
  geometry_msgs::PolygonStamped pointpub_msg;
  geometry_msgs::PolygonStamped pointpub2_msg;

private:
  void getState();
  void initialize();
  StateManager &s_;
  DynamicsManager &d_;
  RobotData &tocabi_;

  bool connected;

  //sim variables
  double time;
  double sim_time;
  double control_time_;
  double control_time_pre_;

  bool safetymode;

  bool task_switch = false;
  bool tc_command = false;

  int dym_hz, stm_hz;
  /*
  Eigen::VectorQd q_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;*/
  Eigen::VectorQd q_desired_;
  Eigen::VectorQd q_dot_desired_;
  Eigen::VectorQd torque_;
  //Command Var
  Eigen::VectorQd torque_desired;

  //accel estim
  Eigen::VectorQd acceleration_estimated;
  Eigen::VectorQd acceleration_observed;
  Eigen::VectorQd acceleration_differance;
  Eigen::VectorQd q_dot_before_;
  Eigen::VectorQd acceleration_estimated_before;

  Eigen::VectorXd contact_force;

  //Kinematics Information :
  //Link link_[LINK_NUMBER + 1];
  double yaw_radian;
  Eigen::MatrixVVd A_;
  Eigen::MatrixVVd A_inv_;
  Com com_;
  int cr_mode;

  //Walking Information
  bool walkingCallbackOn;
  bool set_q_init;
};