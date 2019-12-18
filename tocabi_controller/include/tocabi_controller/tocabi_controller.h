#include "tocabi_controller/dynamics_manager.h"
#include "tocabi_controller/mujoco_interface.h"
#include "tocabi_controller/realrobot_interface.h"
#include "tocabi_controller/wholebody_controller.h"

#define Kp_Yaw1s 1500   //Hip
#define Kp_Roll1s 5000  //Hip
#define Kp_Pitch1s 5000 //Hip
#define Kp_Pitch2s 5000 //Knee
#define Kp_Pitch3s 5000 //Ankle
#define Kp_Roll2s 6500  //Ankle

#define Kv_Yaw1s 50   //Hip
#define Kv_Roll1s 50  //Hip
#define Kv_Pitch1s 50 //Hip
#define Kv_Pitch2s 40 //Knee
#define Kv_Pitch3s 30 //Ankle
#define Kv_Roll2s 60  //Ankle

const double Kps[MODEL_DOF] =
    {
        Kp_Yaw1s,
        Kp_Roll1s,
        Kp_Pitch1s,
        Kp_Pitch2s,
        Kp_Pitch3s,
        Kp_Roll2s,
        Kp_Yaw1s,
        Kp_Roll1s,
        Kp_Pitch1s,
        Kp_Pitch2s,
        Kp_Pitch3s,
        Kp_Roll2s};

const double Kvs[MODEL_DOF] =
    {
        Kv_Yaw1s,
        Kv_Roll1s,
        Kv_Pitch1s,
        Kv_Pitch2s,
        Kv_Pitch3s,
        Kv_Roll2s,
        Kv_Yaw1s,
        Kv_Roll1s,
        Kv_Pitch1s,
        Kv_Pitch2s,
        Kv_Pitch3s,
        Kv_Roll2s};

struct TaskCommand
{
  double command_time;
  double traj_time;
  double ratio;
  double height;
  double angle;
  bool task_init;
  int mode;
};

class TocabiController
{
public:
  TocabiController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm);

  void stateThread();
  void dynamicsThreadLow();
  void dynamicsThreadHigh();
  void tuiThread();
  DataContainer &dc;
  TaskCommand tc;
  ros::Subscriber task_command;
  void TaskCommandCallback(const tocabi_controller::TaskCommandConstPtr &msg);
  void ContinuityChecker(double data);
  void ZMPmonitor();
  std::ofstream out;

  void pubfromcontroller();
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

  bool safetymode;

  bool task_switch = false;

  int dym_hz, stm_hz;

  Eigen::VectorQd q_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorQd q_desired_;
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
};