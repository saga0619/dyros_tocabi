#include "tocabi_controller/dynamics_manager.h"
#include "tocabi_controller/mujoco_interface.h"
#include "tocabi_controller/realrobot_interface.h"
#include "custom_controller.h"

extern volatile bool shutdown_tocabi_bool;

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
  void trajectoryplannar();
  void tuiThread();
  void testThread();
  void TaskCommandCallback(const tocabi_controller::TaskCommandConstPtr &msg);
  void TaskQueCommandCallback(const tocabi_controller::TaskCommandQueConstPtr &msg);
  void ContinuityChecker(double data);
  void ZMPmonitor();
  void gettaskcommand(tocabi_controller::TaskCommand &msg);
  void pubfromcontroller();

  ros::Subscriber task_command;
  ros::Subscriber task_command_que;

  tocabi_controller::TaskCommandQue tque_msg;
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

  std::vector<TaskCommand> tc_que_;

  //sim variables
  double time;
  double sim_time;
  double control_time_;
  double control_time_pre_;

  bool safetymode;

  bool task_switch = false;
  bool task_que_switch = false;
  bool task_que_start = false;
  bool tc_command = false;

  int task_que_left = 0;

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
  Eigen::VectorQd torque_grav;

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