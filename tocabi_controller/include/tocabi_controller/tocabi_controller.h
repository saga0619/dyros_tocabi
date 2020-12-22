#include "tocabi_controller/dynamics_manager.h"
#include "tocabi_controller/mujoco_interface.h"
#include "tocabi_controller/positionCommand.h"
#include "custom_controller.h"

#ifdef COMPILE_REALROBOT
#include "tocabi_controller/realrobot_interface.h"
#endif

extern volatile bool shutdown_tocabi_bool;

class TocabiController
{
public:
  TocabiController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm);
  ~TocabiController();
  DataContainer &dc;
  CustomController &mycontroller;
  TaskCommand tc;
  VelCommand vc;

  WholebodyController wbc_;
  Walking_controller wkc_;

  tocabi_controller::TaskCommand TaskCont;
  tocabi_controller::TaskCommandQue TaskQueCont;

  void GetTaskCommand();
  void stateThread();
  void dynamicsThreadLow();
  void dynamicsThreadHigh();
  void trajectoryplannar();
  void tuiThread();
  void testThread();

  void TaskCommandCallback(const tocabi_controller::TaskCommandConstPtr &msg);
  void TaskQueCommandCallback(const tocabi_controller::TaskCommandQueConstPtr &msg);
  void TaskGainCallback(const tocabi_controller::TaskGainCommandConstPtr &msg);
  void PositionCommandCallback(const tocabi_controller::positionCommandConstPtr &msg);
  void VelocityCommandCallback(const tocabi_controller::VelocityCommandConstPtr &msg);
  void ContinuityChecker(double data);
  void ZMPmonitor();
  void gettaskcommand(tocabi_controller::TaskCommand &msg);
  void customgainhandle();
  void CPpatternGen();
  Eigen::Vector3d velRegulation(Eigen::Vector3d traj_before, Eigen::Vector3d traj_now, Eigen::Vector3d acc_max);
  VectorQd positionCommandExt(double control_time, double traj_time, VectorQd current_pos, VectorQd desired_pos);

  ros::Subscriber task_command;
  ros::Subscriber task_command_que;
  ros::Subscriber taskgain_sub;
  ros::Subscriber vel_command_sub;
  ros::Subscriber position_command_sub;

  tocabi_controller::TaskCommandQue tque_msg;

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

  bool loop_pass = false;
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

  //Walking Information
  bool walkingCallbackOn;
  bool set_q_init;
};