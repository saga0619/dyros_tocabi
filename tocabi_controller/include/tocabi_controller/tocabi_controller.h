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
  RobotData_fast tocabi_fast_;

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
  int dynthread2_cnt = 0;

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

  //////////dg custom controller variables////////
  void setWalkingParameter(double walking_duration, double walking_speed, double step_width, double knee_target_angle);

  void initWalkingParameter();
  void getRobotData(WholebodyController &wc);
  void getProcessedRobotData(WholebodyController &wc);
  void walkingStateManager();
  void motionGenerator();
  void getCOMTrajectory();
  void getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired);
  void computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des);
  Eigen::VectorQd comVelocityControlCompute(WholebodyController &wc);
  Eigen::VectorQd comVelocityControlCompute2(WholebodyController &wc);
  Eigen::VectorQd jointTrajectoryPDControlCompute(WholebodyController &wc);
  bool balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d);
  int checkZMPinWhichFoot(Eigen::Vector2d zmp_measured); // check where the zmp is
  Eigen::VectorQd tuneTorqueForZMPSafety(Eigen::VectorQd task_torque); // check where the zmp is
  Eigen::VectorQd zmpAnkleControl();
  Eigen::VectorQd jointComTrackingTuning();
  Eigen::VectorQd stablePDControl(double kp, double kd, Eigen::VectorQd current_q, Eigen::VectorQd current_q_dot, Eigen::VectorQd estimated_q_ddot, Eigen::VectorQd desired_q_next_step); // without velocity reference
  Eigen::VectorQd stablePDControl(double kp, double kd, Eigen::VectorQd current_q, Eigen::VectorQd current_q_dot, Eigen::VectorQd estimated_q_ddot, Eigen::VectorQd desired_q_next_step, Eigen::VectorQd desired_q_dot_next_step); // with velocity reference

  void computeZmp();
  void savePreData();


  bool walking_mode_on_;                                  // turns on when the walking control command is received and truns off after saving start time

  int foot_contact_;                                      // 1:left,   -1:right,   0:double
  bool foot_swing_trigger_;                               // trigger in on when the absolute value of the com velocity is over the threshold
  double stop_vel_threshold_;                             // acceptable capture point deviation from support foot
  bool first_step_trigger_;                               // turns of this is first step and current speed is over than the certain portion of target speed
  bool first_step_flag_;                                  // turns off when the commanded walking speed is not zero
  bool stop_walking_trigger_;                             // turns on when the robot's speed become zero and lands last foot step
  double stance_start_time_;

  double walking_duration_;
  double walking_phase_;
  double turning_phase_;
  double switching_phase_duration_;
  
  double current_time_;
  double pre_time_;
  double start_time_;
  double dt_;

  double walking_speed_;
  double yaw_angular_vel_;
  double knee_target_angle_;

  double step_width_;
  double step_length_;

  double swing_foot_height_;
  
  double first_torque_supplier_;

  // CoM variables
  Eigen::Vector3d com_pos_desired_; 
  Eigen::Vector3d com_vel_desired_;
  Eigen::Vector3d com_pos_current_;
  Eigen::Vector3d com_vel_current_; 
  Eigen::Vector3d com_pos_init_;
  Eigen::Vector3d com_vel_init_;
  Eigen::Vector3d com_pos_desired_pre_; 
  Eigen::Vector3d com_vel_desired_pre_;
  Eigen::Vector3d com_acc_desired_;

  Eigen::Vector3d com_pos_current_pelvis_;
  Eigen::Vector3d com_vel_current_pelvis_; 
  Eigen::Vector3d com_pos_init_pelvis_;
  Eigen::Vector3d com_vel_init_pelvis_;
  Eigen::Vector3d com_pos_desired_pre_pelvis_; 
  Eigen::Vector3d com_vel_desired_pre_pelvis_;
  // Pevlis related variables
  Eigen::Vector3d pelv_pos_current_;
  Eigen::Vector6d pelv_vel_current_;
  Eigen::Matrix3d pelv_rot_current_;
  Eigen::Vector3d pelv_rpy_current_;
  Eigen::Matrix3d pelv_rot_current_yaw_aline_;

  Eigen::Matrix3d pelv_yaw_rot_current_from_global_;

  Eigen::Vector3d pelv_pos_init_;
  Eigen::Vector6d pelv_vel_init_;
  Eigen::Matrix3d pelv_rot_init_;
  Eigen::Vector3d pelv_rpy_init_;
  Eigen::Matrix3d pelv_rot_init_yaw_aline_;

  // Joint related variables
  Eigen::VectorQd current_q_;
  Eigen::VectorQd current_q_dot_;
  Eigen::VectorQd current_q_ddot_;
  Eigen::VectorQd desired_q_;
  Eigen::VectorQd desired_q_dot_;
  Eigen::VectorQd desired_q_ddot_;
  Eigen::VectorQd pre_q_;
  Eigen::VectorQd pre_desired_q_;

  Eigen::VectorQd motion_q_;
  Eigen::VectorQd motion_q_dot_;
  Eigen::VectorQd init_q_;

  Eigen::VectorQd pd_control_mask_; //1 for joint ik pd control

  Eigen::Vector2d target_foot_landing_from_pelv_;
  Eigen::Vector2d target_foot_landing_from_sup_;
  Eigen::Vector3d swing_foot_pos_trajectory_from_global_;
  Eigen::Vector6d swing_foot_vel_trajectory_from_global_;
  Eigen::Vector6d swing_foot_acc_trajectory_from_global_;
  Eigen::Matrix3d swing_foot_rot_trajectory_from_global_;

  Eigen::Isometry3d swing_foot_transform_init_;
  Eigen::Isometry3d support_foot_transform_init_;

  Eigen::Isometry3d swing_foot_transform_current_;
  Eigen::Isometry3d support_foot_transform_current_;

  Eigen::Vector6d swing_foot_vel_current_;
  Eigen::Vector6d swing_foot_vel_init_;


  Eigen::MatrixXd jac_com_;
  Eigen::MatrixXd jac_com_pos_;
  Eigen::MatrixXd jac_rhand_;
  Eigen::MatrixXd jac_lhand_;
  Eigen::MatrixXd jac_rfoot_;
  Eigen::MatrixXd jac_lfoot_;


  Eigen::Isometry3d lfoot_transform_init_from_global_;
  Eigen::Isometry3d rfoot_transform_init_from_global_;
  Eigen::Isometry3d lfoot_transform_init_from_pelvis_;
  Eigen::Isometry3d rfoot_transform_init_from_pelvis_;
  Eigen::Isometry3d lfoot_transform_init_from_sup_;
  Eigen::Isometry3d rfoot_transform_init_from_sup_;

  Eigen::Isometry3d lfoot_transform_current_from_global_;
  Eigen::Isometry3d rfoot_transform_current_from_global_;
  Eigen::Isometry3d lfoot_transform_current_from_pelvis_;
  Eigen::Isometry3d rfoot_transform_current_from_pelvis_;
  Eigen::Isometry3d lfoot_transform_current_from_sup_;
  Eigen::Isometry3d rfoot_transform_current_from_sup_;

  Eigen::Vector6d lfoot_vel_current_from_global;
  Eigen::Vector6d rfoot_vel_current_from_global;

  Eigen::Vector3d middle_of_both_foot_;

  Eigen::Vector3d zmp_measured_;
  Eigen::Vector3d zmp_measured_pre_;
  Eigen::Vector3d zmp_measured_ppre_;
  Eigen::Vector3d zmp_dot_measured_;

  Eigen::Vector3d zmp_measured_local_; //calc zmp with F/T sensors according to the Robot.ee_[0].contact
  Eigen::Vector3d zmp_dot_measured_local_;

  Eigen::Vector3d zmp_local_lfoot_;
  Eigen::Vector3d zmp_local_rfoot_;
  Eigen::Vector3d zmp_local_lfoot_pre_;
  Eigen::Vector3d zmp_local_rfoot_pre_;
  Eigen::Vector3d zmp_dot_local_rfoot_;
  Eigen::Vector3d zmp_dot_local_lfoot_;

  Eigen::Vector3d zmp_measured_lfoot_; //calc only left foot zmp with a F/T sensor
  Eigen::Vector3d zmp_measured_rfoot_;

  Eigen::Vector3d zmp_desired_from_global_;
  Eigen::Vector3d zmp_desired_pre_;

  Eigen::Vector6d l_ft_;
  Eigen::Vector6d r_ft_;

  Eigen::Vector2d f_star_xy_;
  Eigen::Vector2d f_star_xy_pre_;
  Eigen::Vector6d f_star_6d_;
  Eigen::Vector6d f_star_6d_pre_;

  Eigen::VectorQd torque_task_;
  Eigen::VectorQd torque_grav_;
  Eigen::VectorQd torque_task_pre_;
  Eigen::VectorQd torque_grav_pre_;

  //Walking Information
  bool walkingCallbackOn;
  bool set_q_init;
};