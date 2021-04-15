#ifndef WALKINGCONTROLLER_H
#define WALKINGCONTROLLER_H

#include "tocabi_controller/walking_pattern.h"
#include "tocabi_controller/link.h"
#include "tocabi_controller/qp.h"
#include <qpOASES.hpp>

struct WalkingCommand
{
  int walking_enable;
  int ik_mode;
  int walking_pattern;
  int foot_step_dir;
  double target_x;
  double target_y;
  double target_z;
  double theta;
  double height;
  double step_length_x;
  double step_length_y;
  bool dob;
  bool imu;
};

class Walking_controller : virtual public WalkingPattern
{
public:
  WalkingCommand wtc;
  Eigen::Vector12d desired_leg_q;
  Eigen::Vector12d desired_leg_q_dot;
  Eigen::Vector12d desired_leg_q_NC;
  Eigen::VectorQd desired_init_leg_q;
  Eigen::Vector12d desired_leg_q_prev;
  Eigen::Vector12d desired_leg_q_dot_;
  Eigen::Vector12d desired_leg_q_temp;
  Eigen::Vector12d desired_leg_q_temp_;
  Eigen::Vector3d com_local;
  Eigen::Vector3d H_leg1;

  Eigen::Vector12d dob_debug;
  double a = 0;

  CQuadraticProgram QP_m;

  void walkingCompute(RobotData &Robot);
  void inverseKinematics(RobotData &Robot, Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d &leg_q);
  void setInitPose(RobotData &Robot, Eigen::VectorQd &leg_q);
  void getRobotState(RobotData &Robot);
  void modelFrameToLocal(RobotData &Robot);
  void getRobotInitState(RobotData &Robot);
  void walkingInitialize(RobotData &Robot);
  void setRobotStateInitialize(RobotData &Robot);
  void updateNextStepTime();
  void updateInitTime();
  void getUiWalkingParameter(int controller_Hz, int walking_enable, int ikmode, int walkingpattern, int walkingpattern2, int footstepdir, double target_x, double target_y, double target_z, double theta, double targetheight, double steplength_x, double steplength_y, int dob_walk, int imu_walk, bool mom_walk, int vibration, RobotData &Robot);

  void setWalkingParameter(RobotData &Robot);

  void calcRobotState(RobotData &Robot);
  void hipCompensator();
  void ankleOriControl(RobotData &Robot);
  void comVibrationController(RobotData &Robot);
  void inverseKinematicsdob(RobotData &Robot);
  void comJacobianState(RobotData &Robot);
  void comJacobianIK(RobotData &Robot);
  void momentumControl(RobotData &Robot);
  
private:
};
#endif
