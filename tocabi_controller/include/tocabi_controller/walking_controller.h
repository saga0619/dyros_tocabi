#ifndef WALKINGCONTROLLER_H
#define WALKINGCONTROLLER_H

#include "tocabi_controller/walking_pattern.h"
#include "tocabi_controller/link.h"

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
};

class Walking_controller : virtual public WalkingPattern
{
public:
    WalkingCommand wtc;
    Eigen::Vector12d desired_leg_q;

    void walkingCompute(RobotData Robot);
    void inverseKinematics(Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d& leg_q);
    void setInitPose();
    void getRobotState(RobotData Robot);
    void getRobotInitState(RobotData Robot);
    void setRobotStateInitialize();
    void updateNextStepTime();
    void getUiWalkingParameter(int controller_Hz, RobotData Robot);
    void setWalkingParameter(RobotData Robot);
    void walkingInitialize();

private:

};


#endif