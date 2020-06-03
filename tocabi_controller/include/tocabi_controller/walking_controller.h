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
    Eigen::Vector12d leg_q_NC;

    void walkingCompute(RobotData Robot);
    void inverseKinematics(Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d& leg_q);
    void setInitPose(RobotData Robot, Eigen::Vector12d& leg_q);
    void getRobotState(RobotData Robot);
    void getRobotInitState(RobotData Robot);
    void walkingInitialize(RobotData Robot);
    void setRobotStateInitialize();
    void updateNextStepTime();
    void updateInitTime();
    void getUiWalkingParameter(int controller_Hz, int walking_enable, int ikmode, int walkingpattern, int footstepdir, double target_x, double target_y, double target_z, double theta, double targetheight, double steplength_x, double steplength_y, int dob_, RobotData Robot);
    void setWalkingParameter(RobotData Robot);

    void calcRobotState();
    void hipCompensator();
    void inverseKinematicsdob(RobotData &Robot);

private:

};


#endif