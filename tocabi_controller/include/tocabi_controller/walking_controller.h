#ifndef WALKINGCONTROLLER_H
#define WALKINGCONTROLLER_H

#include "tocabi_controller/walking_pattern.h"


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

class Walking_controller : public WalkingPattern
{
public:
    Walking_controller(DataContainer &dc_global, RobotData &kind);
    //~Walking_controller();
    
    DataContainer &dc;
    RobotData &rk_;

    void inverseKinematics();
    void walkingCompute();
    void setInitialPose();
    void getRobotState();
    void getRobotInitState();
    void setRobotStateInitialize();
    void updateNextStepTime();
    void getUiWalkingParameter(WalkingCommand wtc, int controller_Hz);
    void setWalkingParameter();

private:

};


#endif