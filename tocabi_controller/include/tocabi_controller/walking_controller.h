#ifndef WALKINGCONTROLLER_H
#define WALKINGCONTROLLER_H

#include "tocabi_controller/walking_pattern.h"
#include "tocabi_controller/data_container.h"
#include "math_type_define.h"

class WalkingController
{
public:
    WalkingController(DataContainer &dc_global, RobotData &kind);
    ~WalkingController();
    
    DataContainer &dc;
    RobotData &rk_;

    
    void inverseKinematics();

private:
};


#endif