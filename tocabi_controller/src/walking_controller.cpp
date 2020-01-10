#include "tocabi_controller/walking_controller.h"

WalkingController::WalkingController(DataContainer &dc_global, RobotData &kind) : dc(dc_global), rk_(kind))
{
}

void WalkingController::inverseKinematics()
{

}