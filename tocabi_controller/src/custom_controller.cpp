#include "tocabi_controller/custom_controller.h"

CustomController::CustomController(RobotData &rd) : rd_(rd)
{
    ControlVal_.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::compute()
{
    //rd_.control_time_; current time
    //rd_.link_[Right_Foot].Jac : current rightfoot jac
    //rd_.q_dot_ : current q velocity
}