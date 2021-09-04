#include "tocabi_controller/qp.h"
#include "tocabi_controller/qp_status.h"
#include "tocabi_controller/wholebody_controller.h"

std::mutex mtx_rbdl;
volatile bool shutdown_tocabi_bool = false;

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "qp_test");

  RobotData rd_;
  StatusManager stm(rd_);

  rd_.q_virtual_ << 0, 0, 0.74783, 0, 0, 0,
      0.0, 0, -0.55, 1.4, -1.05, -0.0,
      0.0, 0, -0.55, 1.4, -1.05, -0.0,
      0, 0, 0,
      0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
      0, 0,
      -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;
  rd_.q_dot_virtual_.setZero();
  rd_.q_ddot_virtual_.setZero();

  Eigen::VectorQVQd q_virtual_ = rd_.q_virtual_;
  Eigen::VectorVQd qdot_virtual_ = rd_.q_dot_virtual_;
  Eigen::VectorVQd qddot_virtual_ = rd_.q_ddot_virtual_;

  std::cout << "updat_kinematics" << std::endl;
  stm.updateKinematics(rd_, q_virtual_, qdot_virtual_, qddot_virtual_);

  std::cout << "storeState" << std::endl;
  stm.storeState(rd_);

  WholebodyController wbc;

  std::cout << "init" << std::endl;
  wbc.init(rd_);

  std::cout << "set_contact" << std::endl;
  wbc.set_contact(rd_, 1, 1);

  std::cout << "gravity_compensation_torque" << std::endl;
  wbc.gravity_compensation_torque(rd_);

  return true;
}
