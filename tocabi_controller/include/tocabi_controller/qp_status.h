
#include "tocabi_controller/link.h"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

class StatusManager
{
public:
    StatusManager(RobotData &rd_);

    void updateKinematics(RobotData &rd_, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f);
    void storeState(RobotData &rd_);

    Eigen::MatrixVVd A_;
    Eigen::MatrixVVd A_inv;
    Eigen::MatrixXd A_temp_;
    Eigen::Vector3d gravity_;

    RigidBodyDynamics::Model model_;

    unsigned int link_id_[40];
    Link link_[LINK_NUMBER + 1];
    Com com_;
};