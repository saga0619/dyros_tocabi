#include "tocabi_controller/qp_status.h"

StatusManager::StatusManager(RobotData &rd_)
{

    std::string urdf_path = "/home/saga/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi.urdf";
    RigidBodyDynamics::Model model_;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, true);
    for (int i = 0; i < LINK_NUMBER; i++)
    {
        link_id_[i] = model_.GetBodyId(TOCABI::LINK_NAME[i]);
    }
    double total_mass = 0;

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        rd_.link_[i].initialize(model_, link_id_[i], TOCABI::LINK_NAME[i], model_.mBodies[link_id_[i]].mMass, model_.mBodies[link_id_[i]].mCenterOfMass);
        total_mass += rd_.link_[i].Mass;
    }

    rd_.link_[Right_Foot].contact_point << 0.03, 0, -0.1585;
    rd_.link_[Right_Foot].sensor_point << 0.0, 0.0, -0.09;
    rd_.link_[Left_Foot].contact_point << 0.03, 0, -0.1585;
    rd_.link_[Left_Foot].sensor_point << 0.0, 0.0, -0.09;

    rd_.link_[Right_Hand].contact_point << 0, 0.0, -0.035;
    rd_.link_[Right_Hand].sensor_point << 0.0, 0.0, 0.0;
    rd_.link_[Left_Hand].contact_point << 0, 0.0, -0.035;
    rd_.link_[Left_Hand].sensor_point << 0.0, 0.0, 0.0;

    A_.setZero();
    A_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    
}

void StatusManager::updateKinematics(RobotData &rd_, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{

    A_temp_.setZero();

    std::cout << "1" << std::endl;
    RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);

    std::cout << "1" << std::endl;
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_f, A_temp_, false);

    std::cout << "1" << std::endl;

    A_ = A_temp_;
    A_inv = A_.inverse();
    std::cout << "1" << std::endl;
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        rd_.link_[i].pos_Update(model_, q_virtual_f);
    }
    Eigen::Vector3d zero;
    zero.setZero();
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        rd_.link_[i].Set_Jacobian(model_, q_virtual_f, zero);
    }

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {

        rd_.link_[i].COM_Jac_Update(model_, q_virtual_f);
    }
    //COM link information update ::
    double com_mass;
    RigidBodyDynamics::Math::Vector3d com_pos;
    RigidBodyDynamics::Math::Vector3d com_vel, com_accel, com_ang_momentum, com_ang_moment;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_f, q_dot_virtual_f, &q_ddot_virtual_f, com_mass, com_pos, &com_vel, &com_accel, &com_ang_momentum, &com_ang_moment, false);

    com_.mass = com_mass;
    com_.pos = com_pos;

    Eigen::Vector3d vel_temp;
    vel_temp = com_.vel;
    com_.vel = com_vel;

    com_.accel = com_accel;
    com_.angular_momentum = com_ang_momentum;
    com_.angular_moment = com_ang_moment;

    double w_ = sqrt(9.81 / com_.pos(2));

    com_.ZMP(0) = com_.pos(0) - com_.accel(0) / pow(w_, 2);
    com_.ZMP(1) = com_.pos(1) - com_.accel(1) / pow(w_, 2);

    //com_.ZMP(0) = (com_.pos(0) * (com_.accel(2) + 9.81) - com_pos(2) * com_accel(0)) / (com_.accel(2) + 9.81) - com_.angular_momentum(2) / com_.mass / (com_.accel(2) + 9.81);
    //com_.ZMP(1) = (com_.pos(1) * (com_.accel(2) + 9.81) - com_pos(2) * com_accel(1)) / (com_.accel(2) + 9.81) - com_.angular_momentum(1) / com_.mass / (com_.accel(2) + 9.81);

    com_.CP(0) = com_.pos(0) + com_.vel(0) / w_;
    com_.CP(1) = com_.pos(1) + com_.vel(1) / w_;

    Eigen::Matrix3Vd jacobian_com;

    jacobian_com.setZero();

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        jacobian_com += rd_.link_[i].Jac_COM_p * rd_.link_[i].Mass;
    }

    rd_.link_[COM_id].Jac.setZero(6, MODEL_DOF + 6);

    //link_p[COM_id].Jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6) / com_.mass;
    //link_p[COM_id].Jac.block(2, 0, 4, MODEL_DOF + 6) = link_p[Pelvis].Jac.block(2, 0, 4, MODEL_DOF + 6);

    rd_.link_[COM_id].Jac.block(0, 0, 3, MODEL_DOF + 6) = jacobian_com.block(0, 0, 3, MODEL_DOF + 6) / com_.mass;
    rd_.link_[COM_id].Jac.block(3, 0, 3, MODEL_DOF + 6) = rd_.link_[Pelvis].Jac.block(3, 0, 3, MODEL_DOF + 6);

    rd_.link_[COM_id].Jac_COM_p = jacobian_com / com_.mass;
    rd_.link_[COM_id].xpos = com_.pos;
    //link_p[COM_id].xpos(2) = link_p[Pelvis].xpos(2);
    rd_.link_[COM_id].Rotm = rd_.link_[Pelvis].Rotm;

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        rd_.link_[i].vw_Update(q_dot_virtual_f);
    }
}

void StatusManager::storeState(RobotData &rd_)
{
    rd_.com_ = com_;
    rd_.A_ = A_;
}
