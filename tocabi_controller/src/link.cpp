#include "tocabi_controller/link.h"
#include "ros/ros.h"

void Link::initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass, Eigen::Vector3d &local_com_position)
{
    id = id_;
    Mass = mass;
    COM_position = local_com_position;
    name = name_;
    Rotm.setZero();
    inertia.setZero();
    contact_point.setZero();

    model = &model_;

    Jac.setZero(6, MODEL_DOF + 6);
    Jac_COM.setZero(6, MODEL_DOF + 6);
    Jac_COM_p.setZero(3, MODEL_DOF + 6);
    Jac_COM_r.setZero(3, MODEL_DOF + 6);
    Jac_Contact.setZero(6, MODEL_DOF + 6);
    Jac_point.setZero(6, MODEL_DOF + 6);

    j_temp.setZero(6, MODEL_DOF + 6);
    j_temp2.setZero(6, MODEL_DOF + 6);
}

void Link::pos_Update(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_)
{
    mtx_rbdl.lock();
    xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, Eigen::Vector3d::Zero(), false);
    xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, COM_position, false);
    Rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, id, false)).transpose();

    mtx_rbdl.unlock();
    // COM_position =
    // RigidBodyDynamics::CalcBaseToBodyCoordinates(model_,q_virtual_,link_[i])
}

void Link::COM_Jac_Update(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_)
{
    Eigen::MatrixXd j_p_(3, MODEL_DOF + 6), j_r_(3, MODEL_DOF + 6);
    Eigen::MatrixXd j_(6, MODEL_DOF + 6);
    Eigen::MatrixXd fj_(6, MODEL_DOF + 6);

    fj_.setZero();

    mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, COM_position, fj_, false);

    mtx_rbdl.unlock();
    j_p_ = fj_.block<3, MODEL_DOF + 6>(3, 0);
    j_r_ = fj_.block<3, MODEL_DOF + 6>(0, 0);

    Jac_COM_p = j_p_; //*E_T_;
    Jac_COM_r = j_r_; //*E_T_;
    // Jac_COM_p.block<3,3>(0,3)=- DyrosMath::skm(xipos -
    // link_[0].xpos);
    j_.block<3, MODEL_DOF + 6>(0, 0) = Jac_COM_p;
    j_.block<3, MODEL_DOF + 6>(3, 0) = Jac_COM_r;

    Jac_COM = j_;
}

void Link::Set_Jacobian(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position)
{
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);

    mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Jacobian_position, j_temp, false);

    mtx_rbdl.unlock();
    Jac.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0);
    Jac.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0);
}

void Link::Set_Contact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position)
{
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);

    mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, id, Contact_position, j_temp, false);

    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, id, Contact_position, false);

    mtx_rbdl.unlock();
    // Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    Jac_Contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0);
    Jac_Contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0);

    // Jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}

void Link::Set_Sensor_Position(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Sensor_position)
{
    mtx_rbdl.lock();
    xpos_sensor = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_virtual_, id, Sensor_position, false);
    mtx_rbdl.unlock();
}

void Link::Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position)
{
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);
    mtx_rbdl.lock();
    RigidBodyDynamics::CalcPointJacobian6D(*model, q_virtual_, id, Contact_position, j_temp, false);
    xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_virtual_, id, Contact_position, false);

    mtx_rbdl.unlock();
    // Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
    // Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
    Jac_Contact.block<3, MODEL_DOF + 6>(0, 0) = j_temp.block<3, MODEL_DOF + 6>(3, 0);
    Jac_Contact.block<3, MODEL_DOF + 6>(3, 0) = j_temp.block<3, MODEL_DOF + 6>(0, 0);

    // Jac_Contact.block<3,3>(0,3)= -
    // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,id,Contact_position,false)
    // - link_[0].xpos);
}

void Link::vw_Update(Eigen::VectorVQd &q_dot_virtual)
{
    Eigen::Vector6d vw;
    vw = Jac * q_dot_virtual;
    v = vw.segment(0, 3);
    w = vw.segment(3, 3);
}

void Link::Set_Trajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired)
{
    x_traj = position_desired;
    v_traj = velocity_desired;
    r_traj = rotation_desired;
    w_traj = rotational_velocity_desired;
}

void Link::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, x_init(j), 0, 0, x_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void Link::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, x_init(j), 0, 0, pos_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void Link::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), 0, 0, pos_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void Link::Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), vel_init(j), 0, pos_desired(j), vel_desired(j), 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void Link::Set_Trajectory_rotation(double current_time, double start_time, double end_time, bool local_)
{
    //if local_ is true, global based rotation control
    Eigen::Vector3d axis;
    double angle;
    if (local_)
    {
        Eigen::AngleAxisd aa(rot_desired);
        axis = aa.axis();
        angle = aa.angle();
    }
    else
    {
        Eigen::AngleAxisd aa(rot_init.transpose() * rot_desired);
        axis = aa.axis();
        angle = aa.angle();
    }
    // double c_a = DyrosMath::cubic(current_time, start_time, end_time, 0.0, angle, 0.0, 0.0);
    // Eigen::Matrix3d rmat;
    // rmat = Eigen::AngleAxisd(c_a, axis);

    // r_traj = rot_init * rmat;

    // double dtime = 0.001;
    // double c_a_dtime = DyrosMath::cubic(current_time + dtime, start_time, end_time, 0.0, angle, 0.0, 0.0);

    // Eigen::Vector3d ea = r_traj.eulerAngles(0, 1, 2);

    // Eigen::Vector3d ea_dtime = (rot_init * Eigen::AngleAxisd(c_a_dtime, axis)).eulerAngles(0, 1, 2);

    // w_traj = (ea_dtime - ea) / dtime;
    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, 0.0, 0.0, 0.0, angle, 0.0, 0.0);
    double c_a = quintic(0);
    Eigen::Matrix3d rmat;

    rmat = Eigen::AngleAxisd(c_a, axis);
    r_traj = rot_init * rmat;
    w_traj =  quintic(1) * axis;
}

void Link::Set_Trajectory_rotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired_, bool local_)
{
    Eigen::Vector3d axis;
    double angle;
    if (local_)
    {
        Eigen::AngleAxisd aa(rot_desired_);
        axis = aa.axis();
        angle = aa.angle();
    }
    else
    {
        Eigen::AngleAxisd aa(rot_init.transpose() * rot_desired_);
        axis = aa.axis();
        angle = aa.angle();
    }
    double c_a = DyrosMath::cubic(current_time, start_time, end_time, 0.0, angle, 0.0, 0.0);
    Eigen::Matrix3d rmat;
    rmat = Eigen::AngleAxisd(c_a, axis);

    r_traj = rot_init * rmat;

    double dtime = 0.0001;
    double c_a_dtime = DyrosMath::cubic(current_time + dtime, start_time, end_time, 0.0, angle, 0.0, 0.0);

    Eigen::Vector3d ea = r_traj.eulerAngles(0, 1, 2);

    Eigen::Vector3d ea_dtime = (rot_init * Eigen::AngleAxisd(c_a_dtime, axis)).eulerAngles(0, 1, 2);

    w_traj = (ea_dtime - ea) / dtime;
}

void Link::Set_initpos()
{
    x_init = xpos;
    rot_init = Rotm;
}

std::ostream &operator<<(std::ostream &os, const Link &lk)
{
    os << "position of link : " << lk.name << std::endl
       << "x : " << lk.xpos(0) << " y : " << lk.xpos(1) << " z : " << lk.xpos(2) << std::endl;
    return os;
}

Eigen::Vector2d local2global(double x, double y, double angle)
{
}