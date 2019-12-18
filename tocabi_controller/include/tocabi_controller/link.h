#ifndef LINK_H
#define LINK_H

#include "Eigen/Dense"
#include "math_type_define.h"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <mutex>

extern std::mutex mtx_rbdl;

struct Com
{
  double mass;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d angular_momentum;
  Eigen::Vector2d ZMP;
  Eigen::Vector2d CP;
  Eigen::MatrixXd Jac;
};

class Link
{
public:
  // Update link i of rbdl link id. name : link name, mass : link mass, xipos : local center of mass position
  void initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass, Eigen::Vector3d &local_com_position);

  // Update COM jacobian
  void COM_Jac_Update(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_);

  // Update xpos, xipos, rotm.
  void pos_Update(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_);

  // Set Contact point, Contact jacobian
  void Set_Contact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // Set Contact point, Contact jacobian
  void Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // Set Sensor Position
  void Set_Sensor_Position(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Sensor_position);

  // update Jacobian matrix of local position at link.
  void Set_Jacobian(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position);

  // update link velocity(6D, translation and rotation) from jacobian matrix Jac.
  void vw_Update(Eigen::VectorVQd &q_dot_virtual);

  // set link Trajectory of id i.
  void Set_Trajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from cubic spline.
  void Set_Trajectory_from_cubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

  // set realtime trajectory of rotation of link
  void Set_Trajectory_rotation(double current_time, double start_time, double end_time, bool local_);

  // set realtime trajectory of rotation of link
  void Set_Trajectory_rotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired, bool local_);

  // set link initial position and rotation. initial position for task control.
  void Set_initpos();

  //constant variables
  int id;
  double Mass;
  std::string name;

  //local COM position of body
  Eigen::Vector3d COM_position;

  //inertial matrix
  Eigen::Matrix3d inertia;

  //local contact point
  Eigen::Vector3d contact_point;

  //local sensor point
  Eigen::Vector3d sensor_point;

  //changing variables
  //rotation matrix
  Eigen::Matrix3d Rotm;

  //global position of body
  Eigen::Vector3d xpos;

  //global COM position of body
  Eigen::Vector3d xipos;

  //global position of contact point at body
  Eigen::Vector3d xpos_contact;

  //global position of sensor at body
  Eigen::Vector3d xpos_sensor;

  //cartesian velocity of body
  Eigen::Vector3d v;

  //rotational velocity of body
  Eigen::Vector3d w;

  //fstar of current link
  Eigen::Vector6d fstar;

  Eigen::Matrix6Vd Jac_point;
  Eigen::Matrix6Vd Jac;
  Eigen::Matrix6Vd Jac_COM;
  Eigen::Matrix3Vd Jac_COM_p;
  Eigen::Matrix3Vd Jac_COM_r;
  Eigen::Matrix6Vd Jac_Contact;

  //realtime traj of cartesian & orientation.
  //)) traj is outcome of cubic or quintic function, which will be used to make fstar!
  // x : cartesian coordinate traj (3x1)
  // v : cartesian velocity (3x1)
  // r : rotational matrix of current orientation (3x3)
  // w : rotational speed of current orientation (3x1)

  Eigen::Vector3d x_traj;
  Eigen::Vector3d v_traj;
  Eigen::Vector3d a_traj;

  Eigen::Matrix3d r_traj;
  Eigen::Vector3d w_traj;

  Eigen::Vector3d x_init;
  Eigen::Vector3d v_init;
  Eigen::Matrix3d rot_init;

  Eigen::Vector3d x_desired;
  Eigen::Matrix3d rot_desired;

  Eigen::Vector3d pos_p_gain;
  Eigen::Vector3d pos_d_gain;
  Eigen::Vector3d rot_p_gain;
  Eigen::Vector3d rot_d_gain;

private:
  Eigen::MatrixXd j_temp;
  Eigen::MatrixXd j_temp2;
  RigidBodyDynamics::Model *model;
};

class EndEffector
{
public:
  Eigen::Vector3d cp_;
  Eigen::Vector3d xpos;
  Eigen::Vector3d sensor_xpos;
  Eigen::Matrix3d rotm;
  double cs_x_length;
  double cs_y_length;
  bool contact = false;
};

class Position
{
public:
  double x, y;
  double angle;
  void rotate(double angle);
};

Eigen::Vector2d local2global(double x, double y, double angle);

class RobotData
{
public:
  Com com_;
  Link link_[LINK_NUMBER + 1];
  double orientation;
  double yaw_radian;
  double roll, pitch, yaw;

  Eigen::VectorQd q_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;

  Eigen::VectorXd ContactForce;
  Eigen::Vector12d ContactForce_FT;
  Eigen::Vector6d LH_FT, RH_FT;
  Eigen::Vector3d ZMP;
  Eigen::Vector3d ZMP_local;
  Eigen::Vector3d ZMP_desired;
  Eigen::Vector3d ZMP_desired2;
  Eigen::Vector3d ZMP_ft;
  Eigen::Vector3d ZMP_error;
  Eigen::Vector3d ZMP_eqn_calc;
  Eigen::Vector3d ZMP_command;
  Eigen::Vector3d ZMP_mod;

  bool zmp_feedback_control = false;
  bool check = false;

  Eigen::Vector3d fstar;

  //bool contact_[ENDEFFECTOR_NUMBER] = {true, true};

  //ee_ : Left to Right
  EndEffector ee_[ENDEFFECTOR_NUMBER]; //ee_ : 0: Left 1: Right

  int contact_index;
  int contact_part[4];
  int ee_idx[4];

  double control_time_; // updated by control_base
  double d_time_;

  double start_time_[4];
  double end_time_[4];
  bool target_arrived_[4];
  bool debug;

  int Right = 0;
  int Left = 1;


  Eigen::MatrixXd task_selection_matrix;
  Eigen::VectorXd task_desired_force;
  Eigen::VectorXd task_feedback_reference;
  Eigen::Vector2d ZMP_task;

  double zmp_gain;

  Eigen::MatrixVVd A_matrix;
  Eigen::MatrixVVd A_;
  Eigen::MatrixVVd A_matrix_inverse;

  Eigen::MatrixXd J_C, J_C_INV_T;
  Eigen::MatrixXd J_COM;

  Eigen::MatrixXd J_task;
  Eigen::VectorXd f_star;

  Eigen::MatrixXd Lambda_c;
  Eigen::MatrixXd N_C;
  Eigen::MatrixVVd I37;

  Eigen::VectorXd contact_force_predict;
  Eigen::Vector3d Grav_ref;

  Eigen::MatrixXd J_task_T, J_task_inv, J_task_inv_T;
  Eigen::MatrixXd lambda_inv, lambda;
  Eigen::MatrixXd W, W_inv;
  Eigen::MatrixXd Q, Q_T_, Q_temp, Q_temp_inv, Jtemp, Jtemp_2;
  Eigen::MatrixXd _F;

  Eigen::VectorXd G;

  Eigen::MatrixXd Slc_k, Slc_k_T;
  Eigen::MatrixXd svd_U;

  int task_dof;

  Eigen::Vector2d p_k_1;
  Eigen::Vector3d ZMP_pos;

  double fc_redis;

  bool contact_calc;
  bool task_force_control;
  bool task_force_control_feedback;
  bool zmp_control;
  bool mpc_init;
};

std::ostream &
operator<<(std::ostream &out, const Link &link);
#endif