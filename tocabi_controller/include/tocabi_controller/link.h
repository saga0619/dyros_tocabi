#ifndef LINK_H
#define LINK_H

#include "Eigen/Dense"
#include "math_type_define.h"
#include <rbdl/rbdl.h>
#include "tocabi_controller/tocabi.h"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <mutex>
#include <future>
#include <atomic>

extern std::mutex mtx_rbdl;

struct VelCommand
{
  int link_;
  Eigen::Vector6d des_vel;

  bool used_;
};

struct TaskCommand
{
  double command_time;
  double traj_time;
  bool task_init;
  int mode;
  // COM Related
  double ratio;
  double height;
  double pelv_pitch;
  double roll;
  double pitch;
  double yaw;
  // Arm Related
  double l_x;
  double l_y;
  double l_z;
  double l_roll;
  double l_pitch;
  double l_yaw;
  double r_x;
  double r_y;
  double r_z;
  double r_roll;
  double r_pitch;
  double r_yaw;

  bool maintain_last_control;

  int solver;
  int contactredis;

  double init_com_height;

  //Walking Related
  int walking_enable;
  int ik_mode;
  int walking_pattern;
  int walking_pattern2;
  int vibration_control;
  int foot_step_dir;
  double target_x;
  double target_y;
  double target_z;
  double theta;
  double walking_height;
  double step_length_x;
  double step_length_y;
  bool dob;
  bool mom;
  bool imu_walk;

  bool link_contact[4];

  //taskgain
  bool custom_taskgain;
  double pos_p;
  double pos_d;
  double ang_p;
  double ang_d;
  double acc_p;
};

class TQue
{
public:
  bool update;
  //std::string text;
  char text[256];
};

struct Com
{
  double mass;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d angular_momentum;
  Eigen::Vector3d angular_moment;
  Eigen::Vector2d ZMP;
  Eigen::Vector2d CP;
  Eigen::Matrix6Vd Jac;
};

class Link
{
public:
  // Update link i of rbdl link id. name : link name, mass : link mass, xipos : local center of mass position
  void initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass, Eigen::Vector3d &local_com_position);

  // Update COM jacobian
  void COM_Jac_Update(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_);

  // Update xpos, xipos, rotm.
  void pos_Update(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_);

  // Set Contact point, Contact jacobian
  void Set_Contact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // Set Contact point, Contact jacobian
  void Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // Set Contact point, Contact jacobian
  void Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &Contact_position);

  // Set Sensor Position
  void Set_Sensor_Position(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Sensor_position);

  // update Jacobian matrix of local position at link.
  void Set_Jacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position);

  // update Point Jacobian matrix of local position at link.
  void Set_Jacobian_custom(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position);

  // update link velocity(6D, translation and rotation) from jacobian matrix Jac.
  void vw_Update(const Eigen::VectorVQd &q_dot_virtual);

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

  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d acc_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired, Eigen::Vector3d acc_des);

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

  void Set_initpos_local();

  void Set_initTask();

  bool Check_name(RigidBodyDynamics::Model &model_);

  void Get_PointPos(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos, Eigen::Vector6d &global_velocity6D);

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

  //cartesian velocity of contact point at body
  Eigen::Vector3d v_contact;

  //cartesian velocity of contact point at body
  Eigen::Vector3d w_contact;

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
  Eigen::Vector3d ra_traj;

  Eigen::Vector3d x_traj_local;
  Eigen::Vector3d v_traj_local;
  Eigen::Matrix3d r_traj_local;
  Eigen::Vector3d w_traj_local;

  Eigen::Vector3d x_init;
  Eigen::Vector3d v_init;
  Eigen::Matrix3d rot_init;
  Eigen::Vector3d w_init;

  Eigen::Vector3d x_init_local;
  Eigen::Vector3d v_init_local;
  Eigen::Matrix3d rot_init_local;
  Eigen::Vector3d w_init_local;

  Eigen::Vector3d x_task_init;
  Eigen::Vector3d v_task_init;
  Eigen::Vector3d a_task_init;
  Eigen::Matrix3d r_task_init;
  Eigen::Vector3d w_task_init;

  Eigen::Vector3d x_desired;
  Eigen::Matrix3d rot_desired;

  Eigen::Vector3d pos_p_gain;
  Eigen::Vector3d pos_d_gain;
  Eigen::Vector3d rot_p_gain;
  Eigen::Vector3d rot_d_gain;
  Eigen::Vector3d acc_p_gain;

  Eigen::Vector3d max_p_acc_;
  Eigen::Vector3d max_p_vel_;

  RigidBodyDynamics::Model *model;

private:
  Eigen::MatrixXd j_temp;
};

struct EndEffector
{
  Eigen::Vector3d cp_;
  Eigen::Vector3d xpos;
  Eigen::Vector3d sensor_xpos;
  Eigen::Vector6d contact_force;
  Eigen::Vector6d contact_force_ft;
  Eigen::Matrix3d rotm;
  Eigen::Vector3d zmp;
  double contact_accuracy;
  double contact_time;
  int contact_transition_mode; //-1 nothing to do, 0 disabling, 1 enabling
  double minimum_press_force;
  double friction_ratio;
  double friction_ratio_z;
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

struct RobotData_fast
{

  double control_time_;
  Eigen::MatrixVVd A_matrix_inverse;
  Eigen::MatrixXd W_inv;
  Eigen::MatrixXd N_C;
  Eigen::MatrixXd P_C;
  Eigen::VectorQd torque_grav;
  Eigen::MatrixXd NwJw;
  Eigen::MatrixXd J_C_INV_T;
  Eigen::MatrixXd qr_V2;

  double com_time;

  Eigen::VectorQd q_dot_;

  EndEffector ee_[ENDEFFECTOR_NUMBER];
  
  int ee_idx[4];

  std::vector<Eigen::VectorXd> fstar_hqp;
  std::vector<Eigen::MatrixXd> Jtask_hqp;

  int contact_index;

  Eigen::VectorQd torque_control;

};

struct RobotData
{
  Com com_;
  Link link_[LINK_NUMBER + 1];
  double orientation;
  double roll, pitch, yaw;
  double yaw_init = 0.0;
  double total_mass;

  //PositionPDGain
  double Kps[MODEL_DOF];
  double Kvs[MODEL_DOF];
  std::vector<double> vector_kp, vector_kv, vector_NM2CNT;
  std::vector<double> v_com_kp, v_com_kv, v_up_kp, v_up_kv, v_pelv_kp, v_pelv_kv, v_hand_kp, v_hand_kv, v_foot_kp, v_foot_kv;

  Eigen::VectorQd q_desired_;
  Eigen::VectorQd q_dot_desired_;
  Eigen::VectorQd q_;
  Eigen::VectorQd q_init_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorQd q_dot_est;
  Eigen::VectorQd q_dot_est1;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorVQd q_dot_virtual_lpf_;
  Eigen::VectorQd q_ext_;

  Eigen::VectorQd q_dot_before_;
  Eigen::VectorQd q_dot_diff_;

  Eigen::VectorQd q_ddot_estimate_;

  Eigen::VectorXd ContactForce;
  Eigen::VectorXd ContactForce_qp;
  Eigen::VectorXd torque_qp;
  Eigen::VectorXd qacc_qp;
  Eigen::Vector12d ContactForce_FT;
  Eigen::Vector12d ContactForce_FT_raw;
  Eigen::Vector12d CF_temp;
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
  Eigen::Vector3d ZMP_r;
  Eigen::Vector3d ZMP_l;
  Eigen::Vector3d CP_;
  Eigen::Vector3d CP_desired;
  Eigen::VectorXd TaskForce;

  double com_time;

  std::future<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> winv_ret;

  bool zmp_feedback_control = false;
  bool check = false;
  bool qp2nd = false;
  bool signal_yaw_init = false;
  Eigen::Vector3d fstar;

  double com_vel_limit;
  double com_acc_limit;

  //bool contact_[ENDEFFECTOR_NUMBER] = {true, true};

  //ee_ : Left to Right
  EndEffector ee_[ENDEFFECTOR_NUMBER]; //ee_ : 0: Left 1: Right

  int contact_index;
  int contact_part[4];
  int ee_idx[4];

  double contact_transition_time;

  double control_time_; // updated by control_base
  double control_time_pre_;
  double d_time_;

  double friction_ratio;

  bool data_print_switch = false;

  bool data_print = false;

  double start_time_[4];
  double end_time_[4];
  bool target_arrived_[4];
  bool debug;
  bool lambda_calc = false;
  bool init_qp = false;
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
  Eigen::Matrix6Qd Ag_;
  Eigen::MatrixQQd Cor_;
  Eigen::MatrixQQd M_p;
  Eigen::VectorQd G_;

  Eigen::MatrixVVd Motor_inertia;
  Eigen::MatrixVVd Motor_inertia_inverse;
  Eigen::MatrixXd Lambda_c_motor;
  Eigen::MatrixXd J_task_inv_motor, J_task_inv_motor_T;
  Eigen::MatrixXd lambda_motor_inv, lambda_motor;
  Eigen::MatrixXd W_motor, W_motor_inv;
  Eigen::MatrixXd N_C_motor;
  Eigen::MatrixXd Q_motor, Q_motor_T_, Q_motor_temp, Q_motor_temp_inv; //, Jtemp, Jtemp_2;

  Eigen::MatrixXd J_C, J_C_INV_T;
  Eigen::MatrixXd J_COM;
  Eigen::Matrix<double, MODEL_DOF,MODEL_DOF_VIRTUAL> J_g;

  Eigen::MatrixXd J_task;
  Eigen::VectorXd f_star;
  Eigen::VectorXd f_star_a_;

  Eigen::MatrixXd Lambda_c;
  Eigen::MatrixXd N_C;
  Eigen::VectorXd P_C;
  Eigen::MatrixVVd I37;

  Eigen::VectorXd contact_force_predict;
  Eigen::Vector3d Grav_ref;

  Eigen::MatrixXd J_task_T, J_task_inv, J_task_inv_T;
  Eigen::MatrixXd lambda_inv, lambda;
  Eigen::MatrixXd W, W_inv;
  Eigen::MatrixXd Q, Q_T_, Q_temp, Q_temp_inv, Jtemp, Jtemp_2;
  Eigen::MatrixXd _F;

  Eigen::VectorXd G;
  Eigen::VectorQd torque_grav_cc;
  Eigen::VectorQd torque_grav;
  Eigen::VectorQd torque_contact;
  Eigen::VectorQd torque_disturbance;
  Eigen::VectorQd torque_limit;

  Eigen::MatrixXd Slc_k, Slc_k_T;
  Eigen::MatrixXd svd_U;
  Eigen::MatrixXd qr_V2;
  Eigen::MatrixXd NwJw;
  Eigen::MatrixXd Scf_;

  bool qp_error = false;

  int task_dof;

  Eigen::Vector2d p_k_1;
  Eigen::Vector3d ZMP_pos;

  Eigen::Vector3d imu_pos_;
  Eigen::Vector3d imu_vel_;

  //contact redistribution mode selector. 0 : yslee 1: qp 2: off
  int contact_redistribution_mode = 0;

  double fc_redis;

  bool contact_calc;
  bool task_force_control;
  bool task_force_control_feedback;
  bool zmp_control;
  bool mpc_init;
  bool showdata;
  bool task_control_switch = false;

  RigidBodyDynamics::Model model_virtual;
};

std::ostream &
operator<<(std::ostream &out, const Link &link);
#endif
