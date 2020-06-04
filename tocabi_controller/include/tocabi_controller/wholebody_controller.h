#ifndef WHOLEBODY_CONTROLLER_H
#define WHOLEBODY_CONTROLLER_H

//#include "dyros_red_controller/dyros_red_model.h"
#include "tocabi_controller/link.h"
#include "tocabi_controller/redsvd.h"
#include "tocabi_controller/qp.h"
#include <qpOASES.hpp>

using namespace Eigen;
using namespace std;
using namespace qpOASES;

/* Wholebody controller description :
* This library is made by junewhee ahn, based on theory by jaeheung park
* 
*
* task control how to use : 
* 1. set contact status with function 'Wholebody_controller::contact_set_multi(...)'
* 2. get gravity compensation torque with 'Wholebody_controller::gravity_compensation_torque(...)'
* 3. define task dof number
* 4. construct task jacobian 
* 5. define desired task position and time,
* 6. get trajectory of task with function 'link_.Set_Trajectory_from_quintic(....)' or other set_traj functions at link class!
*    initial position of each task can be determined through following function, link_[].Set_initpos()
* 7. get fstar with Wholebody_controller::getfstar6d(...)
* 8. get torque task with Wholebody_controller::task_control_torque(....)



*/

class WholebodyController
{
public:
  //const VectorQd &current_q_;
  //Main loop wholebody function
  // update kinematics information
  //
  void init(RobotData &Robot);
  void update(RobotData &Robot);

  //set contact status of robot. true for contact false for not contact
  void set_contact(RobotData &Robot);
  void set_contact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand = false, bool right_hand = false);

  //contact force redistribution by yisoolee method at 2 contact(both foot)
  VectorQd contact_force_redistribution_torque(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta);

  VectorQd contact_force_redistribution_torque_walking(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta, double ratio, int supportFoot);

  //set contact force to desired contact force
  VectorQd contact_force_custom(RobotData &Robot, VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired);

  //update gravity compensation torque
  VectorQd gravity_compensation_torque(RobotData &Robot, bool fixed = false, bool redsvd = false);

  //get contact redistribution torque with Quadratic programing
  VectorQd contact_torque_calc_from_QP(RobotData &Robot, VectorQd command_torque);

  // Get Contact Redistribution Torque with QP. Wall contact mode.
  //VectorQd contact_torque_calc_from_QP_wall(VectorQd command_torque, double wall_friction_ratio);
  //Get Contact Redistribution Torque with QP. Wall contact mode.
  //VectorQd contact_torque_calc_from_QP_wall_mod2(VectorQd command_torque, double wall_friction_ratio);

  /*
  * Get Task Control Torque.
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);

  VectorQd task_control_torque_with_gravity(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);

  VectorQd task_control_torque(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_, int mode);

  

  VectorQd task_control_torque_motor(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  /*
  * Get Task Control Torque from QP.
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque_QP(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  VectorQd task_control_torque_QP2(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  VectorQd task_control_torque_QP_dg(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_, int contact_dist_ratio);
  VectorQd task_control_torque_QP_gravity(RobotData &Robot);
  VectorXd check_fstar(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  /*
  * Get Task Control Torque 
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque_custom_force(RobotData &Robot, MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force);

  // Get Task Control Torque task jacobian and f_star must be defined.
  VectorQd task_control_torque_custom_force_feedback(RobotData &Robot, MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand);

  //force control with selection matrix. selec 1 for control with fstar 0 for force control
  void set_force_control(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force);

  //force control selection matrix 1 for control with fstar 0 for force control
  void set_force_control_feedback(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand);
  void set_zmp_control(RobotData &Robot, Vector2d ZMP, double gain);
  void set_zmp_feedback_control(RobotData &Data, Vector2d ZMP, bool &reset_error);
  void zmp_feedback_control(Vector3d desired_zmp);

  //Utility functions

  //Get contact force from command torque
  VectorXd get_contact_force(RobotData &Robot, VectorQd command_torque);

  //Get ZMP position from contact forces and both foot position
  Vector3d GetZMPpos(RobotData &Robot, bool Local = false);
  Vector3d GetZMPpos_fromFT(RobotData &Robot, bool Local = false);
  Vector3d GetZMPpos(RobotData &Robot, VectorXd ContactForce, bool Local = false);

  //Eigen::Vector6d Getfstar( );
  Vector3d getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now);
  Vector3d getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now);
  Vector3d getfstar_tra(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt);
  Vector3d getfstar_tra(RobotData &Robot, int link_id);
  Vector3d getfstar_rot(RobotData &Robot, int link_id, Vector3d kpa, Vector3d kda);
  Vector3d getfstar_rot(RobotData &Robot, int link_id);
  Vector6d getfstar6d(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda);
  Vector6d getfstar6d(RobotData &Robot, int link_id);

  VectorQd get_joint_acceleration(RobotData &Robot, VectorQd commnad_torque);

  Vector3d COM_traj_with_zmp(RobotData &Robot);

  //zmp controller
  VectorQd CP_control_init(RobotData &Robot, double dT);
  VectorQd CP_controller();
  Vector6d zmp_controller(RobotData &Robot, Vector2d ZMP, double height);
  Vector2d CP_ref[20];

  //Vector2d getcptraj(double time, Vector2d zmp);

  Vector2d getcpref(RobotData &Robot, double task_time, double future_time);
  //Contact Mode
  const int DOUBLE_SUPPORT = 0;
  const int SINGLE_SUPPORT_LEFT = 1;
  const int SINGLE_SUPPORT_RIGHT = 2;
  const int TRIPPLE_SUPPORT = 3;
  const int QUAD_SUPPORT = 4;


  void CalcAMatrix(RobotData &Robot, MatrixXd &A_matrix);
  /*

  // motion time
  //const double hz_;
  double control_time_; // updated by control_base
  double d_time_;
  double yaw_radian;

  double start_time_[4];
  double end_time_[4];
  bool target_arrived_[4];
  bool debug;

  int Right = 0;
  int Left = 1;

  bool contact_calc = false;

  VectorQVQd current_q_;

  MatrixXd task_selection_matrix;
  VectorXd task_desired_force;
  VectorXd task_feedback_reference;
  Vector2d ZMP_task;
  bool task_force_control = false;
  bool task_force_control_feedback = false;
  bool zmp_control = false;
  double zmp_gain;
  bool mpc_init = false;
  
  MatrixVVd A_matrix;
  MatrixVVd A_matrix_inverse;

  MatrixXd J_C, J_C_INV_T;
  MatrixXd J_COM;

  MatrixXd J_task;
  VectorXd f_star;

  MatrixXd Lambda_c;
  MatrixXd N_C;
  MatrixVVd I37;

  VectorXd contact_force_predict;
  Vector3d Grav_ref;

  MatrixXd J_task_T, J_task_inv, J_task_inv_T;
  MatrixXd lambda_inv, lambda;
  MatrixXd W, W_inv;
  MatrixXd Q, Q_T_, Q_temp, Q_temp_inv, Jtemp, Jtemp_2;
  MatrixXd _F;

  VectorXd G;

  MatrixXd Slc_k, Slc_k_T;
  MatrixXd svd_U;

  int task_dof;

  Vector2d p_k_1;
  Vector3d ZMP_pos;

  double fc_redis;*/

  //QP solver setting
  void QPInitialize();
  void QPReset();
  int nIter;
  CQuadraticProgram QP_test;
  CQuadraticProgram QP_mpc;
  CQuadraticProgram QP_torque;
  VectorXd result_temp;

private:
  //update contact space dynamics
  //void contact_set(int contact_number, int link_id[]);
  void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
  void ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
};

#endif // WALKING_CONTROLLER_H