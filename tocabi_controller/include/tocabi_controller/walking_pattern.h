#ifndef WALKINGPATTERN_H
#define WALKINGPATTERN_H

#include <fstream>
#include "math_type_define.h"

/* 
 Walking controller algorithm is made by Junhyung Kim
 PELVIS and FOOT traejctory is based on Pelvis Frame
*/

class WalkingPattern
{
public:
    void footStepPlanning();
    void footStepGenerator();
    void footStepTotal();
    void changeFootSteptoLocal();
    void cpReferencePatternGeneration();
    void setCpPosition();
    void cptoComTrajectory();
    void setComTrajectory();
    void setPelvisTrajectory();
    void setFootTrajectory();
    void supportToFloatPattern();
    void floatToSupportPattern();
    void referenceFrameChange();

public:
    double Hz_;
    double dt;

    //User WalkingParameter
    int desired_foot_step_num;
    int t_rest_init;
    int t_rest_last;
    int t_double1;
    int t_double2;
    int t_total;
    int t_temp;
    int t_last;
    int t_start;
    int t_start_real;
    int t_rest_temp;
    int com_control;
    double t_imp;
    double foot_height;
    int current_step_num; // temp

    //////Foot Step//////
    Eigen::MatrixXd foot_step;
    Eigen::MatrixXd foot_step_support;
    Eigen::MatrixXd foot_step_supportOffset;
    int total_step_num;

    //////Walking //////
    Eigen::Vector3d zmp_desired;
    Eigen::Vector3d com_desired;
    Eigen::Vector3d com_dot_desired;
    Eigen::Vector3d p_ref;
    Eigen::Vector3d l_ref;

    //////Capture Point//////
    Eigen::VectorXd capturePoint_ox;
    Eigen::VectorXd capturePoint_oy;
    Eigen::VectorXd capturePoint_offsetx;
    Eigen::VectorXd capturePoint_offsety;
    Eigen::VectorXd capturePoint_refx;
    Eigen::VectorXd capturePoint_refy;
    Eigen::VectorXd zmp_dx;
    Eigen::VectorXd zmp_dy;
    Eigen::VectorXd com_refx;
    Eigen::VectorXd com_refy;
    Eigen::VectorXd com_refdx;
    Eigen::VectorXd com_refdy;
    Eigen::VectorXd zmp_refx;
    Eigen::VectorXd zmp_refy;
    Eigen::VectorXd b;

    // Robot Model
    // LEFT FOOT : LF, RIGHT FOOT : RF, SUPPORT FOOT : SUF, SWING FOOT : SWF
    Eigen::Isometry3d RF_float_current;
    Eigen::Isometry3d LF_float_current;
    Eigen::Isometry3d RF_float_init;
    Eigen::Isometry3d RFx_float_init;
    Eigen::Isometry3d LF_float_init;
    Eigen::Isometry3d RF_support_current;
    Eigen::Isometry3d LF_support_current;
    Eigen::Isometry3d RF_support_init;
    Eigen::Isometry3d LF_support_init;
    Eigen::Isometry3d LF_trajectory_float;
    Eigen::Isometry3d RF_trajectory_float;
    Eigen::Isometry3d LFD_trajectory_float;
    Eigen::Isometry3d RFD_trajectory_float;
    Eigen::Isometry3d LF_trajectory_support;
    Eigen::Isometry3d RF_trajectory_support;
    Eigen::Isometry3d PELV_float_current;
    Eigen::Isometry3d PELV_float_init;
    Eigen::Isometry3d PELV_trajectory_support;
    Eigen::Isometry3d COM_float_current;
    Eigen::Isometry3d COM_float_init;
    Eigen::Isometry3d SUF_float_current;
    Eigen::Isometry3d SUF_float_init;
    Eigen::Isometry3d SWF_float_current;
    Eigen::Isometry3d SWF_float_init;
    Eigen::Isometry3d PELV_support_current;
    Eigen::Isometry3d PELV_support_init;
    Eigen::Isometry3d PELV_trajectory_float;
    Eigen::Isometry3d PELVD_trajectory_float;
    Eigen::Isometry3d COM_support_current;
    Eigen::Isometry3d COM_support_init;

    Eigen::Isometry3d RF_fisrt_init;
    Eigen::Isometry3d LF_fisrt_init;
    Eigen::Isometry3d PELV_first_init;

    Eigen::Vector6d SUF_float_initV;
    Eigen::Vector6d SWF_float_initV;
    Eigen::Vector6d SUF_float_currentV;
    Eigen::Vector6d SWF_float_currentV;
    Eigen::Vector6d SUF_support_initV;
    Eigen::Vector6d SWF_target;
    Eigen::Vector6d SWF_support_initV;
    Eigen::Vector6d LF_trajectory_dot_support;
    Eigen::Vector6d RF_trajectory_dot_support;

    Eigen::Vector3d COMV_support_currentV;
    Eigen::Vector3d COMA_support_currentV;
    Eigen::Vector3d PELV_trajectory_euler;
    Eigen::Vector3d PELV_support_euler_init;
    Eigen::Vector3d LF_support_euler_init;
    Eigen::Vector3d RF_support_euler_init;
    Eigen::Vector3d LF_trajectory_euler_float;
    Eigen::Vector3d RF_trajectory_euler_float;
    Eigen::Vector3d LF_trajectory_euler_support;
    Eigen::Vector3d RF_trajectory_euler_support;
    Eigen::Vector3d SWF_SUF_ori;
    Eigen::Vector3d RF_LFe;
    Eigen::Vector3d LF_RFe;

    Eigen::Vector4d PELV_firstinit;

    Eigen::Vector3d foot_distance;
    double zc;
    double lipm_w;

    Eigen::VectorQd q_init;
    Eigen::VectorQd q_target;

    //Reference Frame Transform
    Eigen::Isometry3d LocaltoGlobal_current;
    Eigen::Isometry3d GlobaltoLocal_current;
    Eigen::Isometry3d FoottoGlobal_current;
    Eigen::Isometry3d FoottoGlobal_current_init;
    Eigen::Isometry3d Com_measured;
    Eigen::Isometry3d RF_measured;
    Eigen::Isometry3d LF_measured;
    Eigen::Isometry3d Framereference;
    Eigen::Isometry3d Debug_Iso;

    //Com Jacobian
    Eigen::Vector3d r_c1;
    Eigen::Matrix3d r_c1_skew;
    Eigen::MatrixXd J;
    Eigen::Matrix6d J_l;
    Eigen::Matrix6d J_r;
    Eigen::Matrix6d J_lc;
    Eigen::Matrix6d J_rc;
    Eigen::Matrix6d X21;
    Eigen::Vector6d Xdot;
    Eigen::Vector6d LFDotTraj;
    Eigen::Vector6d LFDotPrevTraj;
    Eigen::Vector6d RFDotTraj;
    Eigen::Vector6d RFDotPrevTraj;
    Eigen::Vector6d COMDotTraj;
    Eigen::Vector6d SFerr;
    Eigen::Vector3d Cfsemd;
    Eigen::Matrix<double, 3, 6> Jfsem;

    //MomentumControl
    Eigen::VectorXd q_w;
    Eigen::Vector3d H_leg;
    Eigen::Vector3d H_legr;
    Eigen::Vector3d H_legl;
    Eigen::Vector3d Hl_leg;
    Eigen::Vector3d Hd_leg;
    Eigen::Vector3d Td_leg;
    Eigen::Vector3d Hd_leg_prev;
    Eigen::VectorXd q_dm;
    Eigen::Matrix3x12d Ag_leg;
    Eigen::Matrix3x8d Ag_armR;
    Eigen::Matrix3x8d Ag_armL;
    Eigen::Matrix3x3d Ag_waist;
    Eigen::Matrix3x12d Agl_leg;
    Eigen::Matrix3x8d Agl_armR;
    Eigen::Matrix3x8d Agl_armL;
    Eigen::Matrix3x3d Agl_waist;
    Eigen::Vector5d qd_prev;

    Eigen::Vector3d CM_momentLeg_;
    Eigen::Vector3d CM_momentumLeg_prev;
    Eigen::Vector3d CM_momentumLeg_prev1;
    Eigen::Vector3d CM_momentLeg_lpf;
    Eigen::Vector3d CM_momentUpper_;
    Eigen::Vector3d CM_momentumUpperd_;
    Eigen::Vector3d CM_momentLeg_prev;  
    Eigen::Vector3d com_accref;
    Eigen::Vector3d comFcur;
    Eigen::Vector3d comFref;
    Eigen::Vector3d comFerr;   

    //vibrationcontrol
    Eigen::Matrix2d Ax_vib;
    Eigen::Vector2d Bx_vib;
    Eigen::MatrixXd Cx_vib;
    Eigen::Vector3d Dx_vib;
    Eigen::Vector2d kx_vib;
    Eigen::Vector2d cx_vib;
    Eigen::Vector2d xx_vib;
    Eigen::Vector2d xx_vib_est;
    Eigen::Vector3d yx_vib;
    Eigen::Vector3d yx_vibm;
    Eigen::Vector2d final_posx;

    Eigen::Matrix2d Ay_vib;
    Eigen::Vector2d By_vib;
    Eigen::MatrixXd Cy_vib;
    Eigen::Vector3d Dy_vib;
    Eigen::Vector2d ky_vib;
    Eigen::Vector2d cy_vib;
    Eigen::Vector2d xy_vib;
    Eigen::Vector2d xy_vib_est;
    Eigen::Vector3d yy_vib;
    Eigen::Vector3d yy_vibm;
    Eigen::Vector2d final_posy;
    Eigen::MatrixXd L1, L2;

    Eigen::Isometry3d HLR_float_init;
    Eigen::Isometry3d HRR_float_init;
    Eigen::Isometry3d PELV_float_init1;

    double ux_vib;
    double uy_vib;

    double m;
    bool vib_est;

    // Walking
    int walking_tick;
    int contactMode;          // 0 : double, 1 : RF SWING, 2 : LF SWING
    bool phaseChange = false; // true : double, false : single
    bool phaseChange1 = false;
    double double2Single;
    double double2Single_pre;
    double single2Double;
    double single2Double_pre;
    double current_time;
    double rate;
    int time_temp;

    //Ui WalkingParameter

    int ik_mode;
    int walking_pattern;
    int foot_step_dir;
    int walking_enable;
    double height;
    double step_length_x;
    double step_length_y;
    bool dob;
    bool imu;
    bool mom;
    Eigen::Vector4d target;
    int vibration_control;
    bool com_control_mode;
    bool gyro_frame_flag;
    double com_gain;
    double pelvis_pgain;
    double pelvis_dgain;
    double pelvis_offsetx;

    Eigen::Vector3d COM;
    Eigen::Vector3d CP;

    Eigen::Vector3d com_support_temp;
    Eigen::Vector3d com_support_temp_prev;
    Eigen::Vector3d COM_prev;

    Eigen::Vector12d dob_hat;
    Eigen::Vector12d dob_hat_prev;
    Eigen::Vector12d leg_q; //temp

    Eigen::Matrix2d rot_vel; //temp
    Eigen::Matrix2d rot_prev;

    Eigen::Vector3d rf_e, lf_e, rf_e_vel, lf_e_vel;

    double dobGain;
    double debug;

    std::fstream file[2];
};
#endif
