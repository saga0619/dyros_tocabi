#ifndef WALKINGPATTERN_H
#define WALKINGPATTERN_H

#include "tocabi_controller/data_container.h"
#include <fstream>
#include "math_type_define.h"

class WalkingPattern
{
public:

    void footStepGenerator();
    void footStepTotal();
    void chagneFootSteptoLocal();
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
    int Hz_;
    double dt;
    
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
    Eigen::VectorXd zmp_refx;
    Eigen::VectorXd zmp_refy;
    Eigen::VectorXd b;

    // Robot Model
    // LEFT FOOT : LF, RIGHT FOOT : RF, SUPPORT FOOT : SUF, SWING FOOT : SWF
    Eigen::Isometry3d RF_float_current;
    Eigen::Isometry3d LF_float_current;
    Eigen::Isometry3d RF_float_init;
    Eigen::Isometry3d LF_float_init;
    Eigen::Isometry3d RF_support_current;
    Eigen::Isometry3d LF_support_current;
    Eigen::Isometry3d RF_support_init;
    Eigen::Isometry3d LF_support_init;
    Eigen::Isometry3d LF_trajectory_float;
    Eigen::Isometry3d RF_trajectory_float;
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
    Eigen::Isometry3d COM_support_current;
    Eigen::Isometry3d COM_support_init;
 
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

    Eigen::Vector3d foot_distance;
    double zc;
    double lipm_w;

    Eigen::VectorQd q_init;

    //Reference Frame Transform
    Eigen::Isometry3d LocaltoGlobal_current;
    Eigen::Isometry3d GlobaltoLocal_current;

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
    double t_imp;
    double foot_height;
    int current_step_num; // temp

    // Walking
    int walking_tick;



    //Ui WalkingParameter
    int ik_mode;
    int walking_pattern;
    int foot_step_dir;
    double height;
    double step_length_x;
    double step_length_y;
    bool dob;
    Eigen::Vector4d target;
    bool com_control_mode;
    bool gyro_frame_flag;
    double com_gain;
    double pelvis_pgain;
    double pelvis_dgain;
    double pelvis_offsetx;


    std::fstream file[1];
};


const std::string FILE_NAMES[1] =
{
  ///change this directory when you use this code on the other computer///
  "/home/jhk/data/walking/0_tocabi_.txt",
};
#endif