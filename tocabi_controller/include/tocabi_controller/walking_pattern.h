#ifndef WALKINGPATTERN_H
#define WALKINGPATTERN_H

#include "math_type_define.h"

class WalkingPattern
{
public:

    void footStepGenerator(Eigen::Vector4d target, Eigen::Vector3d foot_distance, int desired_foot_step_num, int first_foot_step_dir);
    void foot_step_total(Eigen::Vector4d target, Eigen::Vector3d foot_distance, int first_foot_step_dir);
    void cpPatternGeneration();
    void setCpPosition();

public:
    //////Foot Step//////
    Eigen::MatrixXd foot_step;
    int total_step_num;

    //////Capture Point//////
    Eigen::VectorXd capturePoint_ox;
    Eigen::VectorXd capturePoint_oy;
    Eigen::VectorXd capturePoint_offsetx;
    Eigen::VectorXd capturePoint_offsety;
    Eigen::VectorXd zmp_dx;
    Eigen::VectorXd zmp_dy;

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
    Eigen::Isometry3d PELV_float_current;
    Eigen::Isometry3d PELV_float_init;
    Eigen::Isometry3d COM_float_current;
    Eigen::Isometry3d COM_float_init;
    Eigen::Isometry3d SUF_float_current;
    Eigen::Isometry3d SUF_float_init;
    Eigen::Isometry3d SWF_float_current;
    Eigen::Isometry3d SWF_float_init;
    Eigen::Isometry3d PELV_support_current;
    Eigen::Isometry3d PELV_support_init;
    Eigen::Isometry3d COM_support_current;
    Eigen::Isometry3d COM_support_init;
    Eigen::Vector3d foot_distance;
    double zc;
    double lipm_w;

    Eigen::VectorQd q_init;

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
    int current_step_num; // temp

    // Walking
    int walking_tick;
};


#endif