#include "tocabi_controller/walking_controller.h"

void Walking_controller::walkingCompute(RobotData &Robot)
{
    if (walking_enable == 1.0)
    {
        //////InitModel//////
        getRobotInitState(Robot);

        /////FootStep//////
        footStepGenerator();

        /////ModelUpdate//////
        getRobotState(Robot);

        /////FrameChange//////
        changeFootSteptoLocal();
        referenceFrameChange();

        /////Capturepoint//////
        if (walking_tick == 0)
        {
            setCpPosition();
            cpReferencePatternGeneration();
            cptoComTrajectory();
        }

        /////ComTrajectory//////
        setComTrajectory();

        /////PelvisTrajectory//////
        setPelvisTrajectory();

        /////FootTrajectory//////
        setFootTrajectory();
        supportToFloatPattern();

        if(imu == 1)
        {
            ankleOriControl(Robot);
        }

        desired_leg_q_prev = desired_leg_q;
/*
        if(walking_tick == 0)
        {
            PELV_trajectory_float.linear()= Robot.link_[Pelvis].Rotm;

            PELV_trajectory_float.translation() = Robot.link_[Pelvis].xipos;

            LF_trajectory_float.linear()= Robot.link_[Left_Foot].Rotm;

            LF_trajectory_float.translation() = Robot.link_[Left_Foot].xpos;

            RF_trajectory_float.linear()= Robot.link_[Right_Foot].Rotm;

            RF_trajectory_float.translation() = Robot.link_[Right_Foot].xpos;
           
        }
*/
        if(vibration_control != 0)
        {
            comVibrationController(Robot);
        }
      
        ///InverseKinematics/////
        if(ik_mode == 0)
        {
            inverseKinematics(Robot, PELV_trajectory_float, LF_trajectory_float, RF_trajectory_float, desired_leg_q);
        }
        else
        {
            comJacobianState(Robot);
            comJacobianIK(Robot);
        }
        
        if(walking_tick == 0)
        {
            q_w(0) = desired_init_leg_q(12);
            q_w(1) = desired_init_leg_q(13);
            q_w(2) = desired_init_leg_q(14);

            q_w(3) = desired_init_leg_q(16);
            q_w(4) = desired_init_leg_q(26);
        }

        if(mom == true)
        {   
            momentumControl(Robot);

            q_w(0) = q_w(0) + q_dm(0)/Hz_;
            q_w(1) = q_w(1) + q_dm(1)/Hz_;
            q_w(2) = q_w(2) + q_dm(2)/Hz_;
            q_w(3) = q_w(3) + q_dm(3)/Hz_;
            q_w(4) = q_w(4) + q_dm(4)/Hz_;
        }

        if(dob == 1)
        {
            inverseKinematicsdob(Robot);
        }

        updateNextStepTime();
    }
    else if (walking_enable == 3.0)
    {
        setInitPose(Robot, desired_init_leg_q);
        updateInitTime();
    }
}

void Walking_controller::inverseKinematics(RobotData &Robot, Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d &leg_q)
{
    Eigen::Vector3d lp, rp;
    lp = LF_float_transform.linear().transpose() * (PELV_float_transform.translation() - LF_float_transform.translation());
    rp = RF_float_transform.linear().transpose() * (PELV_float_transform.translation() - RF_float_transform.translation());

    Eigen::Matrix3d PELF_rotation, PERF_rotation;
    PELF_rotation = PELV_float_transform.linear().transpose() * LF_float_transform.linear();
    PERF_rotation = PELV_float_transform.linear().transpose() * RF_float_transform.linear();

    Eigen::Vector3d ld, rd;
    ld.setZero();
    rd.setZero();

    ld(0) = HLR_float_init.translation()(0)-PELV_float_init.translation()(0);
    ld(1) = HLR_float_init.translation()(1)-PELV_float_init.translation()(1);
    ld(2) = -(PELV_float_init1.translation()(2)-HLR_float_init.translation()(2))+(PELV_float_init1.translation()(2)-PELV_float_init.translation()(2));

    rd(0) = HRR_float_init.translation()(0)-PELV_float_init.translation()(0);
    rd(1) = HRR_float_init.translation()(1)-PELV_float_init.translation()(1);
    rd(2) = -(PELV_float_init1.translation()(2)-HRR_float_init.translation()(2))+(PELV_float_init1.translation()(2)-PELV_float_init.translation()(2));

    ld = LF_float_transform.linear().transpose() * ld;
    rd = RF_float_transform.linear().transpose() * rd;

    Eigen::Vector3d lr, rr;
    lr = lp + ld;
    rr = rp + rd;

    double l_upper = 0.35; //direct length from hip to knee
    double l_lower = 0.35; //direct length from knee to ankle

    double offset_hip_pitch = 0.0 * DEG2RAD;
    double offset_knee_pitch = 0.0 * DEG2RAD;
    double offset_ankle_pitch = 0.0 * DEG2RAD;
    //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

    double lc = sqrt(lr(0) * lr(0) + lr(1) * lr(1) + lr(2) * lr(2));
    leg_q(3) = (-acos((l_upper * l_upper + l_lower * l_lower - lc * lc) / (2 * l_upper * l_lower)) + M_PI); // - offset_knee_pitch //+ alpha_lower

    double l_ankle_pitch = asin((l_upper * sin(M_PI - leg_q(3))) / lc);
    leg_q(4) = -atan2(lr(0), sqrt(lr(1) * lr(1) + lr(2) * lr(2))) - l_ankle_pitch; // - offset_ankle_pitch ;
    leg_q(5) = atan2(lr(1), lr(2));

    Eigen::Matrix3d r_tl2;
    Eigen::Matrix3d r_l2l3;
    Eigen::Matrix3d r_l3l4;
    Eigen::Matrix3d r_l4l5;

    r_tl2.setZero();
    r_l2l3.setZero();
    r_l3l4.setZero();
    r_l4l5.setZero();

    r_l2l3 = DyrosMath::rotateWithY(leg_q(3));
    r_l3l4 = DyrosMath::rotateWithY(leg_q(4));
    r_l4l5 = DyrosMath::rotateWithX(leg_q(5));

    r_tl2 = PELF_rotation * r_l4l5.transpose() * r_l3l4.transpose() * r_l2l3.transpose();

    leg_q(1) = asin(r_tl2(2, 1));

    double c_lq5 = -r_tl2(0, 1) / cos(leg_q(1));
    if (c_lq5 > 1.0)
    {
        c_lq5 = 1.0;
    }
    else if (c_lq5 < -1.0)
    {
        c_lq5 = -1.0;
    }

    leg_q(0) = -asin(c_lq5);
    leg_q(2) = -asin(r_tl2(2, 0) / cos(leg_q(1))) + offset_hip_pitch;
    leg_q(3) = leg_q(3) - offset_knee_pitch;
    leg_q(4) = leg_q(4) - offset_ankle_pitch;

    //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

    double rc = sqrt(rr(0) * rr(0) + rr(1) * rr(1) + rr(2) * rr(2));
    leg_q(9) = (-acos((l_upper * l_upper + l_lower * l_lower - rc * rc) / (2 * l_upper * l_lower)) + M_PI); // - offset_knee_pitch //+ alpha_lower

    double r_ankle_pitch = asin((l_upper * sin(M_PI - leg_q(9))) / rc);
    leg_q(10) = -atan2(rr(0), sqrt(rr(1) * rr(1) + rr(2) * rr(2))) - r_ankle_pitch;
    leg_q(11) = atan2(rr(1), rr(2));

    Eigen::Matrix3d r_tr2;
    Eigen::Matrix3d r_r2r3;
    Eigen::Matrix3d r_r3r4;
    Eigen::Matrix3d r_r4r5;

    r_tr2.setZero();
    r_r2r3.setZero();
    r_r3r4.setZero();
    r_r4r5.setZero();

    r_r2r3 = DyrosMath::rotateWithY(leg_q(9));
    r_r3r4 = DyrosMath::rotateWithY(leg_q(10));
    r_r4r5 = DyrosMath::rotateWithX(leg_q(11));

    r_tr2 = PERF_rotation * r_r4r5.transpose() * r_r3r4.transpose() * r_r2r3.transpose();
    leg_q(7) = asin(r_tr2(2, 1));
    double c_rq5 = -r_tr2(0, 1) / cos(leg_q(7));
    
    if (c_rq5 > 1.0)
    {
        c_rq5 = 1.0;
    }
    else if (c_rq5 < -1.0)
    {
        c_rq5 = -1.0;
    }

    leg_q(6) = -asin(c_rq5);
    leg_q(8) = asin(r_tr2(2, 0) / cos(leg_q(7))) - offset_hip_pitch;
    leg_q(9) = -leg_q(9) + offset_knee_pitch;
    leg_q(10) = -leg_q(10) + offset_ankle_pitch;

    leg_q(0) = leg_q(0) * (-1);
    leg_q(6) = leg_q(6) * (-1);
    leg_q(8) = leg_q(8) * (-1);
    leg_q(9) = leg_q(9) * (-1);
    leg_q(10) = leg_q(10) * (-1);
}

void Walking_controller::setInitPose(RobotData &Robot, Eigen::VectorQd &leg_q)
{
    if (walking_tick == 0)
    {
        Eigen::VectorQd q_temp;
        q_temp << 0.0, 0.00, -0.35, 1.0, -0.65, 0.00, 0.0, 0.00, -0.35, 1.0, -0.65, 0.00, 0.0, 0.0, 0.0, 0.2, 0.5, 1.5, -1.27, -1, 0, -1, 0, 0, 0, -0.2, -0.5, -1.5, 1.27, 1.0, 0, 1.0, 0;
       
       //q_temp.setZero();
        //q_target = Robot.q_;
        q_target = q_temp;
        walkingInitialize(Robot);
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        leg_q(i) = DyrosMath::QuinticSpline(walking_tick, 0.0, 6.0 * Hz_, q_init(i), 0.0, 0.0, q_target(i), 0.0, 0.0)(0);
    }
}

void Walking_controller::walkingInitialize(RobotData &Robot)
{
    q_init = Robot.q_;
}

void Walking_controller::getRobotState(RobotData &Robot)
{
    //////Real Robot Float Frame//////PELV_traejctory_float_id].xipos;
    COM_float_current.linear() = Robot.link_[COM_id].Rotm;

    COMV_support_currentV = Robot.com_.vel;
    COMV_support_currentV = Robot.com_.accel;

    if (foot_step(current_step_num, 6) == 0)
    {
        SUF_float_current = RF_float_current;
        SWF_float_current = LF_float_current;
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i) = SUF_float_current.translation()(i);
            SWF_float_currentV(i ) = SWF_float_current.translation()(i);
        }
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i + 3) = DyrosMath::rot2Euler(SUF_float_current.linear())(i);
            SWF_float_currentV(i + 3) = DyrosMath::rot2Euler(SWF_float_current.linear())(i);
        }
    }
    else
    {
        SUF_float_current = LF_float_current;
        SWF_float_current = RF_float_current;
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i) = SUF_float_current.translation()(i);
            SWF_float_currentV(i) = SWF_float_current.translation()(i);
        }
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i + 3) = DyrosMath::rot2Euler(SUF_float_current.linear())(i);
            SWF_float_currentV(i + 3) = DyrosMath::rot2Euler(SWF_float_current.linear())(i);
        }
    }

    //////Real Robot Support Foot Frame//////
    PELV_support_current = DyrosMath::inverseIsometry3d(SUF_float_current) * PELV_float_current;
    RF_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, RF_float_current);
    LF_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, LF_float_current);
    COM_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, COM_float_current);

    Ag_leg = Robot.Ag_.block(3, 0, 3, 12);
    Ag_armR = Robot.Ag_.block(3, 25, 3, 8);
    Ag_armL = Robot.Ag_.block(3, 15, 3, 8);
    Ag_waist = Robot.Ag_.block(3, 12, 3, 3);

    Agl_leg = Robot.Ag_.block(0, 0, 3, 12);
    Agl_armR = Robot.Ag_.block(0, 25, 3, 8);
    Agl_armL = Robot.Ag_.block(0, 15, 3, 8);
    Agl_waist = Robot.Ag_.block(0, 12, 3, 3);

    yx_vibm(0) = Robot.ZMP(0);
    yx_vibm(1) = Robot.link_[Pelvis].xipos(0);
    yx_vibm(2) = Robot.link_[Pelvis].v(0);
  
    yy_vibm(0) = Robot.ZMP(1);
    yy_vibm(1) = Robot.link_[COM_id].xipos(1);
    yy_vibm(2) = Robot.link_[COM_id].v(1);

    calcRobotState(Robot);
}


void Walking_controller::getRobotInitState(RobotData &Robot)
{
    if (walking_tick == 0)
    {
        contactMode = 1.0;

        RF_float_init.translation() = Robot.link_[Right_Foot].xpos;
        RFx_float_init.translation() = Robot.link_[Right_Foot].xipos;
        RF_float_init.linear() = Robot.link_[Right_Foot].Rotm;
        LF_float_init.translation() = Robot.link_[Left_Foot].xpos;
        LF_float_init.linear() = Robot.link_[Left_Foot].Rotm;

        COM_float_init.translation() = Robot.com_. pos;
        COM_float_init.linear() = Robot.link_[COM_id].Rotm; 

        PELV_float_init.translation() = Robot.link_[Pelvis].xipos;
        PELV_float_init.linear() = Robot.link_[Pelvis].Rotm;

        PELV_float_init1.translation() = Robot.link_[Pelvis].xpos;
        PELV_float_init1.linear() = Robot.link_[Pelvis].Rotm;

        HLR_float_init.translation() = Robot.link_[4].xpos;
        HLR_float_init.linear() = Robot.link_[4].Rotm;

        HRR_float_init.translation() = Robot.link_[10].xpos;
        HRR_float_init.linear() = Robot.link_[10].Rotm;

        PELV_first_init = PELV_float_init;
        RF_fisrt_init = RF_float_init;
        LF_fisrt_init = LF_float_init;

        Eigen::Isometry3d temp;
        temp.linear() = PELV_first_init.linear();
        temp.translation().setZero();
        foot_distance = temp.inverse() * (LF_fisrt_init.translation() - RF_fisrt_init.translation());

        if (foot_step_dir != 1)
        {
            SUF_float_init = RF_float_init;
            SWF_float_init = LF_float_init;
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i + 3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i + 3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        else
        {
            SUF_float_init = LF_float_init;
            SWF_float_init = RF_float_init;
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i + 3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i + 3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        //////Real Robot Support Foot Frame//////
        RF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), RF_float_init);
        LF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), LF_float_init);
        PELV_support_init = DyrosMath::inverseIsometry3d(SUF_float_init) * PELV_float_init;
        COM_support_init = DyrosMath::multiplyIsometry3d(PELV_support_init, COM_float_init);

        RF_support_euler_init = DyrosMath::rot2Euler(RF_support_init.linear());
        LF_support_euler_init = DyrosMath::rot2Euler(LF_support_init.linear());
        PELV_support_euler_init = DyrosMath::rot2Euler(PELV_support_init.linear());

        PELV_firstinit.head(3) = PELV_support_init.translation();
        PELV_firstinit(3, 0) = 1.0;

        zc = COM_support_init.translation()(2);

        lipm_w = sqrt(GRAVITY / zc);
    }
}

void Walking_controller::modelFrameToLocal(RobotData &Robot)
{
    Eigen::Vector4d com_temp;
    //com_temp.head(3) = R.com_.pos;
    com_temp(3) = 1.0;
}

void Walking_controller::calcRobotState(RobotData &Robot)
{
    modelFrameToLocal(Robot);

    if(walking_tick == 0)
    {
        com_support_temp = -1 * COM_support_current.translation();
        com_support_temp_prev = -1 * COM_support_current.translation();
    }

    if (walking_tick < t_start) //+t_double1)
    {
        COM = COM_support_current.translation() + com_support_temp_prev;
    }
    else
    {
        COM = COM_support_current.translation() + com_support_temp;
    }

    if (walking_tick == t_last && current_step_num <= total_step_num)
    {
        com_support_temp_prev(1) = com_support_temp(1);
        com_support_temp(0) = com_support_temp(0) + COM_support_current.translation()(0);
        com_support_temp(1) = COM_support_current.translation()(1) - COM(1);
        com_support_temp(2) = 0.0;
    }

    if (walking_tick == 0)
    {
        COM_prev = COM;
    }

    for (int i = 0; i < 3; i++)
    {
        CP(i) = COM(i) + (COM(i) - COM_prev(i)) * Hz_ / lipm_w;
    }
    COM_prev = COM;
}

void Walking_controller::setRobotStateInitialize(RobotData &Robot)
{
    RF_float_current.linear().setIdentity();
    LF_float_current.linear().setIdentity();
    RF_float_init.linear().setIdentity();
    LF_float_init.linear().setIdentity();
    RF_support_current.linear().setIdentity();
    LF_support_current.linear().setIdentity();
    RF_support_init.linear().setIdentity();
    LF_support_init.linear().setIdentity();
    PELV_float_current.linear().setIdentity();
    PELV_float_init.linear().setIdentity();
    COM_float_current.linear().setIdentity();
    COM_float_init.linear().setIdentity();
    SUF_float_current.linear().setIdentity();
    SUF_float_init.linear().setIdentity();
    SWF_float_current.linear().setIdentity();
    SWF_float_init.linear().setIdentity();
    PELV_support_current.linear().setIdentity();
    PELV_support_init.linear().setIdentity();
    PELV_trajectory_support.linear().setIdentity();
    COM_support_current.linear().setIdentity();
    COM_support_init.linear().setIdentity();

    RF_float_current.translation().setZero();
    LF_float_current.translation().setZero();
    RF_float_init.translation().setZero();
    LF_float_init.translation().setZero();
    RF_support_current.translation().setZero();
    LF_support_current.translation().setZero();
    RF_support_init.translation().setZero();
    LF_support_init.translation().setZero();
    PELV_float_current.translation().setZero();
    PELV_float_init.translation().setZero();
    COM_float_current.translation().setZero();
    COM_float_init.translation().setZero();
    SUF_float_current.translation().setZero();
    SUF_float_init.translation().setZero();
    SWF_float_current.translation().setZero();
    SWF_float_init.translation().setZero();
    PELV_support_current.translation().setZero();
    PELV_support_init.translation().setZero();
    PELV_trajectory_support.translation().setZero();
    COM_support_current.translation().setZero();
    COM_support_init.translation().setZero();
    
    PELV_trajectory_float.translation() = Robot.link_[Pelvis].xipos;
    RF_trajectory_float.translation() = Robot.link_[Right_Foot].xpos;
    LF_trajectory_float.translation() = Robot.link_[Left_Foot].xpos;
    PELV_trajectory_float.linear() = Robot.link_[Pelvis].Rotm;
    RF_trajectory_float.linear() = Robot.link_[Right_Foot].Rotm;
    LF_trajectory_float.linear() = Robot.link_[Left_Foot].Rotm;

    PELVD_trajectory_float.translation().setZero();
    RFD_trajectory_float.translation().setZero();
    LFD_trajectory_float.translation().setZero();
    PELVD_trajectory_float.linear().setIdentity();
    RFD_trajectory_float.linear().setIdentity();
    LFD_trajectory_float.linear().setIdentity();


    PELV_trajectory_euler.setZero();
    com_desired.setZero();
    zmp_desired.setZero();
    com_support_temp.setZero();
    q_w.resize(19);
    q_w.setZero();

    RF_float_current(3, 3) = 1.0;
    LF_float_current(3, 3) = 1.0;
    RF_float_init(3, 3) = 1.0;
    LF_float_init(3, 3) = 1.0;
    RF_support_current(3, 3) = 1.0;
    LF_support_current(3, 3) = 1.0;
    RF_support_init(3, 3) = 1.0;
    LF_support_init(3, 3) = 1.0;
    PELV_float_current(3, 3) = 1.0;
    PELV_float_init(3, 3) = 1.0;
    COM_float_current(3, 3) = 1.0;
    COM_float_init(3, 3) = 1.0;
    SUF_float_current(3, 3) = 1.0;
    SUF_float_init(3, 3) = 1.0;
    SWF_float_current(3, 3) = 1.0;
    SWF_float_init(3, 3) = 1.0;
    PELV_support_current(3, 3) = 1.0;
    PELV_support_init(3, 3) = 1.0;
    PELV_trajectory_support(3, 3) = 1.0;
    COM_support_current(3, 3) = 1.0;
    COM_support_init(3, 3) = 1.0;
    pelvis_offsetx = 0.0;
    target.setZero();

    q_dm.resize(5);
       

    kx_vib(0) = 17417.76;
    cx_vib(0) = 144.9;
/*
    kx_vib(0) = 918.22;
    cx_vib(0) = 7788.16;
*/
//    kx_vib(0) = 918.22;
//    cx_vib(0) = 7788.16;

    ky_vib(0) = 8276.02;
    cy_vib(0) = 130.1744;

    m = 9.81 * Robot.total_mass;

    Ax_vib(0,0) = 0.0;
    Ax_vib(0,1) = 1.0;
    Ax_vib(1,0) = 9.81 - kx_vib(0)/Robot.total_mass;
    Ax_vib(1,1) = -cx_vib(0)/Robot.total_mass;

    Bx_vib(0) = 0;
    Bx_vib(1) = kx_vib(0)/Robot.total_mass;

    Dx_vib.setZero();
    Dx_vib(0) = - kx_vib(0)/m;

    Ay_vib(0,0) = 0.0;
    Ay_vib(0,1) = 1.0;
    Ay_vib(1,0) = 9.81 - ky_vib(0)/Robot.total_mass;
    Ay_vib(1,1) = -cy_vib(0)/Robot.total_mass;

    By_vib(0) = 0;
    By_vib(1) = ky_vib(0)/Robot.total_mass;

    Dy_vib.setZero();
    Dy_vib(0) = - ky_vib(0)/m;

                    
    L1.resize(2,3); 
    L2.resize(2,3);

    L1(0,0) = 0.0199;
    L1(0,1) = 0.0579;
    L1(0,2) = 1.9969;
    L1(1,0) = -7.785;
    L1(1,1) = 1.9969;
    L1(1,2) = 114.3008;
    
    vib_est = false;

    J.resize(INERITA_SIZE, MODEL_DOF);
    J.setZero();

    current_step_num = 0.0;
}

void Walking_controller::updateNextStepTime()
{
    if (walking_tick == t_last)
    {
        if (current_step_num != total_step_num - 1)
        {
            t_start = t_last +1;    
            foot_step_dir = 1.0;
            t_start_real = t_start + t_rest_init;
            t_last = t_start + t_total - 1;

            current_step_num++;
        }
    }
    if (current_step_num == total_step_num - 1 && walking_tick >= t_total + t_last - 3)
    {
        walking_enable = 2.0;
    }

    if (walking_tick >= t_start_real + t_double1 + t_rest_temp - 0.075*Hz_ && walking_tick <= t_start_real + t_double1 + t_rest_temp + 0.015*Hz_ + 1&& current_step_num !=0)
    {
        phaseChange = true;
        phaseChange1 = false;
        double2Single_pre = t_start_real + t_double1 + t_rest_temp - 0.075*Hz_;
        double2Single = t_start_real + t_double1 + t_rest_temp + 0.015*Hz_-1;
    }
    else
    {
        phaseChange = false;
    }

    if(walking_tick >= t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp && walking_tick <= t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp +0.1*Hz_ - 1 &&current_step_num != 0 && phaseChange == false)
    {
        phaseChange1 = true;
        phaseChange = false;
        single2Double_pre = t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp + 1;
        single2Double = t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp + 0.1*Hz_;
    }
    else
    {
        phaseChange1 = false;
        if(walking_tick < t_start_real + t_double1 + t_rest_temp - 0.075*Hz_ &&  walking_tick > t_start_real + t_double1 + t_rest_temp + 0.015*Hz_ + 1)  
            phaseChange = false; 
    }
    walking_tick++;
}

void Walking_controller::getUiWalkingParameter(int controller_Hz, int walkingenable, int ikmode, int walkingpattern, int walkingpattern2, int footstepdir, double target_x, double target_y, double target_z, double theta, double targetheight, double steplength_x, double steplength_y, int dob_walk, int imu_walk, bool mom_walk, int vibration, RobotData &Robot)
{
    ik_mode = ikmode;
    walking_pattern = walkingpattern;
    if (footstepdir == 0)
    {
        foot_step_dir = 1.0;
    }
    else
    {
        foot_step_dir = -1.0;
    }
    target(0) = target_x;
    target(1) = target_y;
    target(2) = target_z;
    target(3) = theta;
    height = targetheight;
    vibration_control = vibration;
    step_length_y = steplength_y;
    step_length_x = steplength_x;
    com_control = walkingpattern2;
    dob = dob_walk;
    imu = imu_walk;
    mom = mom_walk;
    Hz_ = controller_Hz;
    dt = 1 / Hz_;
    walking_enable = walkingenable;
    foot_height = 0.030;
    com_control_mode = true;
    gyro_frame_flag = false;

    if(com_control_mode == true)
    {
        pelvis_pgain = 0.1;
        com_gain = 100.0;
    }
    else
    {
        pelvis_pgain = 0.5;
        pelvis_dgain = 0.5;
        com_gain = 100.0;
    }
    std::cout << "ik mode " << ik_mode << std::endl;
    std::cout << "dob" << dob << std::endl;
    std::cout << "imu" << imu << std::endl; 
    std::cout << "mom" << mom << std::endl;
    std::cout << "vibration" << vibration_control << std::endl;

    setWalkingParameter(Robot);
}

void Walking_controller::hipCompensator()
{
    double left_hip_angle = 2.0 * DEG2RAD, right_hip_angle = 2.0 * DEG2RAD, left_hip_angle_first_step = 2.0 * DEG2RAD, right_hip_angle_first_step = 2.0 * DEG2RAD,
           left_hip_angle_temp = 0.0, right_hip_angle_temp = 0.0, temp_time = 0.1 * Hz_, left_pitch_angle = 0.0 * DEG2RAD;

    if (current_step_num == 0)
    {
        if (foot_step(current_step_num, 6) == 1) //left support foot
        {
            if (walking_tick < t_start + t_total - t_rest_last - t_double2 - temp_time)
                left_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1, t_start_real + t_double1 + temp_time, 0.0 * DEG2RAD, 0.0, 0.0, left_hip_angle_first_step, 0.0, 0.0)(0);
            else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - temp_time)
                left_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - temp_time, t_start + t_total - t_rest_last, left_hip_angle_first_step, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            else
                left_hip_angle_temp = 0.0 * DEG2RAD;
        }
        else if (foot_step(current_step_num, 6) == 0) // right support foot
        {
            if (walking_tick < t_start + t_total - t_rest_last - t_double2 - temp_time)
                right_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1, t_start_real + t_double1 + temp_time, 0.0 * DEG2RAD, 0.0, 0.0, right_hip_angle_first_step, 0.0, 0.0)(0);
            else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - temp_time)
                right_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - temp_time, t_start + t_total - t_rest_last, right_hip_angle_first_step, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            else
                right_hip_angle_temp = 0.0 * DEG2RAD;
        }
        else
        {
            left_hip_angle_temp = 0.0 * DEG2RAD;
            right_hip_angle_temp = 0.0 * DEG2RAD;
        }
    }
    else
    {
        if (foot_step(current_step_num, 6) == 1)
        {
            if (walking_tick < t_start + t_total - t_rest_last - t_double2 - temp_time)
                left_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1, t_start_real + t_double1 + temp_time, 0.0 * DEG2RAD, 0.0, 0.0, left_hip_angle, 0.0, 0.0)(0);
            else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - temp_time)
                left_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - temp_time, t_start + t_total - t_rest_last, left_hip_angle, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            else
                left_hip_angle_temp = 0.0 * DEG2RAD;
        }
        else if (foot_step(current_step_num, 6) == 0)
        {
            if (walking_tick < t_start + t_total - t_rest_last - t_double2 - temp_time)
                right_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1, t_start_real + t_double1 + temp_time, 0.0 * DEG2RAD, 0.0, 0.0, right_hip_angle, 0.0, 0.0)(0);
            else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - temp_time)
                right_hip_angle_temp = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - temp_time, t_start + t_total - t_rest_last, left_hip_angle, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            else
                right_hip_angle_temp = 0.0 * DEG2RAD;
        }
        else
        {
            left_hip_angle_temp = 0.0 * DEG2RAD;
            right_hip_angle_temp = 0.0 * DEG2RAD;
        }
    }
    desired_leg_q(1) = desired_leg_q(1) + left_hip_angle_temp;
    desired_leg_q(7) = desired_leg_q(7) - right_hip_angle_temp;
}

void Walking_controller::inverseKinematicsdob(RobotData &Robot)
{
    leg_q = desired_leg_q;
    double Kp, Kv;

    for (int i = 0; i < 12; i++)
    {
        dob_hat(i) = desired_leg_q(i) - Robot.q_(i);
    }

    if (walking_tick == 0)
        dob_hat_prev = dob_hat;

    dob_hat = 0.3 * dob_hat + 0.7 * dob_hat_prev;

    double defaultGain = 0.0;
    double compliantGain = 3.0;
    double compliantTick = 0.1 * Hz_;

    for (int i = 0; i < 12; i++)
    {
        if (i < 6)
        {
            dobGain = defaultGain;

            if (foot_step(current_step_num, 6) == 0)
            {
                if (walking_tick < t_start + t_total - t_rest_last - t_double2 - compliantTick)
                {
                    dobGain = defaultGain;
                }
                else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - compliantTick && walking_tick < t_start + t_total - t_rest_last - t_double2)
                {
                    dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - compliantTick, t_start + t_total - t_rest_last - t_double2, defaultGain, 0.0, 0.0, compliantGain, 0.0, 0.0)(0);
                }
                else
                {
                    dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last, t_start + t_total, compliantGain, 0.0, 0.0, defaultGain, 0.0, 0.0)(0);
                }
                
            }            
            else
            {
                dobGain = defaultGain;
            }

            desired_leg_q(i) = desired_leg_q(i) - dobGain * dob_hat(i);
        }
        else
        {
            dobGain = defaultGain;

            if (foot_step(current_step_num, 6) == 1)
            {
                if (walking_tick < t_start + t_total - t_rest_last - t_double2 - compliantTick)
                {
                    dobGain = defaultGain;
                }
                else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - compliantTick && walking_tick < t_start + t_total - t_rest_last - t_double2)
                {
                    dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - compliantTick, t_start + t_total - t_rest_last - t_double2, defaultGain, 0.0, 0.0, compliantGain, 0.0, 0.0)(0);
                }
                else
                {
                    dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last, t_start + t_total, compliantGain, 0.0, 0.0, defaultGain, 0.0, 0.0)(0);
                }
            }
            else
            {
                dobGain = defaultGain;
            }

            desired_leg_q(i) = desired_leg_q(i) - dobGain * dob_hat(i);
        }
    }
    dob_debug = dobGain*dob_hat;
}

void Walking_controller::ankleOriControl(RobotData &Robot)
{
    Eigen::Vector2d k, kv;
    //2.5
    k(0) = -0.2;
    kv(0) = 0.0;//-0.01;
    k(1) = -0.2;
    kv(1) = 0.0;//-0.01;

    //2.5
  /*  k(0) = -2.5;
    kv(0) = -0.5;
    k(1) = -2.5;
    kv(1) = -0.5;
*/
    /*  if(walking_tick < t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp)/2.0) // the period for lifting the right foot
    {
        for(int i = 0; i<2;i ++)
        {
                k(i) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+t_rest_temp, t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2, 0,k(i),0,0);
                kv(i) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+t_rest_temp, t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2, 0,kv(i),0,0);
        }
    }
    else
    {
        for(int i = 0; i <2 ; i++)
        {
            k(i) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0, t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp, k(i),0,0,0);
            kv(i) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0, t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp, kv(i),0,0,0);
        }
    }*/

    if (foot_step(current_step_num, 6) == 1)
    {
        rf_e(0) = DyrosMath::rot2Euler(Robot.link_[Right_Foot].Rotm)(0);
        rf_e(1) = DyrosMath::rot2Euler(Robot.link_[Right_Foot].Rotm)(1);
        rf_e(2) = DyrosMath::rot2Euler(Robot.link_[Right_Foot].Rotm)(2);

        rf_e_vel = Robot.link_[Right_Foot].w;
         
        for(int i = 0; i<2; i++)
        {
            rf_e(i) = k(i) * rf_e(i) + kv(i) * rf_e_vel(i);
        }

        if (abs(rf_e(0)) > 25 * DEG2RAD)
        {
            if (rf_e(0) < 0)
            {
                rf_e(0) = -25 * DEG2RAD;
            }
            else
            {
                rf_e(0) = 25 * DEG2RAD;
            }
        }

        if (abs(rf_e(1)) > 25 * DEG2RAD)
        {
            if (rf_e(1) < 0)
            {
                rf_e(1) = -25 * DEG2RAD;
            }
            else
            {
                rf_e(1) = 25 * DEG2RAD;
            }
        }

        RF_trajectory_float.linear() = DyrosMath::rotateWithY(rf_e(1)) * DyrosMath::rotateWithX(rf_e(0));
    }
    else
    {
        lf_e(0) = DyrosMath::rot2Euler(Robot.link_[Left_Foot].Rotm)(0);
        lf_e(1) = DyrosMath::rot2Euler(Robot.link_[Left_Foot].Rotm)(1);
        lf_e(2) = DyrosMath::rot2Euler(Robot.link_[Left_Foot].Rotm)(2);

        lf_e_vel = Robot.link_[Left_Foot].w;

        for (int i = 0; i < 2; i++)
        {
            lf_e(i) = k(i) * lf_e(i) + kv(i) * lf_e_vel(i);
        }

        if (abs(lf_e(0)) > 25 * DEG2RAD)
        {
            if (lf_e(0) < 0)
            {
                lf_e(0) = -25 * DEG2RAD;
            }
            else
            {
                lf_e(0) = 25 * DEG2RAD;
            }
        }

        if (abs(lf_e(1)) > 25 * DEG2RAD)
        {
            if (lf_e(1) < 0)
            {
                lf_e(1) = -25 * DEG2RAD;
            }
            else
            {
                lf_e(1) = 25 * DEG2RAD;
            }
        }
        LF_trajectory_float.linear() = DyrosMath::rotateWithY(lf_e(1))*DyrosMath::rotateWithX(lf_e(0));
    }
}

void Walking_controller::comJacobianState(RobotData &Robot)
{
    Eigen::Vector6d kp;
    if(foot_step(current_step_num, 6) == 1)
    {
        r_c1(0) = com_refx(walking_tick) - LF_trajectory_float.translation()(0);
        r_c1(1) = com_refy(walking_tick) - LF_trajectory_float.translation()(1);
        r_c1(2) = COM_float_init.translation()(2)- LF_float_init.translation()(2);
        r_c1_skew = DyrosMath::skew(r_c1);

        X21.setZero();
        X21.block<3,3>(0,0).setIdentity();
        X21.block<3,3>(3,3).setIdentity();
        X21.block<3,3>(0,3) = DyrosMath::skew(LF_trajectory_float.translation() - RF_trajectory_float.translation());

        if(walking_tick == 0 || walking_tick == t_temp + (current_step_num)*t_total + 1)
        {
            RFDotTraj.setZero();
            LFDotTraj.setZero();
        }
        else
        {
            RFDotTraj.head(3) = (RF_trajectory_float.translation()-RFDotPrevTraj.head(3))*Hz_;
            RFDotTraj.tail(3).setZero();
            LFDotTraj.tail(3).setZero();
        }
        SFerr.head(3) = (RF_trajectory_float.translation() - PELV_float_init.inverse()*RF_float_current.translation());
        SFerr.tail(3) = DyrosMath::rot2Axis(RF_trajectory_float.linear() * (RF_float_current.linear().transpose())).head(3);
     //   RFDotTraj(0) = -1 * RFDotTraj(0);
        RFDotTraj(1) = -1 * RFDotTraj(1);
        //RFDotTraj(2) = -1 * RFDotTraj(2);
        RFDotPrevTraj.head(3) = RF_trajectory_float.translation();
        LFDotPrevTraj.head(3) = LF_trajectory_float.translation();
        COMDotTraj(0) = com_refdx(walking_tick) - 50*(com_refx(walking_tick) - (PELV_float_init.inverse()*COM_float_current).translation()(0));
        COMDotTraj(1) = com_refdy(walking_tick);
        COMDotTraj(2) = 0.0;
        COMDotTraj.tail(3).setZero();
     //   COMDotTraj(0) = -1 * COMDotTraj(0);
        COMDotTraj(1) = -1 * COMDotTraj(1);
     //   COMDotTraj(2) = -1 * COMDotTraj(2);
        Jfsem = - J_l.block<3,6>(0,0) + r_c1_skew * J_l.block<3,6>(3,0) + J_lc.block<3,6>(0,0) + J_rc.block<3,6>(0,0)*J_r.inverse()*X21*J_l;
        SFerr.tail(3).setZero();
        for(int i = 0; i < 3; i++)
        {
            kp(i) = 2;
            kp(i+3) = 0;
        }
        for(int i = 0; i < 6; i++)
        {
            RFDotTraj(i) = RFDotTraj(i);// + kp(i)  * SFerr(i);
        }

        Cfsemd = COMDotTraj.head(3) - J_rc.block<3,6>(0,0)*J_r.inverse()*RFDotTraj;   
    }
    else
    {
        r_c1(0) = com_refx(walking_tick) - RF_trajectory_float.translation()(0);
        r_c1(1) = com_refy(walking_tick) - RF_trajectory_float.translation()(1);
        r_c1(2) = COM_float_init.translation()(2)- LF_float_init.translation()(2);
        r_c1_skew = DyrosMath::skew(r_c1);

        X21.setZero();
        X21.block<3,3>(0,0).setIdentity();
        X21.block<3,3>(3,3).setIdentity();
        X21.block<3,3>(0,3) = DyrosMath::skew(RF_trajectory_float.translation() - LF_trajectory_float.translation());

        if(walking_tick == 0 || walking_tick == t_temp + (current_step_num)*t_total + 1)
        {
            RFDotTraj.setZero();
            LFDotTraj.setZero();
            COMDotTraj.setZero();
        }
        else
        {
            LFDotTraj.head(3) = (LF_trajectory_float.translation()-LFDotPrevTraj.head(3))*Hz_;
            RFDotTraj.tail(3).setZero();
            LFDotTraj.tail(3).setZero();
        }
        SFerr.head(3) = (LF_trajectory_float.translation() - PELV_float_init.inverse()*LF_float_current.translation());

        SFerr.tail(3) = DyrosMath::rot2Axis(LF_trajectory_float.linear() * (LF_float_current.linear().transpose())).head(3);
        RFDotPrevTraj.head(3) = RF_trajectory_float.translation();
        LFDotPrevTraj.head(3) = LF_trajectory_float.translation();
     //   LFDotTraj(0) = -1 * LFDotTraj(0);
        LFDotTraj(1) = -1 * LFDotTraj(1);
     //   LFDotTraj(2) = -1 * LFDotTraj(2); 
        COMDotTraj(0) = com_refdx(walking_tick) - 50*(com_refx(walking_tick) - (PELV_float_init.inverse()*COM_float_current).translation()(0));
        COMDotTraj(1) = com_refdy(walking_tick);
        COMDotTraj(2) = 0.0;
        COMDotTraj.tail(3).setZero();
     //   COMDotTraj(0) = -1 * COMDotTraj(0);
        COMDotTraj(1) = -1 * COMDotTraj(1);
      //  COMDotTraj(2) = -1 * COMDotTraj(2);
        Jfsem = - J_r.block<3,6>(0,0) + r_c1_skew * J_r.block<3,6>(3,0) + J_rc.block<3,6>(0,0) + J_lc.block<3,6>(0,0)*J_l.inverse()*X21*J_r;
        SFerr.tail(3).setZero();
        for(int i = 0; i < 3; i++)
        {
            kp(i) = 2;
            kp(i+3) = 0;
        }

        for(int i = 0; i < 6; i++)
        {
            LFDotTraj(i) = LFDotTraj(i);// + kp(i) * SFerr(i);
        }
        Cfsemd = COMDotTraj.head(3) - J_lc.block<3,6>(0,0)*J_l.inverse()*LFDotTraj;
    }
}

void Walking_controller::comJacobianIK(RobotData &Robot)
{
    Eigen::Vector6d temp;
    Eigen::Vector12d leg_qdot;

    if(walking_tick == 0)
    {
        desired_leg_q_NC = Robot.q_.head(12);
    }

    if(foot_step(current_step_num, 6) == 1)
    {
        Eigen::Matrix6d J; 
        Eigen::Vector6d Cdot;
        J.block<3,6>(0,0) = Jfsem; 
        J.block<3,6>(3,0) = -J_l.block<3,6>(3,0);

        Cdot.segment<3>(0) = Cfsemd;
        Cdot.segment<3>(3).setZero();
        Cdot(2) = 0.0;
        leg_qdot.segment<6>(0) = J.inverse()*Cdot;

        temp = X21*J_l*leg_qdot.segment<6>(0);
        temp(2) = 0.0;
        leg_qdot.segment<6>(6) = J_r.inverse()*(RFDotTraj +  temp);
    }
    else
    {
        Eigen::Matrix6d J; 
        Eigen::Vector6d Cdot;
        J.block<3,6>(0,0) = Jfsem; 
        J.block<3,6>(3,0) = -J_r.block<3,6>(3,0);

        Cdot.segment<3>(0) = Cfsemd;
        Cdot.segment<3>(3).setZero();
        Cdot(2) = 0.0;

        leg_qdot.segment<6>(6) = J.inverse()*Cdot;

        temp = X21*J_r*leg_qdot.segment<6>(6);
        temp(2) = 0.0;
        leg_qdot.segment<6>(0) = J_l.inverse()*(LFDotTraj + temp);
    }

    desired_leg_q = desired_leg_q_NC + leg_qdot/Hz_; 
    desired_leg_q_NC = desired_leg_q;
}

void Walking_controller::momentumControl(RobotData &Robot)
{
    int variable_size, constraint_size;

    variable_size = 5;
    constraint_size = 5;

    if(walking_tick == 0)
        QP_m.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    Eigen::Vector3d q_waistd;
    Eigen::Vector8d q_rarmd, q_larmd;
    
    if(walking_tick == 0)
    {
        q_waistd.setZero();
        q_rarmd.setZero();
        q_larmd.setZero();
        qd_prev.setZero();
        desired_leg_q_dot.setZero();
    }
    else
    {   
        q_waistd.setZero();
        q_rarmd.setZero();
        q_larmd.setZero();
        for(int i = 0; i <12; i++)
        {
            desired_leg_q_dot(i) = (desired_leg_q(i)-desired_leg_q_prev(i))*Hz_;
        }
        
        for(int i = 0; i < 3; i++)
        {
           q_waistd(i) = q_dm(i);
        }        
        q_rarmd(1) = q_dm(4);
	    q_larmd(1) = q_dm(3);
    }
    H_leg.setZero();
    H_leg = Ag_leg * Robot.q_dot_est.head(12) + Ag_waist * Robot.q_dot_est.segment(12,3) + Ag_armL * Robot.q_dot_est.segment(15,8) + Ag_armR * Robot.q_dot_est.segment(25,8);

    Eigen::MatrixXd Ag_temp;
    Eigen::Matrix5d I;
    I.setIdentity();
    double alpha = 0.05;

    Ag_temp.resize(3, 5);
    Ag_temp.block<3,3>(0,0) = Ag_waist;
    Ag_temp.block<3,1>(0,3) = Ag_armL.block<3,1>(0,1);
    Ag_temp.block<3,1>(0,4) = Ag_armR.block<3,1>(0,1);
   
    H = Ag_temp.transpose()*Ag_temp;// + alpha*I;
    g = 2*Ag_temp.transpose()*H_leg;//- 2*alpha*qd_prev;
 
    A.setIdentity();

    for(int i=0; i<3; i++)
    {   
        lbA(i) = (-0.2 - q_w(i))*Hz_;
        ubA(i) = (0.2 - q_w(i))*Hz_;
    }

    lbA(3) = (0.15 - q_w(3))*Hz_;
    ubA(3) = (0.45 - q_w(3))*Hz_;
    lbA(4) = (-0.45 - q_w(4))*Hz_;
    ubA(4) = (-0.15 - q_w(4))*Hz_;

    for(int i=0; i<variable_size; i++)
    {
        lb(i) = -2.0;
        ub(i) = 2.0;
    }

    lb(3) = -0.5;
    lb(4) = -0.5;

    ub(3) = 0.5;
    ub(4) = 0.5;

    QP_m.EnableEqualityCondition(0.001);
    QP_m.UpdateMinProblem(H, g);
    QP_m.UpdateSubjectToAx(A, lbA, ubA);
    QP_m.UpdateSubjectToX(lb, ub);

    QP_m.SolveQPoases(100, q_dm);

    qd_prev = q_dm;

}


void Walking_controller::comVibrationController(RobotData &Robot)
{
    if(vibration_control == 1)
    {
        if (current_step_num != total_step_num  && walking_tick < t_total + t_last - 3)
        {
            if(walking_tick ==0)
            {
                xx_vib(0) = com_refx(walking_tick);
                xx_vib(1) = 0.0;
                xy_vib(0) = com_refy(walking_tick);
                xy_vib(1) = 0.0;
            }
                
            if(vib_est == false)
            {
                xx_vib_est = xx_vib;
                xy_vib_est = xy_vib;
                ux_vib = PELV_first_init.translation()(0);
                uy_vib = PELV_first_init.translation()(1);
                vib_est = true;
                yx_vib = yx_vibm;
            }

            L1(0,0) = 0.0353;
            L1(0,1) = 0.6480;
            L1(0,2) = 1.3826;
            L1(1,0) = -9.7031;
            L1(1,1) = 1.3826;
            L1(1,2) = 10.8382;

            L2(0,0) = 0.0751;
            L2(0,1) = 0.6530;
            L2(0,2) = 1.3531;
            L2(1,0) = -9.5172;
            L2(1,1) = 1.3531;
            L2(1,2) = 10.6833;

            Cx_vib.resize(3,2);
            Cy_vib.resize(3,2);

            Cx_vib(0,0) = kx_vib(0)/m;
            Cx_vib(0,1) = cx_vib(0)/m;
            Cx_vib(1,0) = 1.0;
            Cx_vib(1,1) = 0.0; 
            Cx_vib(2,0) = 0.0;
            Cx_vib(2,1) = 1.0;

            Cy_vib(0,0) = ky_vib(0)/m;
            Cy_vib(0,1) = cy_vib(0)/m;
            Cy_vib(1,0) = 1.0;
            Cy_vib(1,1) = 0.0; 
            Cy_vib(2,0) = 0.0;
            Cy_vib(2,1) = 1.0;

            if(vib_est = true)
            {
                xx_vib_est = Ax_vib*xx_vib_est/Hz_ + xx_vib_est + Bx_vib*ux_vib/Hz_ + L1*(yx_vibm-yx_vib)/Hz_;

                yx_vib = Cx_vib * xx_vib_est + Dx_vib*ux_vib;
               
                ux_vib = yx_vib(1);

                yy_vib = Cy_vib * xy_vib_est + Dy_vib*uy_vib;

                xy_vib_est = Ay_vib*xy_vib_est/Hz_ + xy_vib_est + By_vib*uy_vib/Hz_ + L2*(yy_vibm-yy_vib)/Hz_;
            }    
          
            PELV_trajectory_float.translation()(0) = com_refx(walking_tick) - 5.5/*0.9*/ * (Robot.com_.pos(0)-(-PELV_float_init.translation()(0)+COM_float_init.translation()(0))-com_refx(walking_tick));
            PELV_trajectory_float.translation()(1) = com_refy(walking_tick) - 3.0/*0.9*/ * (Robot.com_.pos(1)- com_refy(walking_tick));
        }
        else
        {
            PELV_trajectory_float.translation()(0) = com_refx(t_total + t_last - 4) - 5.5/*0.9*/ * (Robot.com_.pos(0)-(-PELV_float_init.translation()(0)+COM_float_init.translation()(0))-com_refx(t_total + t_last - 4));
            PELV_trajectory_float.translation()(1) = com_refy(t_total + t_last - 4) - 3.0/*0.9*/ * (Robot.com_.pos(1)-com_refy(t_total + t_last - 4));
        }
        
    }
    else
    {
        PELV_trajectory_float.translation()(0) = PELV_trajectory_float.translation()(0) + 3.0*(COM_float_current.translation()(0) - com_refx(walking_tick));        
    }    
}

void Walking_controller::setWalkingParameter(RobotData &Robot)
{
    desired_foot_step_num = 8;
    walking_tick = 0;
    t_rest_init = 0.1 * Hz_;
    t_rest_last = 0.1 * Hz_;
    t_double1 = 0.1 * Hz_;
    t_double2 = 0.1 * Hz_;
    t_total = 1.0 * Hz_;
    t_temp = 4.0 * Hz_;
/*
    t_double1 = 0.35*Hz_;
    t_double2 = 0.35*Hz_;
    t_rest_init = .15*Hz_;
    t_rest_last = .15*Hz_;
    t_total= 2.0*Hz_;*/
    t_rest_temp = 0.0 * Hz_;

    t_imp = 0.0 * Hz_;
    t_last = t_total + t_temp;
    t_start = t_temp + 1;

    t_start_real = t_start + t_rest_init;
}

void Walking_controller::updateInitTime()
{
    walking_tick++;
}
