#include "tocabi_controller/walking_controller.h"
void Walking_controller::walkingCompute(RobotData &Robot)
{
    if(walking_enable == 1.0)
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
        if(walking_tick == 0)
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
        
        /////InverseKinematics//////
        inverseKinematics(PELV_trajectory_float, LF_trajectory_float, RF_trajectory_float, desired_leg_q);

        leg_q_NC = desired_leg_q;
        
        //hipCompensator();
        //ankleOriControl(Robot);
        inverseKinematicsdob(Robot);

        updateNextStepTime();
    }
    else if(walking_enable == 3.0)
    {
        setInitPose(Robot, desired_init_leg_q);
        updateInitTime();
    }
}

void Walking_controller::inverseKinematics(Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d& leg_q)
{
    Eigen::Vector3d lp, rp;  
    lp = LF_float_transform.linear().transpose()*(PELV_float_transform.translation()-LF_float_transform.translation());
    rp = RF_float_transform.linear().transpose()*(PELV_float_transform.translation()-RF_float_transform.translation());

    Eigen::Matrix3d PELF_rotation,PERF_rotation;
    PELF_rotation = PELV_float_transform.linear().transpose() * LF_float_transform.linear();
    PERF_rotation = PELV_float_transform.linear().transpose() * RF_float_transform.linear();

    Eigen::Vector3d ld, rd;
    ld.setZero(); rd.setZero();
    ld(0) = 0;
    ld(1) = 0.1025;
    ld(2) = -0.1225;
    rd(0) = 0;
    rd(1) = -0.1025;
    rd(2) = -0.1225;

    ld = PELF_rotation.transpose() * ld;
    rd = PERF_rotation.transpose() * rd;

    Eigen::Vector3d lr, rr;
    lr = lp + ld;
    rr = rp + rd;

    double l_upper = 0.35; //direct length from hip to knee
    double l_lower = 0.35; //direct length from knee to ankle

    double offset_hip_pitch = 0.0*DEG2RAD;
    double offset_knee_pitch =  0.0*DEG2RAD;
    double offset_ankle_pitch = 0.0*DEG2RAD;
  //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

    double lc = sqrt(lr(0)*lr(0)+lr(1)*lr(1)+lr(2)*lr(2));
    leg_q(3) = (-acos((l_upper*l_upper + l_lower*l_lower - lc*lc) / (2*l_upper*l_lower))+ M_PI); // - offset_knee_pitch //+ alpha_lower

    double l_ankle_pitch = asin((l_upper*sin(M_PI-leg_q(3)))/lc);
    leg_q(4) = -atan2(lr(0), sqrt(lr(1)*lr(1)+lr(2)*lr(2))) - l_ankle_pitch;// - offset_ankle_pitch ;
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

    leg_q(1) = asin(r_tl2(2,1));

    double c_lq5 = -r_tl2(0,1)/cos(leg_q(1));
    if(c_lq5 > 1.0)
    {
        c_lq5 =1.0;
    }
    else if(c_lq5 < -1.0)
    {
        c_lq5 = -1.0;
    }

    leg_q(0) = -asin(c_lq5);
    leg_q(2) = -asin(r_tl2(2,0)/cos(leg_q(1)))+ offset_hip_pitch;
    leg_q(3) = leg_q(3)- offset_knee_pitch;
    leg_q(4) = leg_q(4)- offset_ankle_pitch;

    //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

    double rc = sqrt(rr(0)*rr(0)+rr(1)*rr(1)+rr(2)*rr(2));
    leg_q(9) = (-acos((l_upper*l_upper + l_lower*l_lower - rc*rc) / (2*l_upper*l_lower))+ M_PI); // - offset_knee_pitch //+ alpha_lower

    double r_ankle_pitch = asin((l_upper*sin(M_PI-leg_q(9)))/rc);
    leg_q(10) = -atan2(rr(0), sqrt(rr(1)*rr(1)+rr(2)*rr(2)))-r_ankle_pitch;
    leg_q(11) = atan2(rr(1),rr(2));

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
    leg_q(7) = asin(r_tr2(2,1));
    double c_rq5 = -r_tr2(0,1)/cos(leg_q(7));
    
    if (c_rq5 > 1.0)
    {
        c_rq5 =1.0;
    }
    else if (c_rq5 < -1.0)
    {
        c_rq5 = -1.0;
    }

    leg_q(6) = -asin(c_rq5);
    leg_q(8) = asin(r_tr2(2,0)/cos(leg_q(7))) - offset_hip_pitch;
    leg_q(9) = -leg_q(9) + offset_knee_pitch;
    leg_q(10) = -leg_q(10) + offset_ankle_pitch;

    leg_q(0) = leg_q(0)*(-1);
    leg_q(6) = leg_q(6)*(-1);
    leg_q(8) = leg_q(8)*(-1);
    leg_q(9) = leg_q(9)*(-1);
    leg_q(10) = leg_q(10)*(-1);
}

void Walking_controller::setInitPose(RobotData &Robot, Eigen::VectorQd& leg_q)
{
    if(walking_tick == 0)
    {
        Eigen::VectorQd q_temp;
        q_temp << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0, 0.0, 0.0, -0.24, 0.6, -0.36, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0, 0, 0, -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0;
        //q_target = Robot.q_;
        q_target = q_temp;
        walkingInitialize(Robot);        
    }

    for(int i = 0; i < MODEL_DOF; i++)
    {
        leg_q(i) = DyrosMath::cubic(walking_tick, 0.0, 3.0*Hz_, q_init(i), q_target(i), 0.0, 0.0);
    }
}

void Walking_controller::walkingInitialize(RobotData &Robot)
{
    q_init = Robot.q_;
}

void Walking_controller::getRobotState(RobotData &Robot)
{
    //////Real Robot Float Frame//////
    RF_float_current.translation() = Robot.link_[Right_Foot].xpos;
    RF_float_current.linear() = Robot.link_[Right_Foot].Rotm;
    LF_float_current.translation() = Robot.link_[Left_Foot].xpos;
    LF_float_current.linear() = Robot.link_[Left_Foot].Rotm;
    PELV_float_current.translation() = Robot.link_[Pelvis].xpos;
    PELV_float_current.linear() = Robot.link_[Pelvis].Rotm;

    COM_float_current.translation() = Robot.link_[COM_id].xpos;
    COM_float_current.linear() = Robot.link_[COM_id].Rotm;

    COMV_support_currentV = Robot.com_.vel;
    COMV_support_currentV = Robot.com_.accel;

    if(foot_step(current_step_num,6) == 0)
    {
        SUF_float_current = RF_float_current;
        SWF_float_current = LF_float_current;
        for(int i=0; i<3; i++)
        {
            SUF_float_currentV(i) = SUF_float_current.translation()(i);
            SWF_float_currentV(i) = SWF_float_current.translation()(i);
        }
        for(int i=0; i<3; i++)
        {
            SUF_float_currentV(i+3) = DyrosMath::rot2Euler(SUF_float_current.linear())(i);
            SWF_float_currentV(i+3) = DyrosMath::rot2Euler(SWF_float_current.linear())(i);
        }
    }
    else
    {
        SUF_float_current = LF_float_current;
        SWF_float_current = RF_float_current;
        for(int i=0; i<3; i++)
        {
            SUF_float_currentV(i) = SUF_float_current.translation()(i);
            SWF_float_currentV(i) = SWF_float_current.translation()(i);
        }
        for(int i=0; i<3; i++)
        {
            SUF_float_currentV(i+3) = DyrosMath::rot2Euler(SUF_float_current.linear())(i);
            SWF_float_currentV(i+3) = DyrosMath::rot2Euler(SWF_float_current.linear())(i);
        }
    }

    //////Real Robot Support Foot Frame//////
    PELV_support_current = DyrosMath::inverseIsometry3d(SUF_float_current)*PELV_float_current;
    RF_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, RF_float_current);
    LF_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, LF_float_current);
    COM_support_current =  DyrosMath::multiplyIsometry3d(PELV_support_current, COM_float_current);

    calcRobotState();
}


void Walking_controller::getRobotInitState(RobotData &Robot)
{
    if(walking_tick == 0)
    {   
        contactMode = 1.0;

        RF_float_init.translation() = Robot.link_[Right_Foot].xpos;
        RF_float_init.linear() = Robot.link_[Right_Foot].Rotm;
        LF_float_init.translation() = Robot.link_[Left_Foot].xpos;
        LF_float_init.linear() = Robot.link_[Left_Foot].Rotm;

        COM_float_init.translation() = Robot.com_.pos;
        COM_float_init.linear() = Robot.link_[COM_id].Rotm;

        PELV_float_init.translation() = Robot.link_[Pelvis].xpos;
        PELV_float_init.linear() = Robot.link_[Pelvis].Rotm;

        PELV_first_init = PELV_float_init;
        RF_fisrt_init = RF_float_init;
        LF_fisrt_init = LF_float_init;
        
        Eigen::Isometry3d temp;
        temp.linear() = PELV_first_init.linear();
        temp.translation().setZero();
        foot_distance = temp.inverse()*(LF_fisrt_init.translation() - RF_fisrt_init.translation());

        if(foot_step_dir != 1)
        {   
            SUF_float_init = RF_float_init;
            SWF_float_init = LF_float_init;
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i+3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i+3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        else
        {   
            SUF_float_init = LF_float_init;
            SWF_float_init = RF_float_init;
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i+3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i+3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        //////Real Robot Support Foot Frame//////
        RF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), RF_float_init);
        LF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), LF_float_init);
        PELV_support_init = DyrosMath::inverseIsometry3d(SUF_float_init)*PELV_float_init;
        COM_support_init =  DyrosMath::multiplyIsometry3d(PELV_support_init, COM_float_init);

        RF_support_euler_init = DyrosMath::rot2Euler(RF_support_init.linear());
        LF_support_euler_init = DyrosMath::rot2Euler(LF_support_init.linear());
        PELV_support_euler_init = DyrosMath::rot2Euler(PELV_support_init.linear());

        PELV_firstinit.head(3) = PELV_support_init.translation();
        PELV_firstinit(3,0) = 1.0;

        zc = COM_support_init.translation()(2);

        lipm_w = sqrt(GRAVITY/zc);
    }
  /*  else if(current_step_num!=0 && walking_tick == t_start)
    {   
        RF_float_init.translation() = Robot.link_[Right_Foot].xpos;
        RF_float_init.linear() = Robot.link_[Right_Foot].Rotm;
        LF_float_init.translation() = Robot.link_[Left_Foot].xpos;
        LF_float_init.linear() = Robot.link_[Left_Foot].Rotm;

        COM_float_init.translation() = Robot.com_.pos;
        COM_float_init.linear() = Robot.link_[COM_id].Rotm;

        PELV_float_init.translation() = Robot.link_[Pelvis].xpos;
        PELV_float_init.linear() = Robot.link_[Pelvis].Rotm;

        if(foot_step(current_step_num,6) == 0)
        {
            SUF_float_init = RF_float_init;
            SWF_float_init = LF_float_init;
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i+3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i+3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        else
        {
            SUF_float_init = LF_float_init;
            SWF_float_init = RF_float_init;
            for(int i=0; i<2; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for(int i=0; i<3; i++)
            {
                SUF_float_initV(i+3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i+3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }

        //////Real Robot Support Foot Frame//////
        RF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), RF_float_init);
        LF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), LF_float_init);
        PELV_support_init = DyrosMath::inverseIsometry3d(SUF_float_init)*PELV_float_init;
        COM_support_init =  DyrosMath::multiplyIsometry3d(PELV_support_init, COM_float_init);
    
        RF_support_euler_init = DyrosMath::rot2Euler(RF_support_init.linear());
        LF_support_euler_init = DyrosMath::rot2Euler(LF_support_init.linear());
        PELV_support_euler_init = DyrosMath::rot2Euler(PELV_support_init.linear());

        zc = COM_support_init.translation()(2);
        lipm_w = sqrt(GRAVITY/zc);
    }*/
}

void Walking_controller::calcRobotState()
{
    if(walking_tick == 0)
    {
        com_support_temp = -1*COM_support_current.translation();
        com_support_temp_prev = -1*COM_support_current.translation();    
    }

    if(walking_tick < t_start)//+t_double1)
    {
        COM = COM_support_current.translation() + com_support_temp_prev;
    }
    else
    {
        COM = COM_support_current.translation() + com_support_temp;    
    }
       

    if(walking_tick == t_last && current_step_num <= total_step_num) 
    {
        com_support_temp_prev(1) = com_support_temp(1);
        com_support_temp(0) = com_support_temp(0) + COM_support_current.translation()(0);
        com_support_temp(1) = COM_support_current.translation()(1) - COM(1);// - (COM(1) - com_support_temp_prev(1));
        com_support_temp(2) = 0.0;
        //com_support_temp(0) = com_support_temp(0) + COM_support_current.translation()(0);
    }

    if(walking_tick == 0)
    {
        COM_prev = COM;
    }

    for(int i = 0; i<3; i++)
    {
        CP(i)  = COM(i) + (COM(i)-COM_prev(i))*Hz_/lipm_w ;
    }

    COM_prev = COM;

}

void Walking_controller::setRobotStateInitialize()
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
    PELV_trajectory_euler.setZero();
    com_desired.setZero();
    zmp_desired.setZero();
    com_support_temp.setZero();

    RF_float_current(3,3) = 1.0;
    LF_float_current(3,3) = 1.0;
    RF_float_init(3,3) = 1.0;
    LF_float_init(3,3) = 1.0;
    RF_support_current(3,3) = 1.0;
    LF_support_current(3,3) = 1.0;
    RF_support_init(3,3) = 1.0;
    LF_support_init(3,3) = 1.0;
    PELV_float_current(3,3) = 1.0;
    PELV_float_init(3,3) = 1.0;
    COM_float_current(3,3) = 1.0;
    COM_float_init(3,3) = 1.0;
    SUF_float_current(3,3) = 1.0;
    SUF_float_init(3,3) = 1.0;
    SWF_float_current(3,3) = 1.0;
    SWF_float_init(3,3) = 1.0;
    PELV_support_current(3,3) = 1.0;
    PELV_support_init(3,3) = 1.0;
    PELV_trajectory_support(3,3) = 1.0;
    COM_support_current(3,3) = 1.0;
    COM_support_init(3,3) = 1.0;
    pelvis_offsetx = 0.0;
    target.setZero();
    
    current_step_num = 0.0;
}

void Walking_controller::updateNextStepTime()
{
    if(walking_tick == t_last)
    {
        if(current_step_num != total_step_num-1)
        {
            t_start = t_last +1;
            t_start_real = t_start + t_rest_init;
            t_last = t_start + t_total -1;

            current_step_num ++;
        }
    }
    if(current_step_num == total_step_num -1 && walking_tick >= t_total+t_last-3)
    {
        walking_enable = 2.0;
    }

    if(walking_tick >= t_start_real+t_double1+t_rest_temp - 300 && walking_tick<= t_start_real+t_double1+t_rest_temp)
    {
        phaseChange = true;
        double2Single_pre = t_start_real+t_double1+t_rest_temp - 300;
        double2Single = t_start_real+t_double1+t_rest_temp;
    }
    else
    {
        phaseChange = false;
    }

    walking_tick ++;
}

void Walking_controller::getUiWalkingParameter(int controller_Hz, int walkingenable, int ikmode, int walkingpattern, int footstepdir, double target_x, double target_y, double target_z, double theta, double targetheight, double steplength_x, double steplength_y, int dob_, RobotData &Robot)
{
    ik_mode = ikmode;
    walking_pattern = walkingpattern;
    if(footstepdir == 0)
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
    step_length_y = steplength_y;
    step_length_x = steplength_x;
    dob = dob_;
    Hz_ = controller_Hz;
    dt = 1/Hz_;
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
        pelvis_pgain = 3.0;
        pelvis_dgain = 0.5;
        com_gain = 100.0;
    }

    setWalkingParameter(Robot);
}

void Walking_controller::hipCompensator()
{
    double left_hip_angle = 2.0*DEG2RAD, right_hip_angle = 2.0*DEG2RAD, left_hip_angle_first_step = 2.0*DEG2RAD, right_hip_angle_first_step = 2.0*DEG2RAD,
    left_hip_angle_temp = 0.0, right_hip_angle_temp = 0.0, temp_time = 0.1*Hz_, left_pitch_angle = 0.0*DEG2RAD;

    if(current_step_num == 0)
    {
        if(foot_step(current_step_num, 6) == 1) //left support foot
        {
            if(walking_tick < t_start+t_total-t_rest_last-t_double2-temp_time)
                left_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start_real+t_double1+temp_time,0.0*DEG2RAD, left_hip_angle_first_step, 0.0, 0.0);
            else if(walking_tick >= t_start+t_total-t_rest_last-t_double2-temp_time)
                left_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start+t_total-t_rest_last-t_double2-temp_time,t_start+t_total-t_rest_last,left_hip_angle_first_step, 0.0, 0.0, 0.0);
            else
                left_hip_angle_temp = 0.0*DEG2RAD;
        }
        else if (foot_step(current_step_num, 6) == 0) // right support foot
        {
            if(walking_tick < t_start+t_total-t_rest_last-t_double2-temp_time)
                right_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start_real+t_double1+temp_time,0.0*DEG2RAD, right_hip_angle_first_step, 0.0, 0.0);
            else if(walking_tick >= t_start+t_total-t_rest_last-t_double2-temp_time)
                right_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start+t_total-t_rest_last-t_double2-temp_time,t_start+t_total-t_rest_last,right_hip_angle_first_step, 0.0, 0.0, 0.0);
            else
                right_hip_angle_temp = 0.0*DEG2RAD;
        }
        else
        {
            left_hip_angle_temp = 0.0*DEG2RAD;
            right_hip_angle_temp = 0.0*DEG2RAD;
        }
    }
    else
    {
        if(foot_step(current_step_num, 6) == 1)
        {
            if(walking_tick < t_start+t_total-t_rest_last-t_double2-temp_time)
                left_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start_real+t_double1+temp_time,0.0*DEG2RAD,left_hip_angle,0.0,0.0);
            else if (walking_tick >= t_start+t_total-t_rest_last-t_double2-temp_time)
                left_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start+t_total-t_rest_last-t_double2-temp_time,t_start+t_total-t_rest_last,left_hip_angle,0.0,0.0,0.0);
            else
                left_hip_angle_temp = 0.0*DEG2RAD;

        }
        else if(foot_step(current_step_num,6) == 0)
        {
            if(walking_tick < t_start+t_total-t_rest_last-t_double2-temp_time)
                right_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start_real+t_double1+temp_time,0.0*DEG2RAD,right_hip_angle,0.0,0.0);
            else if(walking_tick >= t_start+t_total-t_rest_last-t_double2-temp_time)
                right_hip_angle_temp = DyrosMath::cubic(walking_tick,t_start+t_total-t_rest_last-t_double2-temp_time,t_start+t_total-t_rest_last,left_hip_angle,0.0,0.0,0.0);
            else
                right_hip_angle_temp = 0.0*DEG2RAD;
        }
        else
        {
            left_hip_angle_temp = 0.0*DEG2RAD;
            right_hip_angle_temp = 0.0*DEG2RAD;
        }
    }
    desired_leg_q(1) = desired_leg_q(1) + left_hip_angle_temp;
    desired_leg_q(7) = desired_leg_q(7) - right_hip_angle_temp;
}

void Walking_controller::inverseKinematicsdob(RobotData &Robot)
{
    leg_q = desired_leg_q;
    double Kp, Kv;

    rot_prev(0,0) = DyrosMath::rot2Euler((Robot.link_[Right_Foot].Rotm))(0);
    rot_prev(1,0) = DyrosMath::rot2Euler((Robot.link_[Right_Foot].Rotm))(1);
    
    rot_prev(0,1) = DyrosMath::rot2Euler((Robot.link_[Left_Foot].Rotm))(0);
    rot_prev(1,1) = DyrosMath::rot2Euler((Robot.link_[Left_Foot].Rotm))(1); 

    for(int i =0; i<12; i++)
    {
         dob_hat(i) = desired_leg_q(i) - Robot.q_(i);
    }
    
    if(walking_tick == 0)
        dob_hat_prev = dob_hat;
    
    dob_hat = 0.3*dob_hat + 0.7*dob_hat_prev;

    double defaultGain = 0.0;
    double compliantGain = 1.5;
    double compliantTick = 0.1 * Hz_;

    for(int i = 0; i <12 ; i++)
    {
        if(i < 6)
        {
            dobGain = defaultGain;

            if(foot_step(current_step_num,6) == 0)
            {
                if(walking_tick < t_start + t_total - t_rest_last - t_double2 - compliantTick)
                {
                    dobGain = defaultGain;
                }
                else if(walking_tick >= t_start + t_total - t_rest_last - t_double2 -compliantTick && walking_tick < t_start + t_total - t_rest_last - t_double2 )
                {
                    dobGain = DyrosMath::cubic(walking_tick, t_start+t_total-t_rest_last-t_double2-compliantTick, t_start+t_total-t_rest_last-t_double2, defaultGain, compliantGain, 0.0, 0.0);
                }
                else
                {
                    dobGain = DyrosMath::cubic(walking_tick, t_start+t_total-t_rest_last, t_start+t_total, compliantGain, defaultGain, 0.0, 0.0);
                }
                
            }            else
            {
                dobGain = defaultGain;   
            }  

            desired_leg_q(i) = desired_leg_q(i) - dobGain*dob_hat(i);
        }
        else
        {
            dobGain = defaultGain;

            if(foot_step(current_step_num,6) == 1)
            {
                if(walking_tick < t_start + t_total - t_rest_last - t_double2 - compliantTick)
                {
                    dobGain = defaultGain;
                }
                else if(walking_tick >= t_start + t_total - t_rest_last - t_double2 -compliantTick && walking_tick < t_start + t_total - t_rest_last - t_double2 )
                {
                    dobGain = DyrosMath::cubic(walking_tick, t_start+t_total-t_rest_last-t_double2-compliantTick, t_start+t_total-t_rest_last-t_double2, defaultGain, compliantGain, 0.0, 0.0);
                }
                else
                {
                    dobGain = DyrosMath::cubic(walking_tick, t_start+t_total-t_rest_last, t_start+t_total, compliantGain, defaultGain, 0.0, 0.0);
                }
                
            }
            else
            {
                dobGain = defaultGain;   
            }  

            desired_leg_q(i) = desired_leg_q(i) - dobGain*dob_hat(i);
        }        
    }
   
  /*  if(walking_tick >= t_start_real + t_double1 && walking_tick <= t_start_real + t_double1 + 0.01*Hz_)
    {
        K = DyrosMath::cubic(walking_tick, t_start_real + t_double1, t_start_real + t_double1 + 0.01*Hz_, 0.0, 0.9, 0.0, 0.0);
    }
    else if(walking_tick >= t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp - 0.01*Hz_)
    {
        K = DyrosMath::cubic(walking_tick, t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp - 0.01*Hz_, t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp, 0.9, 0.0, 0.0, 0.0);
    }
    else if(walking_tick>=t_start_real + t_double1 + 0.01*Hz_ && walking_tick <= t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp - 0.01*Hz_)
    {
        K = 0.9;
    }*/

   /* if(foot_step(current_step_num,6) == 1)
    {
        if(walking_tick >= t_start_real + t_double1 && walking_tick < t_start + t_total - t_double2 - t_rest_last) // the period for lifting the right foot
        {
            desired_leg_q(11) = desired_leg_q(11) - K*DyrosMath::rot2Euler((Robot.link_[Right_Foot].Rotm))(0); 
        }
    }
    else
    {
        if(walking_tick >= t_start_real + t_double1 && walking_tick < t_start + t_total - t_double2 - t_rest_last) // the period for lifting the right foot
        {
            desired_leg_q(5) = desired_leg_q(5) - K*DyrosMath::rot2Euler((Robot.link_[Left_Foot].Rotm))(0);
        }
    }*/

  
}

void Walking_controller::ankleOriControl(RobotData &Robot)
{
    Eigen::Vector2d k, kv;
    //2.5
    k(0) = -4.5;
    kv(0) = -0.003;
    k(1) = -2.5;
    kv(1) = -0.004;

    Eigen::Vector3d rf_e, lf_e, rf_e_vel, lf_e_vel;

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

    if(foot_step(current_step_num,6) == 1)
    {
        rf_e(0) = DyrosMath::rot2Euler(Robot.link_[Right_Foot].Rotm)(0);
        rf_e(1) = DyrosMath::rot2Euler(Robot.link_[Right_Foot].Rotm)(1);
        rf_e(2) = DyrosMath::rot2Euler(Robot.link_[Right_Foot].Rotm)(2);
        
        rf_e_vel = Robot.link_[Right_Foot].w;
        
        for(int i = 0; i<2; i++)
        {
            rf_e(i) = k(i) * rf_e(i) + kv(i)*rf_e_vel(i); 
        }
        
        RF_trajectory_float.linear() = DyrosMath::rotateWithY(rf_e(1))*DyrosMath::rotateWithX(rf_e(0));
    }
    else
    {
        lf_e(0) = DyrosMath::rot2Euler(Robot.link_[Left_Foot].Rotm)(0);
        lf_e(1) = DyrosMath::rot2Euler(Robot.link_[Left_Foot].Rotm)(1);
        lf_e(2) = DyrosMath::rot2Euler(Robot.link_[Left_Foot].Rotm)(2);

        lf_e_vel = Robot.link_[Left_Foot].w;

        for(int i = 0; i<2; i++)
        {
            lf_e(i) = k(i) * lf_e(i) + kv(i)*lf_e_vel(i); 
        }

        LF_trajectory_float.linear() = DyrosMath::rotateWithY(lf_e(1))*DyrosMath::rotateWithX(lf_e(0));
    }
}

void Walking_controller::setWalkingParameter(RobotData &Robot)
{
    desired_foot_step_num = 8;
    walking_tick = 0;
    t_rest_init = 0.1*Hz_;
    t_rest_last = 0.1*Hz_;
    t_double1 = 0.1*Hz_;
    t_double2 = 0.1*Hz_;
    t_total = 1.0*Hz_;
    t_temp = 4.0*Hz_;

/*    t_double1 = 0.35*Hz_;
    t_double2 = 0.35*Hz_;
    t_rest_init = .15*Hz_;
    t_rest_last = .15*Hz_;
    t_total= 2.0*Hz_;*/
    t_rest_temp = 0.0*Hz_;

    t_imp = 0.0*Hz_;
    t_last = t_total + t_temp;
    t_start = t_temp + 1;

    t_start_real = t_start + t_rest_init;
}

void Walking_controller::updateInitTime()
{
    walking_tick++;
}


/*
Kp: [2000.0, 3200.0, 2000.0, 3700.0, 3200.0, 3200.0, 
     2000.0, 3200.0, 2000.0, 3700.0, 3200.0, 3200.0, 
     4000.0, 4000.0, 4000.0,  
     400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 
     400.0, 200.0,
     400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0] 
Kv: [15.0, 24.0, 15.0, 25.0, 24.0, 24.0, 
     15.0, 24.0, 15.0, 25.0, 24.0, 24.0,
     100.0, 100.0, 100.0, 
     10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
     10.0, 5.0,
     10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
s
*/