#include "tocabi_controller/walking_controller.h"

Walking_controller::Walking_controller(DataContainer &dc_global, RobotData &kind) : dc(dc_global), rk_(kind)
{
    walking_tick = 0;
    setRobotStateInitialize();

    for(int i=0; i<1;i++)
    {
      file[i].open(FILE_NAMES[i].c_str(),std::ios_base::out);
    }
}

void Walking_controller::walkingCompute()
{
    footStepGenerator();
    getRobotInitState();
    getRobotState();
    chagneFootSteptoLocal();
    setCpPosition();
    cpReferencePatternGeneration();

    updateNextStepTime();
}

void Walking_controller::inverseKinematics()
{

}

void Walking_controller::setInitialPose()
{

}

void Walking_controller::getRobotState()
{
    if(walking_tick == 0)
    {

    }
  
    //////Real Robot Float Frame//////
    RF_float_current.translation() = dc.link_[Right_Foot].xpos;
    RF_float_current.linear() = dc.link_[Right_Foot].Rotm;
    LF_float_current.translation() = dc.link_[Left_Foot].xpos;
    LF_float_current.linear() = dc.link_[Left_Foot].Rotm;
    PELV_float_current.translation() = dc.link_[Pelvis].xpos;
    PELV_float_current.linear() = dc.link_[Pelvis].Rotm;

    COM_float_current.translation() = dc.link_[COM_id].xpos;
    COM_float_current.linear() = dc.link_[COM_id].Rotm;

    COMV_support_currentV = dc.com_.vel;
    COMV_support_currentV = dc.com_.accel;

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
    RF_support_current = DyrosMath::inverseIsometry3d(RF_float_current);
    LF_support_current = DyrosMath::inverseIsometry3d(LF_float_current);
    PELV_support_current = DyrosMath::inverseIsometry3d(SUF_float_current);
    COM_support_current =  DyrosMath::multiplyIsometry3d(PELV_support_current, COM_float_current);
}

void Walking_controller::getRobotInitState()
{
    if(walking_tick == 0)
    {
        RF_float_init.translation() = dc.link_[Right_Foot].xpos;
        RF_float_init.linear() = dc.link_[Right_Foot].Rotm;
        LF_float_init.translation() = dc.link_[Left_Foot].xpos;
        LF_float_init.linear() = dc.link_[Left_Foot].Rotm;

        COM_float_init.translation() = dc.com_.pos;
        COM_float_init.linear() = dc.link_[COM_id].Rotm;

        PELV_float_init.translation() = dc.link_[Pelvis].xpos;
        PELV_float_init.linear() = dc.link_[Pelvis].Rotm;
        
        if(foot_step(0,6) == 0)
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
        RF_support_init = DyrosMath::inverseIsometry3d(RF_float_init);
        LF_support_init = DyrosMath::inverseIsometry3d(LF_float_init);
        PELV_support_init = DyrosMath::inverseIsometry3d(SUF_float_init);
        COM_support_init =  DyrosMath::multiplyIsometry3d(PELV_support_init, COM_float_init);
    
        zc = COM_support_init.translation()(2);
        lipm_w = sqrt(GRAVITY/zc);
    }
    else if(current_step_num!=0 && walking_tick == t_start)
    {
        RF_float_init.translation() = dc.link_[Right_Foot].xpos;
        RF_float_init.linear() = dc.link_[Right_Foot].Rotm;
        LF_float_init.translation() = dc.link_[Left_Foot].xpos;
        LF_float_init.linear() = dc.link_[Left_Foot].Rotm;

        COM_float_init.translation() = dc.com_.pos;
        COM_float_init.linear() = dc.link_[COM_id].Rotm;

        PELV_float_init.translation() = dc.link_[Pelvis].xpos;
        PELV_float_init.linear() = dc.link_[Pelvis].Rotm;

        if(foot_step(0,6) == 0)
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
        RF_support_init = DyrosMath::inverseIsometry3d(RF_float_init);
        LF_support_init = DyrosMath::inverseIsometry3d(LF_float_init);
        PELV_support_init = DyrosMath::inverseIsometry3d(SUF_float_init);
        COM_support_init =  DyrosMath::multiplyIsometry3d(PELV_support_init, COM_float_init);
    
        RF_support_euler_init = DyrosMath::rot2Euler(RF_support_init.linear());
        LF_support_euler_init = DyrosMath::rot2Euler(LF_support_init.linear());
        PELV_support_euler_init = DyrosMath::rot2Euler(PELV_support_init.linear());

        zc = COM_support_init.translation()(2);
        lipm_w = sqrt(GRAVITY/zc);
    }
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
  walking_tick ++;
}

void Walking_controller::getUiWalkingParameter(WalkingCommand wtc, int controller_Hz)
{
    ik_mode = wtc.ik_mode;
    walking_pattern = wtc.walking_pattern;
    if(wtc.foot_step_dir == 0)
    {
        foot_step_dir = 1.0;
    }
    else
    {
        foot_step_dir = -1.0;
    }
    target(0) = wtc.target_x;
    target(1) = wtc.target_y;
    target(2) = wtc.target_z;
    target(3) = wtc.theta;
    height = wtc.height;
    step_length_y = wtc.step_length_y;
    step_length_x = wtc.step_length_x;
    dob = wtc.dob;
    Hz_ = controller_Hz;
    dt = 1/Hz_;
    foot_height = 0.05;
    com_control_mode = true;
    gyro_frame_flag = false;
    if(com_control_mode == true)
    {
        pelvis_pgain = 0.15;
        com_gain = 100.0;
    }
    else
    {
        pelvis_pgain = 3.0;
        pelvis_dgain = 0.5;
        com_gain = 100.0;
    }
    setWalkingParameter();
}

void Walking_controller::setWalkingParameter()
{
    desired_foot_step_num = 10;
    foot_distance = dc.link_[Left_Foot].xpos -  dc.link_[Right_Foot].xpos; 

    t_rest_init = 0.05*Hz_;
    t_rest_last = 0.05*Hz_;
    t_double1 = 0.10*Hz_;
    t_double2 = 0.10*Hz_;
    t_total = 1.2*Hz_;
    t_temp = 3.0*Hz_;
    t_imp = 0.0*Hz_;
    t_last = t_total + t_temp;
    t_start = t_temp + 1;

    t_start_real = t_start + t_rest_init;

}