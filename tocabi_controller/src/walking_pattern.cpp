#include "tocabi_controller/walking_pattern.h"

void WalkingPattern::footStepGenerator()
{
    if(target(0) == 0 && target(1) == 0 && target(3) == 0)
    {
        foot_step.resize(desired_foot_step_num,7);
        foot_step.setZero();
        for(int i=0; i<desired_foot_step_num/2; i++)
        {
            foot_step(2*i,0) = 0.0;
            foot_step(2*i,1) = (-1)*foot_step_dir * foot_distance(1)/2.0; 
            foot_step(2*i,2) = 0.0;    
            foot_step(2*i,6) = 0.5+0.5*foot_step_dir;

            foot_step(2*i+1,0) = 0.0;
            foot_step(2*i+1,1) = foot_step_dir * foot_distance(1)/2.0; 
            foot_step(2*i+1,2) = 0.0;    
            foot_step(2*i+1,6) = 0.5+0.5*(-1)*foot_step_dir;
        }
    }
    else
    {
        footStepTotal();
    }
    total_step_num = foot_step.col(1).size();
}

void WalkingPattern::footStepTotal()
{
    double initial_rot, initial_drot, final_rot, final_drot;
    
    initial_rot = atan2(target(1),target(0));
    if(initial_rot>0.0)
        initial_drot = 10.0*DEG2RAD;
    else
        initial_drot = -10.0*DEG2RAD; 
    
    unsigned int init_totalstep_num = initial_rot/initial_drot;
    double init_residual_angle = initial_rot - init_totalstep_num * initial_drot;
    
    final_rot = target(3)-initial_rot;
    if(final_rot>0.0) 
        final_drot = 10.0*DEG2RAD;
    else
        final_drot = -10.0*DEG2RAD;

    unsigned int final_total_step_num = final_rot/final_drot;
    double final_residual_angle = final_rot - final_total_step_num * final_drot;
    double l = sqrt(target(0)*target(0)+target(1)*target(1));
    double dlength = step_length_x;//거리 추가 필요 
    int middle_total_step_num = l/dlength;
    double middle_residual_length = l-middle_total_step_num*dlength;
    int numberOfFootstep;
    int del_size = 1;
    numberOfFootstep = init_totalstep_num * del_size + middle_total_step_num * del_size + final_total_step_num * del_size;

    if(init_totalstep_num != 0 || abs(init_residual_angle)>=0.0001)
    {
        if(init_totalstep_num % 2 == 0)
            numberOfFootstep = numberOfFootstep + 2;
        else
        {
            if(abs(init_residual_angle)>=0.0001)
                numberOfFootstep = numberOfFootstep + 3;
            else
                numberOfFootstep = numberOfFootstep + 1;
        }   
    }

    if(middle_total_step_num != 0 || abs(middle_residual_length)>=0.0001)
    {
        if(middle_total_step_num%2 == 0)
            numberOfFootstep = numberOfFootstep + 2;
        else
        {
            if(abs(middle_residual_length)>=0.0001)
                numberOfFootstep = numberOfFootstep + 3;
            else
                numberOfFootstep = numberOfFootstep + 1;
        }   
    }

    if(final_total_step_num != 0 || abs(final_residual_angle)>=0.0001)
    {
        if(abs(final_residual_angle)>=0.0001)
            numberOfFootstep = numberOfFootstep + 2;
        else
            numberOfFootstep = numberOfFootstep + 1;   
    }
    foot_step.resize(numberOfFootstep, 7);
    foot_step.setZero();
 
    int index = 0;

    int temp, temp2, temp3, is_right;

    if(foot_step_dir == 1)
        is_right = 1;
    else
        is_right = -1;


    temp = -is_right; //right foot will be first swingfoot
    temp2 = -is_right;
    temp3 = -is_right;
    if(init_totalstep_num!=0 || abs(init_residual_angle)>=0.0001)
    {
        for (int i =0 ; i<init_totalstep_num; i++)
        {
            temp *= -1;
            foot_step(index,0) = temp*foot_distance(1)/2.0*sin((i+1)*initial_drot);
            foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((i+1)*initial_drot);
            foot_step(index,5) = (i+1)*initial_drot;
            foot_step(index,6) = 0.5+0.5*temp;
            index++;
        }

        if(temp==is_right)
        {
            if(abs(init_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step(index,0) = temp*foot_distance(1)/2.0*sin((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_totalstep_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;

                temp *= -1;

                foot_step(index,0) = temp*foot_distance(1)/2.0*sin((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_totalstep_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;

                temp *= -1;

                foot_step(index,0) = temp*foot_distance(1)/2.0*sin((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_totalstep_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;

            }
            else
            {
                temp *= -1;

                foot_step(index,0) = temp*foot_distance(1)/2.0*sin((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((init_totalstep_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_totalstep_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;
            }
        }
        else if(temp==-is_right)
        {
            temp *= -1;

            foot_step(index,0) = temp*foot_distance(1)/2.0*sin((init_totalstep_num)*initial_drot+init_residual_angle);
            foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((init_totalstep_num)*initial_drot+init_residual_angle);
            foot_step(index,5) = (init_totalstep_num)*initial_drot+init_residual_angle;
            foot_step(index,6) = 0.5+0.5*temp;
            index++;

            temp *= -1;

            foot_step(index,0) = temp*foot_distance(1)/2.0*sin((init_totalstep_num)*initial_drot+init_residual_angle);
            foot_step(index,1) = -temp*foot_distance(1)/2.0*cos((init_totalstep_num)*initial_drot+init_residual_angle);
            foot_step(index,5) = (init_totalstep_num)*initial_drot+init_residual_angle;
            foot_step(index,6) = 0.5+0.5*temp;
            index++;
        }
  }

  if(middle_total_step_num!=0 || abs(middle_residual_length)>=0.0001)
  {
      for (int i =0 ; i<middle_total_step_num; i++)
      {
          temp2 *= -1;

          foot_step(index,0) = cos(initial_rot)*(dlength*(i+1))+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
          foot_step(index,1) = sin(initial_rot)*(dlength*(i+1))-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
          foot_step(index,5) = initial_rot;
          foot_step(index,6) = 0.5+0.5*temp2;
          index++;
      }

      if(temp2==is_right)
      {
          if(abs(middle_residual_length) >= 0.0001)
          {
              temp2 *= -1;

              foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,5) = initial_rot;
              foot_step(index,6) = 0.5+0.5*temp2;
              index++;

              temp2 *= -1;

              foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,5) = initial_rot;
              foot_step(index,6) = 0.5+0.5*temp2;
              index++;

              temp2 *= -1;

              foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,5) = initial_rot;
              foot_step(index,6) = 0.5+0.5*temp2;
              index++;
          }
          else
          {
              temp2 *= -1;

              foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
              foot_step(index,5) = initial_rot;
              foot_step(index,6) = 0.5+0.5*temp2;
              index++;
          }
      }
      else if(temp2==-is_right)
      {
          temp2 *= -1;

          foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
          foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
          foot_step(index,5) = initial_rot;
          foot_step(index,6) = 0.5+0.5*temp2;
          index++;

          temp2 *= -1;

          foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1)/2.0);
          foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1)/2.0);
          foot_step(index,5) = initial_rot;
          foot_step(index,6) = 0.5+0.5*temp2;
          index++;
      }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length);

  if(final_total_step_num!=0 || abs(final_residual_angle)>= 0.0001)
  {
      for (int i =0 ; i<final_total_step_num; i++)
      {
          temp3 *= -1;

          foot_step(index,0) = final_position_x+temp3*foot_distance(1)/2.0*sin((i+1)*final_drot+initial_rot);
          foot_step(index,1) = final_position_y-temp3*foot_distance(1)/2.0*cos((i+1)*final_drot+initial_rot);
          foot_step(index,5) = (i+1)*final_drot+initial_rot;
          foot_step(index,6) = 0.5+0.5*temp3;
          index++;
      }

      if(abs(final_residual_angle) >= 0.0001)
      {
          temp3 *= -1;

          foot_step(index,0) = final_position_x+temp3*foot_distance(1)/2.0*sin(target(3));
          foot_step(index,1) = final_position_y-temp3*foot_distance(1)/2.0*cos(target(3));
          foot_step(index,5) = target(3);
          foot_step(index,6) = 0.5+0.5*temp3;
          index++;

          temp3 *= -1;

          foot_step(index,0) = final_position_x+temp3*foot_distance(1)/2.0*sin(target(3));
          foot_step(index,1) = final_position_y-temp3*foot_distance(1)/2.0*cos(target(3));
          foot_step(index,5) = target(3);
          foot_step(index,6) = 0.5+0.5*temp3;
          index++;
      }
      else
      {
          temp3 *= -1;

          foot_step(index,0) = final_position_x+temp3*foot_distance(1)/2.0*sin(target(3));
          foot_step(index,1) = final_position_y-temp3*foot_distance(1)/2.0*cos(target(3));
          foot_step(index,5) = target(3);
          foot_step(index,6) = 0.5+0.5*temp3;
          index++;
      }
  }
}

void WalkingPattern::changeFootSteptoLocal()
{
    Eigen::Isometry3d reference;
    foot_step_support.resize(total_step_num,7);

    if(current_step_num == 0)
    {
        if(foot_step(0,6) == 0) //right support
        {
            reference.translation() = RF_float_init.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(RF_float_init.linear())(2));
        }
        else  //left support
        {
            reference.translation() = LF_float_init.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(LF_float_init.linear())(2));
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step(current_step_num-1,5));
        for(int i=0 ;i<3; i++)
            reference.translation()(i) = foot_step(current_step_num-1,i);
    }
    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    if(current_step_num == 0)
    {
        for(int i=0; i<total_step_num; i++)
        {
            for(int j=0; j<3; j++)
                temp_global_position(j)  = foot_step(i,j);

                temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

            for(int j=0; j<3; j++)
                 foot_step_support(i,j) = temp_local_position(j);

            foot_step_support(i,3) = foot_step(i,3);
            foot_step_support(i,4) = foot_step(i,4);
            foot_step_support(i,5) = foot_step(i,5) -  SUF_float_initV(5);
        }
    }
    else
    {
        for(int i=0; i<total_step_num; i++)
        {
            for(int j=0; j<3; j++)
                temp_global_position(j)  = foot_step(i,j);

            temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

            for(int j=0; j<3; j++)
                foot_step_support(i,j) = temp_local_position(j);

            foot_step_support(i,3) = foot_step(i,3);
            foot_step_support(i,4) = foot_step(i,4);
            foot_step_support(i,5) = foot_step(i,5) - foot_step(current_step_num-1,5);
        }
    }

    for(int j=0;j<3;j++)
        temp_global_position(j) = SWF_float_init.translation()(j);

    temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

    for(int j=0;j<3;j++)
        SWF_support_initV(j) = temp_local_position(j);

        SWF_support_initV(3) = SWF_float_initV(3);
        SWF_support_initV(4) = SWF_float_initV(4);

    if(current_step_num == 0)
        SWF_support_initV(5) = SWF_float_initV(5) - SUF_float_initV(5);
    else
        SWF_support_initV(5) = SWF_float_initV(5) - foot_step(current_step_num-1,5);

    for(int j=0;j<3;j++)
        temp_global_position(j) = SUF_float_initV(j);

        temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

    for(int j=0;j<3;j++)
        SUF_support_initV(j) = temp_local_position(j);

        SUF_support_initV(3) = SUF_float_initV(3);
        SUF_support_initV(4) = SUF_float_initV(4);

    if(current_step_num == 0)
        SUF_support_initV(5) = 0;
    else
        SUF_support_initV(5) = SUF_float_initV(5) - foot_step(current_step_num-1,5);
}

void WalkingPattern::cpReferencePatternGeneration()
{
  capturePoint_refx.resize((t_total*(total_step_num+1)+t_temp-1));
  capturePoint_refy.resize((t_total*(total_step_num+1)+t_temp-1));
  com_refx.resize((t_total*(total_step_num+1)+t_temp-1));
  com_refy.resize((t_total*(total_step_num+1)+t_temp-1));
  zmp_refx.resize((t_total*(total_step_num+1)+t_temp-1));
  zmp_refy.resize((t_total*(total_step_num+1)+t_temp-1));


  for(int i=0; i<(t_total*(total_step_num+1)+t_temp-1); i++)
  {
    int current_step, capturePointChange;
    double tick;
    if(i<t_temp-1)
    {
        current_step=i/(t_temp+t_total);
        if(t_temp-t_total <= i)
        {  
            tick = (i-(t_temp-t_total-1))/Hz_; 
        }
        else
        {
            tick = i/(Hz_);
        }
        capturePointChange = i/(t_temp-1);
    }
   else
    {
        current_step = (i-t_temp-t_total)/(t_total)+1;
        capturePointChange = (i-t_temp+1)/(t_total)+1;
        tick = i/(Hz_)-t_total*(capturePointChange-1)/Hz_-(t_temp-1)/Hz_;
    }
    //ZMP trajectory from CP

    if(!(capturePointChange==total_step_num+1 && tick>(t_total)/Hz_)) //revise
    {
        if(capturePointChange == total_step_num+2)
        {
            capturePoint_refx(i) = exp(lipm_w*tick)*capturePoint_ox(capturePointChange-1)+(1-exp(lipm_w*tick))*zmp_dx(capturePointChange-1);
            capturePoint_refy(i) = exp(lipm_w*tick)*capturePoint_oy(capturePointChange-1)+(1-exp(lipm_w*tick))*zmp_dy(capturePointChange-1);
        }
        else
        {
            capturePoint_refx(i) = exp(lipm_w*tick)*capturePoint_ox(capturePointChange)+(1-exp(lipm_w*tick))*zmp_dx(capturePointChange);
            capturePoint_refy(i) = exp(lipm_w*tick)*capturePoint_oy(capturePointChange)+(1-exp(lipm_w*tick))*zmp_dy(capturePointChange);
        }
    }
    else
    {
        capturePoint_refx(i) = exp(lipm_w*t_total/Hz_)*capturePoint_ox(capturePointChange)+(1-exp(lipm_w*t_total/Hz_))*zmp_dx(capturePointChange);
        capturePoint_refy(i) = exp(lipm_w*t_total/Hz_)*capturePoint_oy(capturePointChange)+(1-exp(lipm_w*t_total/Hz_))*zmp_dy(capturePointChange);
    }
    if(capturePointChange==0 && i<t_temp-t_total)
    {
        capturePoint_refx(i) = capturePoint_ox(0);
        capturePoint_refy(i) = capturePoint_oy(0);
    }
    else if(capturePointChange==0 && t_temp-t_total <= i)
    {   
        capturePoint_refx(i) = exp(lipm_w*tick)*capturePoint_ox(capturePointChange)+(1-exp(lipm_w*tick))*zmp_dx(capturePointChange);
        capturePoint_refy(i) = exp(lipm_w*tick)*capturePoint_oy(capturePointChange)+(1-exp(lipm_w*tick))*zmp_dy(capturePointChange);
    }
    if(i == 0)
    {
        zmp_refx(0) = COM_support_init.translation()(0);
        zmp_refy(0) = COM_float_init.translation()(1);
    }
    else
    {
        zmp_refx(i) = (capturePoint_refx(i-1))-(capturePoint_refx(i)-capturePoint_refx(i-1))*Hz_/(lipm_w);
        zmp_refy(i) = (capturePoint_refy(i-1))-(capturePoint_refy(i)-capturePoint_refy(i-1))*Hz_/(lipm_w); 
    }
  }
}

void WalkingPattern::setCpPosition()
{
    capturePoint_ox.resize(total_step_num+3);
    capturePoint_oy.resize(total_step_num+3);

    zmp_dx.resize(total_step_num+2);
    zmp_dy.resize(total_step_num+2);

    b.resize(total_step_num+2);

    capturePoint_offsetx.resize(total_step_num+3);
    capturePoint_offsety.resize(total_step_num+3);

    /////// INITIALIZE //////

    capturePoint_offsetx.setZero();
    capturePoint_offsety.setZero();

    for(int i=0; i<total_step_num+2; i++)
    {
        b(i) = exp(lipm_w*t_total/Hz_);
    }
    /////////////////////TEMP 200228 JH

    capturePoint_ox(0) = COM_support_init.translation()(0) + capturePoint_offsetx(0);
    capturePoint_oy(0) = COM_float_init.translation()(1) + capturePoint_offsety(0);
    capturePoint_ox(total_step_num + 1) = foot_step(total_step_num-1,0) + capturePoint_offsetx(total_step_num + 1);
    capturePoint_oy(total_step_num + 1) = 0.0 + capturePoint_offsety(total_step_num + 1);
    capturePoint_ox(total_step_num + 2) = foot_step(total_step_num-1,0) + capturePoint_offsetx(total_step_num + 2);
    capturePoint_oy(total_step_num + 2) = 0.0 + capturePoint_offsety(total_step_num + 2);

    for(int i = 0; i<total_step_num; i++)
    {
        if(foot_step(0,6)==0) //right support
        {
            if(i == 0)
            {
                capturePoint_ox(1) = 0.0 + capturePoint_offsetx(1);
                capturePoint_oy(1) = 0.0 + capturePoint_offsety(1);
            }
            else
            {
                if(i % 2 == 0)
                {
                    capturePoint_ox(i+1) = foot_step(i-1,0) + capturePoint_offsetx(i+1);
                    capturePoint_oy(i+1) = foot_step(i-1,1) + capturePoint_offsety(i+1);
                }
                else
                {
                    capturePoint_ox(i+1) = foot_step(i-1,0) + capturePoint_offsetx(i+1);
                    capturePoint_oy(i+1) = foot_step(i-1,1) + capturePoint_offsety(i+1);
                }
            }
        }
        else
        {
            if(i == 0)
            {
                capturePoint_ox(1) = 0.0 + capturePoint_offsetx(1);
                capturePoint_oy(1) = 0.0 + capturePoint_offsety(1);
            }
            else
            {
                if(i % 2 == 0)
                {
                    capturePoint_ox(i+1) = foot_step(i-1,0) + capturePoint_offsetx(i+1);
                    capturePoint_oy(i+1) = foot_step(i-1,1) + capturePoint_offsety(i+1);
                }      
                else
                {
                    capturePoint_ox(i+1) = foot_step(i-1,0) + capturePoint_offsetx(i+1);
                    capturePoint_oy(i+1) = foot_step(i-1,1) + capturePoint_offsety(i+1);
                }
            }
        }
    }

    for(int i = 0; i<total_step_num+2; i++)
    {
        zmp_dx(i) = capturePoint_ox(i+1)/(1-b(i))-(b(i)*capturePoint_ox(i))/(1-b(i));
        zmp_dy(i) = capturePoint_oy(i+1)/(1-b(i))-(b(i)*capturePoint_oy(i))/(1-b(i));
    }
}

void WalkingPattern::cptoComTrajectory()
{
    for(int i=0; i<(t_total*(total_step_num+1)+t_temp-1); i++)
    {
        if(i>=t_temp-t_total)
        {
            com_refx(i) = lipm_w/Hz_*capturePoint_refx(i)+(1-lipm_w/Hz_)*com_refx(i-1);
            com_refy(i) = lipm_w/Hz_*capturePoint_refy(i)+(1-lipm_w/Hz_)*com_refy(i-1);
        }
        else
        {
            com_refx(i) = COM_support_init.translation()(0);
            com_refy(i) = COM_float_init.translation()(1);
        }      
    }
}

void WalkingPattern::setComTrajectory()
{
    double start_time;
    Eigen::Vector4d xyd, xd, yd, xs, ys;

    if(current_step_num == 0)
        start_time = 0;
    else
        start_time = t_start;

    if(walking_pattern == 0)
    {
        ///Preview
    }
    else if(walking_pattern == 1)
    {
        ///CapturePoint
        xyd.setZero();
        xyd(0) = com_refx(walking_tick);
        xyd(1) = com_refy(walking_tick);
        zmp_desired(0) = zmp_refx(walking_tick);
        zmp_desired(1) = zmp_refy(walking_tick);  
        xyd(3) = 1.;
        xyd = GlobaltoLocal_current * xyd;
        xd(0) = xyd(0);
        yd(0) = xyd(1);

        if(walking_tick == 0)
        {
            xd(1) = 0.0;
            xd(2) = 0.0;
            yd(1) = 0.0;
            yd(2) = 0.0;
        }
        else
        {
            xd(1) = COMV_support_currentV(0);
            xd(2) = COMV_support_currentV(0);
            yd(1) = COMV_support_currentV(1);
            yd(2) = COMV_support_currentV(1);
        }
        xs = xd;
        ys = yd;
    }
    else
    {
        ///MPC
    }

    if(walking_tick == t_start+t_total-1 && current_step_num != total_step_num - 1)
    {
        Eigen::Vector3d com_pos_prev, com_pos, com_vel_prev, com_vel, com_acc_prev, com_acc, temp_pos;
        Eigen::Matrix3d temp_rot;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support(current_step_num,5));
        for(int i=0; i<3; i++)
            temp_pos(i) = foot_step_support(current_step_num,i);

        com_pos_prev(0) = xs(0);        
        com_pos_prev(1) = ys(0);
        com_pos = temp_rot*(com_pos_prev - temp_pos);

        com_vel_prev(0) = xs(1);
        com_vel_prev(1) = ys(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot*com_vel_prev;

        com_acc_prev(0) = xs(2);
        com_acc_prev(1) = ys(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot*com_acc_prev;

        xs(0) = com_pos(0);
        ys(0) = com_pos(1);
        xs(1) = com_vel(0);
        ys(1) = com_vel(1);
        xs(2) = com_acc(0);
        ys(2) = com_acc(1);
    }

    if(com_control_mode == true)
    {
        com_desired(0) = xd(0);
        com_desired(1) = yd(0);
        com_desired(2) = DyrosMath::cubic(walking_tick, t_start, t_start_real, PELV_support_init.translation()(2), PELV_support_init.translation()(2), 0, 0);

        com_dot_desired(0) = xd(1);
        com_dot_desired(1) = yd(1);
        com_dot_desired(2) = DyrosMath::cubicDot(walking_tick, t_start, t_start_real, PELV_support_init.translation()(2), PELV_support_init.translation()(2), 0, 0, Hz_);

        p_ref(0) = xd(1)+com_gain*(xd(0)-COM_support_current.translation()(0));
        p_ref(1) = yd(1)+com_gain*(yd(0)-COM_support_current.translation()(1));
        p_ref(2) = com_gain*(com_desired(2)-COM_support_current.translation()(2));
        l_ref.setZero();
    }
    else
    {
        com_desired(0) = xd(0);
        com_desired(1) = yd(0);
        com_desired(2) = DyrosMath::cubic(walking_tick, t_start, t_start_real, PELV_support_init.translation()(2), PELV_support_init.translation()(2), 0, 0);

        com_dot_desired(0) = xd(1);
        com_dot_desired(1) = yd(1);
        com_dot_desired(2) = DyrosMath::cubicDot(walking_tick, t_start, t_start_real, PELV_support_init.translation()(2), PELV_support_init.translation()(2), 0, 0, Hz_);

        p_ref(0) = xd(1)+com_gain*(xd(0)-COM_support_current.translation()(0));
        p_ref(1) = yd(1)+com_gain*(yd(0)-COM_support_current.translation()(1));
        p_ref(2) = com_gain*(com_desired(2)-COM_support_current.translation()(2));
        l_ref.setZero();
    }
}

void WalkingPattern::setPelvisTrajectory()
{
    double z_rot = foot_step_support(current_step_num,5);

    //Trunk Position
    if(com_control_mode == true)
    {
        PELV_trajectory_support.translation()(0) = PELV_support_current.translation()(0) + pelvis_pgain*(com_desired(0) - COM_support_current.translation()(0));
        PELV_trajectory_support.translation()(1) = PELV_support_current.translation()(1) + pelvis_pgain*(com_desired(1) - COM_support_current.translation()(1));
        PELV_trajectory_support.translation()(2) = com_desired(2); //PELV_trajectory_support.translation()(2) + pelvis_pgain*(com_desired(2) - COM_support_current.translation()(2));
    }
    else
    {
        if(walking_tick >= t_start && walking_tick < t_start+0.3*Hz_)
        {
            pelvis_pgain = 0+ 3.0*(walking_tick-t_start)/(0.3*Hz_);
            pelvis_dgain = 0+ 0.5*(walking_tick-t_start)/(0.3*Hz_);
        }
        if(foot_step(current_step_num,6) == 1) //right foot swing(left foot support)
        {
            double temp_time = 0.1*Hz_;
            if(walking_tick < t_start_real)
                pelvis_offsetx = DyrosMath::cubic(walking_tick, t_start+temp_time,t_start_real-temp_time,0.0,0.02,0.0,0.0);
            else
                pelvis_offsetx = DyrosMath::cubic(walking_tick, t_start+t_total-t_rest_last+temp_time,t_start+t_total-temp_time,0.02,0.0,0.0,0.0);
        }
        PELV_trajectory_support.translation()(0) = PELV_support_current.translation()(0) + 1.0*(com_desired(0)-COM_support_current.translation()(0));
        PELV_trajectory_support.translation()(1) = PELV_support_current.translation()(1) + 1.0*(com_desired(1)-COM_support_current.translation()(1));
        PELV_trajectory_support.translation()(2) = com_desired(2);

        pelvis_pgain = 100.0;
        pelvis_dgain = 2000.0;
    }

    if(walking_tick < t_start_real + t_double1)
    {
        for(int i=0; i<2; i++)
            PELV_trajectory_euler(i) = DyrosMath::cubic(walking_tick, t_start, t_start_real+t_double1, PELV_support_euler_init(i),0.0,0.0,0.0);;
        PELV_trajectory_euler(2) = PELV_support_euler_init(2);
    }
  else if(walking_tick >= t_start_real + t_double1 && walking_tick < t_start + t_total - t_double2 - t_rest_last)
  {
    for(int i=0; i<2; i++)
      PELV_trajectory_euler(i) = 0.0;

    if(foot_step(current_step_num,6) == 2)
      PELV_trajectory_euler(2) = PELV_support_euler_init(2);
    else
      PELV_trajectory_euler(2) = DyrosMath::cubic(walking_tick, t_start_real + t_double1, t_start + t_total - t_double2 - t_rest_last, PELV_support_euler_init(2), z_rot/2.0, 0.0,0.0);
  }
  else
  {
    for(int i=0; i<2; i++)
      PELV_trajectory_euler(i) = 0.0;

    if(foot_step(current_step_num,6) == 2)
      PELV_trajectory_euler(2) = PELV_support_euler_init(2);
    else
      PELV_trajectory_euler(2) = z_rot/2.0;
  }
  PELV_trajectory_support.linear() = DyrosMath::rotateWithZ(PELV_trajectory_euler(2))*DyrosMath::rotateWithY(PELV_trajectory_euler(1))*DyrosMath::rotateWithX(PELV_trajectory_euler(0));
}

void WalkingPattern::setFootTrajectory()
{   double a = 0;
    for(int i=0; i<6; i++)
        SWF_target(i) = foot_step_support(current_step_num,i);

    if(walking_tick < t_start_real + t_double1)
    {
        LF_trajectory_support.translation() = LF_support_init.translation();
        LF_trajectory_dot_support.setZero();

        if(foot_step(current_step_num,6) == 1) //left foot support
            LF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick, t_start, t_start_real, LF_support_init.translation()(2),0.0,0.0,0.0);
        else // swing foot (right foot support)
        {
            if(current_step_num == 0)
                    LF_trajectory_support.translation()(2) = LF_support_init.translation()(2);
            else
            {
                if(walking_tick < t_start)
                    LF_trajectory_support.translation()(2) = LF_support_init.translation()(2);
                else if(walking_tick >= t_start && walking_tick < t_start_real)
                    LF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick, t_start, t_start_real, LF_support_init.translation()(2),0.0,0.0,0.0);
                else
                    LF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick, t_start_real, t_start_real + t_double1,0.0,0.0,0.0,0.0);
            }
        }

        LF_trajectory_euler_support = LF_support_euler_init;

        for(int i=0; i<2; i++)
            LF_trajectory_euler_support(i) = DyrosMath::cubic(walking_tick, t_start, t_start_real, LF_support_euler_init(i),0.0,0.0,0.0);

        LF_trajectory_support.linear() = DyrosMath::rotateWithZ(LF_trajectory_euler_support(2))*DyrosMath::rotateWithY(LF_trajectory_euler_support(1))*DyrosMath::rotateWithX(LF_trajectory_euler_support(0));

        RF_trajectory_support.translation() = RF_support_init.translation();
        RF_trajectory_dot_support.setZero();

        if(foot_step(current_step_num,6) == 0) //right foot support
        RF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick, t_start, t_start_real, RF_support_init.translation()(2),0.0,0.0,0.0);
        else // swing foot (left foot support)
        {
            if(current_step_num == 0)
                RF_trajectory_support.translation()(2) = RF_support_init.translation()(2);
            else
            {
                if(walking_tick < t_start)
                RF_trajectory_support.translation()(2) = RF_support_init.translation()(2);
                else if(walking_tick >= t_start && walking_tick < t_start_real)
                RF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick, t_start, t_start_real, RF_support_init.translation()(2),0.0,0.0,0.0);
                else
                RF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick, t_start_real, t_start_real + t_double1,0.0,0.0,0.0,0.0);
            }
        }

        RF_trajectory_euler_support = RF_support_euler_init;

        for(int i=0; i<2; i++)
            RF_trajectory_euler_support(i) = DyrosMath::cubic(walking_tick, t_start, t_start_real, RF_support_euler_init(i),0.0,0.0,0.0);

        RF_trajectory_support.linear() = DyrosMath::rotateWithZ(RF_trajectory_euler_support(2))*DyrosMath::rotateWithY(RF_trajectory_euler_support(1))*DyrosMath::rotateWithX(RF_trajectory_euler_support(0));
    }
    else if(walking_tick >= t_start_real + t_double1 && walking_tick < t_start + t_total - t_double2 - t_rest_last)
    { 
        double t_rest_temp = 0.05*Hz_;
        double ankle_temp = 0*DEG2RAD;
        
        if(foot_step(current_step_num,6) == 1) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
        {
            LF_trajectory_support.translation() = LF_support_init.translation();
            LF_trajectory_euler_support = LF_support_euler_init;
            LF_trajectory_euler_support.setZero();

            LF_trajectory_dot_support.setZero();
            LF_trajectory_support.linear() = DyrosMath::rotateWithZ(LF_trajectory_euler_support(2))*DyrosMath::rotateWithY(LF_trajectory_euler_support(1))*DyrosMath::rotateWithX(LF_trajectory_euler_support(0));

            if(walking_tick < t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp)/2.0) // the period for lifting the right foot
            {
                RF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0,foot_height,0.0,0.0);
                RF_trajectory_dot_support(2) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0,foot_height,0.0,0.0,Hz_);

                RF_trajectory_euler_support(1) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0.0,ankle_temp,0.0,0.0);
                RF_trajectory_dot_support(4) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0.0,ankle_temp,0.0,0.0,Hz_);
            } // the period for lifting the right foot
            else
            {
                RF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp,foot_height,SWF_target(2),0.0,0.0);
                RF_trajectory_dot_support(2) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp,foot_height,SWF_target(2),0.0,0.0,Hz_);

                RF_trajectory_euler_support(1) = DyrosMath::cubic(walking_tick,t_start+t_total-t_rest_last-t_double2-t_rest_temp,t_start+t_total-t_rest_last,ankle_temp,0.0,0.0,0.0);
                RF_trajectory_dot_support(4) = DyrosMath::cubicDot(walking_tick,t_start+t_total-t_rest_last-t_double2-t_rest_temp,t_start+t_total-t_rest_last,ankle_temp,0.0,0.0,0.0,Hz_);
            } // the period for putting the right foot

            RF_trajectory_euler_support(0) = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,0.0,SWF_target(3),0.0,0.0);
            RF_trajectory_dot_support(0+3) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,0.0,SWF_target(3),0.0,0.0,Hz_);

            for(int i=0; i<2; i++)
            {  a = 0.1;
                RF_trajectory_support.translation()(i) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+2*t_rest_temp,t_start+t_total-t_rest_last-t_double2-t_imp-2*t_rest_temp,RF_support_init.translation()(i),SWF_target(i),0.0,0.0);
                RF_trajectory_dot_support(i) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+2*t_rest_temp,t_start+t_total-t_rest_last-t_double2-t_imp-2*t_rest_temp,RF_support_init.translation()(i),SWF_target(i),0.0,0.0,Hz_);
            }
            RF_trajectory_euler_support(2) = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,RF_support_euler_init(2),SWF_target(5),0.0,0.0);
            RF_trajectory_dot_support(5) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,RF_support_euler_init(2),SWF_target(5),0.0,0.0,Hz_);
            RF_trajectory_support.linear() = DyrosMath::rotateWithZ(RF_trajectory_euler_support(2))*DyrosMath::rotateWithY(RF_trajectory_euler_support(1))*DyrosMath::rotateWithX(RF_trajectory_euler_support(0));
        }
        else if(foot_step(current_step_num,6) == 0) // Right foot support : Right foot is fixed at initial values, and Left foot is set to go target position
        {   a = -0.2;
            RF_trajectory_support.translation() = RF_support_init.translation();
            RF_trajectory_support.translation()(2) = 0.0;
            RF_trajectory_euler_support = RF_support_euler_init;
            RF_trajectory_euler_support(0) = 0.0;
            RF_trajectory_euler_support(1) = 0.0;
            RF_trajectory_dot_support.setZero();

            double ankle_temp;
            ankle_temp = 0*DEG2RAD;

            RF_trajectory_support.linear() = DyrosMath::rotateWithZ(RF_trajectory_euler_support(2))*DyrosMath::rotateWithY(RF_trajectory_euler_support(1))*DyrosMath::rotateWithX(RF_trajectory_euler_support(0));

            if(walking_tick < t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0)
            {
                LF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0,foot_height,0.0,0.0);
                LF_trajectory_dot_support(2) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0,foot_height,0.0,0.0,Hz_);
                LF_trajectory_euler_support(1) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0.0,ankle_temp,0.0,0.0);
                LF_trajectory_dot_support(4) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+t_rest_temp,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,0.0,ankle_temp,0.0,0.0,Hz_);
            }
            else
            {
                LF_trajectory_euler_support(1) = DyrosMath::cubic(walking_tick,t_start+t_total-t_rest_last-t_double2-t_rest_temp,t_start+t_total-t_rest_last,ankle_temp,0.0,0.0,0.0);
                LF_trajectory_dot_support(4) = DyrosMath::cubicDot(walking_tick,t_start+t_total-t_rest_last-t_double2-t_rest_temp,t_start+t_total-t_rest_last,ankle_temp,0.0,0.0,0.0,Hz_);
                LF_trajectory_support.translation()(2) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp,foot_height,SWF_target(2),0.0,0.0);
                LF_trajectory_dot_support(2) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+(t_total-t_rest_init-t_rest_last-t_double1-t_double2-t_imp)/2.0,t_start+t_total-t_rest_last-t_double2-t_imp-t_rest_temp,foot_height,SWF_target(2),0.0,0.0,Hz_);
            }

            LF_trajectory_euler_support(0) = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,0.0,SWF_target(3),0.0,0.0);
            LF_trajectory_dot_support(0+3) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,0.0,SWF_target(3),0.0,0.0,Hz_);

            for(int i=0; i<2; i++)
            {a = -0.1;
                LF_trajectory_support.translation()(i) = DyrosMath::cubic(walking_tick,t_start_real+t_double1+2*t_rest_temp,t_start+t_total-t_rest_last-t_double2-t_imp-2*t_rest_temp,LF_support_init.translation()(i),SWF_target(i),0.0,0.0);
                LF_trajectory_dot_support(i) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1+2*t_rest_temp,t_start+t_total-t_rest_last-t_double2-t_imp-2*t_rest_temp,LF_support_init.translation()(i),SWF_target(i),0.0,0.0,Hz_);
            }

            LF_trajectory_euler_support(2) = DyrosMath::cubic(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,LF_support_euler_init(2),SWF_target(5),0.0,0.0);
            LF_trajectory_dot_support(5) = DyrosMath::cubicDot(walking_tick,t_start_real+t_double1,t_start+t_total-t_rest_last-t_double2-t_imp,LF_support_euler_init(2),SWF_target(5),0.0,0.0,Hz_);

            LF_trajectory_support.linear() = DyrosMath::rotateWithZ(LF_trajectory_euler_support(2))*DyrosMath::rotateWithY(LF_trajectory_euler_support(1))*DyrosMath::rotateWithX(LF_trajectory_euler_support(0));
        }
        else
        {a = 0.2;
            LF_trajectory_support.translation() = LF_support_init.translation();
            LF_trajectory_support.linear() = LF_support_init.linear();
            LF_trajectory_euler_support = LF_support_euler_init;
            LF_trajectory_dot_support.setZero();

            RF_trajectory_support.translation() = RF_support_init.translation();
            RF_trajectory_support.linear() = RF_support_init.linear();
            RF_trajectory_euler_support = RF_support_euler_init;
            RF_trajectory_dot_support.setZero();
        }
    }
    else
    {
        if(foot_step(current_step_num,6) == 1)
        {a = 0.5;
            LF_trajectory_support.translation() = LF_support_init.translation();
            LF_trajectory_support.translation()(2) = 0.0;
            LF_trajectory_euler_support = LF_support_euler_init;
            LF_trajectory_euler_support(0) = 0.0;
            LF_trajectory_euler_support(1) = 0.0;
            LF_trajectory_support.linear() = DyrosMath::rotateWithZ(LF_trajectory_euler_support(2))*DyrosMath::rotateWithY(LF_trajectory_euler_support(1))*DyrosMath::rotateWithX(LF_trajectory_euler_support(0));
            LF_trajectory_dot_support.setZero();

            for(int i=0; i<3; i++)
            {   a = 0.3;
                RF_trajectory_support.translation()(i) = SWF_target(i);
                RF_trajectory_euler_support(i) = SWF_target(i+3);
            }
            RF_trajectory_dot_support.setZero();

            RF_trajectory_support.linear() = DyrosMath::rotateWithZ(RF_trajectory_euler_support(2))*DyrosMath::rotateWithY(RF_trajectory_euler_support(1))*DyrosMath::rotateWithX(RF_trajectory_euler_support(0));
        }
        else if (foot_step(current_step_num,6) == 0)
        {
            RF_trajectory_support.translation() = RF_support_init.translation();
            RF_trajectory_support.translation()(2) = 0.0;
            RF_trajectory_euler_support = RF_support_euler_init;
            RF_trajectory_euler_support(0) = 0.0;
            RF_trajectory_euler_support(1) = 0.0;
            RF_trajectory_dot_support.setZero();
            RF_trajectory_support.linear() = DyrosMath::rotateWithZ(RF_trajectory_euler_support(2))*DyrosMath::rotateWithY(RF_trajectory_euler_support(1))*DyrosMath::rotateWithX(RF_trajectory_euler_support(0));

            for(int i=0; i<3; i++)
            {a = 0.4;
                LF_trajectory_support.translation()(i) = SWF_target(i);
                LF_trajectory_euler_support(i) = SWF_target(i+3);
            }
            LF_trajectory_dot_support.setZero();
            LF_trajectory_support.linear() = DyrosMath::rotateWithZ(LF_trajectory_euler_support(2))*DyrosMath::rotateWithY(LF_trajectory_euler_support(1))*DyrosMath::rotateWithX(LF_trajectory_euler_support(0));
        }
        else
        {
            LF_trajectory_support.translation() = LF_support_init.translation();
            LF_trajectory_support.linear() = LF_support_init.linear();
            LF_trajectory_euler_support = LF_support_euler_init;
            LF_trajectory_dot_support.setZero();
            RF_trajectory_support.translation() = RF_support_init.translation();
            RF_trajectory_support.linear() = RF_support_init.linear();
            RF_trajectory_euler_support = RF_support_euler_init;
            RF_trajectory_dot_support.setZero();
        }
    }
}

void WalkingPattern::supportToFloatPattern()
{
    if(gyro_frame_flag == true)
    {
        Eigen::Isometry3d reference = PELV_trajectory_float;
        DyrosMath::floatGyroframe(PELV_trajectory_support,reference,PELV_trajectory_float);
        DyrosMath::floatGyroframe(PELV_trajectory_support,reference,LF_trajectory_float);
        DyrosMath::floatGyroframe(PELV_trajectory_support,reference,RF_trajectory_float);
        LF_trajectory_euler_float = DyrosMath::rot2Euler(LF_trajectory_float.linear());
        RF_trajectory_euler_float = DyrosMath::rot2Euler(RF_trajectory_float.linear());
    }
    else
    {
        PELV_trajectory_float = DyrosMath::inverseIsometry3d(PELV_trajectory_support)*PELV_trajectory_support;
        LF_trajectory_float = DyrosMath::inverseIsometry3d(PELV_trajectory_support)*LF_trajectory_support;
        RF_trajectory_float = DyrosMath::inverseIsometry3d(PELV_trajectory_support)*RF_trajectory_support;
        LF_trajectory_euler_float = DyrosMath::rot2Euler(LF_trajectory_float.linear());
        RF_trajectory_euler_float = DyrosMath::rot2Euler(RF_trajectory_float.linear());
    }
}

void WalkingPattern::floatToSupportPattern()
{

}

void WalkingPattern::referenceFrameChange()
{
    Eigen::Isometry3d reference;

    if(current_step_num == 0)
    {
        if(foot_step(0,6) == 0) //right support
        {
            reference.translation() = RF_float_init.translation();
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(RF_float_init.linear())(2));
        }
        else  //left support
        {
            reference.translation() = LF_float_init.translation();
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(LF_float_init.linear())(2));
        }
        if (walking_tick == 0)
        {
            reference.translation() = reference.translation();
            reference.linear() =  reference.linear();
            LocaltoGlobal_current = reference;
            GlobaltoLocal_current = DyrosMath::inverseIsometry3d(reference);
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step(current_step_num-1,5));
        for(int i=0 ;i<3; i++)
        reference.translation()(i) = foot_step(current_step_num-1,i);
        if(walking_tick == t_start)
        {
            reference.translation() = reference.translation();
            reference.linear() =  reference.linear();
            LocaltoGlobal_current = reference;
            GlobaltoLocal_current = DyrosMath::inverseIsometry3d(reference);
         }
    }  
}