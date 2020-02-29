#include "tocabi_controller/walking_pattern.h"

void WalkingPattern::footStepGenerator(Eigen::Vector4d target, Eigen::Vector3d foot_distance, int desired_foot_step_num, int first_foot_step_dir)
{

    if(target(0) == 0 && target(1) == 0 && target(3) == 0)
    {
        foot_step.resize(desired_foot_step_num,7);
        foot_step.setZero();

        for(int i=0; i<desired_foot_step_num; i++)
        {
            foot_step(2*i,1) = first_foot_step_dir * foot_distance(1) *(-1); 
            foot_step(2*i,6) = 1;
            first_foot_step_dir *= -1;
        }
    }
    else
    {
        foot_step_total(target, foot_distance, first_foot_step_dir);
    }
    
}

void WalkingPattern::foot_step_total(Eigen::Vector4d target, Eigen::Vector3d foot_distance, int first_foot_step_dir)
{
    double initial_rot, initial_drot, final_rot, final_drot;
    
    initial_rot = atan2(target(1),target(0));
    if(initial_rot>0.0)
        initial_drot = 10.0*DEG2RAD;
    else
        initial_drot = -10.0*DEG2RAD; 
    
    unsigned int init_total_step_num = initial_rot/initial_drot;
    double init_residual_angle = initial_rot - init_total_step_num * initial_drot;
    
    final_rot = target(4)-initial_rot;
    if(final_rot>0.0) 
        final_drot = 10.0*DEG2RAD;
    else
        final_drot = -10.0*DEG2RAD;

    unsigned int final_total_step_num = final_rot/final_drot;
    double final_residual_angle = final_rot - final_total_step_num * final_drot;
    double l = sqrt(target(0)*target(0)+target(1)*target(1));
    double dlength;//거리 추가 필요 
    int middle_total_step_num = l/dlength;
    double middle_residual_length = l-middle_total_step_num*dlength;
    
    total_step_num = init_total_step_num+middle_total_step_num+final_total_step_num;

    if(init_total_step_num != 0 || abs(init_residual_angle)>=0.0001)
    {
        if(init_total_step_num % 2 == 0)
            total_step_num = total_step_num + 2;
        else
        {
            if(abs(init_residual_angle)>=0.0001)
                total_step_num = total_step_num + 3;
            else
                total_step_num = total_step_num + 1;
        }   
    }

    if(middle_total_step_num != 0 || abs(middle_residual_length)>=0.0001)
    {
        if(middle_total_step_num%2 == 0)
            total_step_num = total_step_num + 2;
        else
        {
            if(abs(middle_residual_length)>=0.0001)
                total_step_num = total_step_num + 3;
            else
                total_step_num = total_step_num + 1;
        }   
    }

    if(final_total_step_num != 0 || abs(final_residual_angle)>=0.0001)
    {
        if(abs(final_residual_angle)>=0.0001)
            total_step_num = total_step_num + 2;
        else
            total_step_num = total_step_num + 1;   
    }

    foot_step.resize(total_step_num, 7);
    foot_step.setZero();

    int index = 0;

    int temp, temp2, temp3, is_right;

    if(first_foot_step_dir == 1)
        is_right = 1;
    else
        is_right = -1;


    temp = -is_right; //right foot will be first swingfoot
    temp2 = -is_right;
    temp3 = -is_right;

    if(init_total_step_num != 0 || abs(init_residual_angle)>=0.0001)
    {
        for(int i=0; i<init_total_step_num; i++)
        {
            temp *= -1;
            foot_step(index, 0) = temp*foot_distance(1)*sin((i+1)*initial_drot);
            foot_step(index, 1) = -temp*foot_distance(1)*cos((i+1)*initial_drot);
            foot_step(index, 5) = (i+1)*initial_drot;
            foot_step(index, 6) = 0.5+0.5*temp;
            index++;
        }

        if(temp == 1)
        {
            if(abs(init_residual_angle) >= 0.0001)  
            {
                foot_step(index,0) = temp*foot_distance(1)*sin((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)*cos((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_total_step_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;

                temp *= -1;

                foot_step(index,0) = temp*foot_distance(1)*sin((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)*cos((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_total_step_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;

                temp *= -1;

                foot_step(index,0) = temp*foot_distance(1)*sin((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)*cos((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_total_step_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;
            }
            else
            {
                temp *= -1;
                foot_step(index,0) = temp*foot_distance(1)*sin((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,1) = -temp*foot_distance(1)*cos((init_total_step_num)*initial_drot+init_residual_angle);
                foot_step(index,5) = (init_total_step_num)*initial_drot+init_residual_angle;
                foot_step(index,6) = 0.5+0.5*temp;
                index++;
            }  
        }
        else
        {
            temp *= -1;
            foot_step(index,0) = temp*foot_distance(1)*sin((init_total_step_num)*initial_drot+init_residual_angle);
            foot_step(index,1) = -temp*foot_distance(1)*cos((init_total_step_num)*initial_drot+init_residual_angle);
            foot_step(index,5) = (init_total_step_num)*initial_drot+init_residual_angle;
            foot_step(index,6) = 0.5+0.5*temp;
            index++;

            temp *= -1;

            foot_step(index,0) = temp*foot_distance(1)*sin((init_total_step_num)*initial_drot+init_residual_angle);
            foot_step(index,1) = -temp*foot_distance(1)*cos((init_total_step_num)*initial_drot+init_residual_angle);
            foot_step(index,5) = (init_total_step_num)*initial_drot+init_residual_angle;
            foot_step(index,6) = 0.5+0.5*temp;
            index++;
        }
    }

    if(temp2 == is_right)
    {
        for (int i =0 ; i<middle_total_step_num; i++)
        {
            temp2 *= -1;
            foot_step(index,0) = cos(initial_rot)*(dlength*(i+1))+temp2*sin(initial_rot)*(foot_distance(1));
            foot_step(index,1) = sin(initial_rot)*(dlength*(i+1))-temp2*cos(initial_rot)*(foot_distance(1));
            foot_step(index,5) = initial_rot;
            foot_step(index,6) = 0.5+0.5*temp2;
            index++;
        }

        if(temp2==is_right)
        {
            if(abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1));
                foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1));
                foot_step(index,5) = initial_rot;
                foot_step(index,6) = 0.5+0.5*temp2;
                index++;

                temp2 *= -1;

                foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1));
                foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1));
                foot_step(index,5) = initial_rot;
                foot_step(index,6) = 0.5+0.5*temp2;
                index++;

                temp2 *= -1;

                foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1));
                foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1));
                foot_step(index,5) = initial_rot;
                foot_step(index,6) = 0.5+0.5*temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
                foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
                foot_step(index,5) = initial_rot;
                foot_step(index,6) = 0.5+0.5*temp2;
                index++;
            }
        }
        else if(temp2==-is_right)
        {
            temp2 *= -1;

            foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1));
            foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1));
            foot_step(index,5) = initial_rot;
            foot_step(index,6) = 0.5+0.5*temp2;
            index++;

            temp2 *= -1;
            foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)+temp2*sin(initial_rot)*(foot_distance(1));
            foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_step_num)+middle_residual_length)-temp2*cos(initial_rot)*(foot_distance(1));
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

            foot_step(index,0) = final_position_x+temp3*foot_distance(1)*sin((i+1)*final_drot+initial_rot);
            foot_step(index,1) = final_position_y-temp3*foot_distance(1)*cos((i+1)*final_drot+initial_rot);
            foot_step(index,5) = (i+1)*final_drot+initial_rot;
            foot_step(index,6) = 0.5+0.5*temp3;
            index++;
        }

        if(abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step(index,0) = final_position_x+temp3*foot_distance(1)*sin(target(3));
            foot_step(index,1) = final_position_y-temp3*foot_distance(1)*cos(target(3));
            foot_step(index,5) = target(3);
            foot_step(index,6) = 0.5+0.5*temp3;
            index++;

            temp3 *= -1;

            foot_step(index,0) = final_position_x+temp3*foot_distance(1)*sin(target(3));
            foot_step(index,1) = final_position_y-temp3*foot_distance(1)*cos(target(3));
            foot_step(index,5) = target(3);
            foot_step(index,6) = 0.5+0.5*temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step(index,0) = final_position_x+temp3*foot_distance(1)*sin(target(3));
            foot_step(index,1) = final_position_y-temp3*foot_distance(1)*cos(target(3));
            foot_step(index,5) = target(3);
            foot_step(index,6) = 0.5+0.5*temp3;
            index++;
        }
    }
}

void WalkingPattern::cpPatternGeneration()
{

}

void WalkingPattern::setCpPosition()
{
    capturePoint_ox.resize(total_step_num+3);
    capturePoint_oy.resize(total_step_num+3);

    zmp_dx.resize(total_step_num+2);
    zmp_dy.resize(total_step_num+2);

    capturePoint_offsetx.resize(total_step_num+3);
    capturePoint_offsety.resize(total_step_num+3);

    /////////////////////TEMP 200228 JH

    capturePoint_ox(0) = COM_support_init.translation()(0) + capturePoint_offsetx(0);
    capturePoint_oy(0) = COM_float_init.translation()(0) + capturePoint_offsety(0);
    capturePoint_ox(total_step_num + 1) = foot_step(total_step_num-1,0) + capturePoint_offsetx(total_step_num + 1);
    capturePoint_oy(total_step_num + 1) = 0.0 + capturePoint_offsety(total_step_num + 1);
    capturePoint_ox(total_step_num + 2) = foot_step(total_step_num-1,0) + capturePoint_offsetx(total_step_num + 2);
    capturePoint_oy(total_step_num + 2) = 0.0 + capturePoint_offsety(total_step_num + 2);

    for(int i = 0; i<total_step_num+1; i++)
    {
        if(foot_step(0,6)==0) //right support
        {
            if(i == 0)
            {
                capturePoint_ox(1) = 0.0 + capturePoint_offsetx(1);
                capturePoint_oy(1) = -1*foot_step(0,1) + capturePoint_offsety(1);
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
                capturePoint_oy(1) = -1*foot_step(0,1) + capturePoint_offsety(1);
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
}