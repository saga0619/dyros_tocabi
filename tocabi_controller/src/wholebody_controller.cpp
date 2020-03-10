#include "tocabi_controller/wholebody_controller.h"

//Left Foot is first! LEFT = 0, RIGHT = 1 !
// #include "cvxgen/solver.h"

// Vars vars;
// Params params;
// Workspace work;(DataContainer &dc, RobotData &kd_);
// Settings settings;
void WholebodyController::init(RobotData &Robot)
{
    Robot.contact_calc = false;
    Robot.task_force_control = false;
    Robot.task_force_control_feedback = false;
    Robot.zmp_control = false;
    Robot.mpc_init = false;
    Robot.task_force_control = false;
    Robot.task_force_control_feedback = false;
    Robot.zmp_control = false;
    Robot.zmp_feedback_control = false;

    Robot.Grav_ref.setZero(3);
    Robot.Grav_ref(2) = -9.81;
}

void WholebodyController::update(RobotData &Robot)
{
    Robot.A_matrix = Robot.A_;
    Robot.A_matrix_inverse = Robot.A_.inverse();
}
/*
void WholebodyController::contact_set(int contact_number, int link_id[])
{
    J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
    for (int i = 0; i < contact_number; i++)
    {
        rk_.link_[link_id[i]].Set_Contact(current_q_, rk_.link_[link_id[i]].contact_point);
        J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = rk_.link_[link_id[i]].Jac_Contact;
    }
    Lambda_c = (J_C * A_matrix_inverse * (J_C.transpose())).inverse();
    J_C_INV_T = Lambda_c * J_C * A_matrix_inverse;
    N_C.setZero(MODEL_DOF + 6, MODEL_DOF + 6);
    I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    N_C = I37 - J_C.transpose() * J_C_INV_T;
    Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Slc_k_T = Slc_k.transpose();
    //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
    W = Slc_k * A_matrix_inverse * N_C * Slc_k_T; //2 types for w matrix
    W_inv = DyrosMath::pinv_SVD(W);
    contact_force_predict.setZero();
}*/

void WholebodyController::zmp_feedback_control(Vector3d desired_zmp)
{
}

void WholebodyController::set_contact(RobotData &Robot)
{

    Robot.contact_index = 0;
    if (Robot.ee_[0].contact)
    {
        Robot.contact_part[Robot.contact_index] = Left_Foot;
        Robot.contact_index++;
    }
    if (Robot.ee_[1].contact)
    {
        Robot.contact_part[Robot.contact_index] = Right_Foot;
        Robot.contact_index++;
    }
    /* if (right_hand)
    {
        contact_part[contact_index] = Right_Hand;
        contact_index++;
    }
    if (left_hand)
    {
        contact_part[contact_index] = Left_Hand;
        contact_index++;
    }*/
    //contact_set(contact_index, contact_part);

    Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Robot.link_[Robot.contact_part[i]].Set_Contact(Robot.q_virtual_, Robot.link_[Robot.contact_part[i]].contact_point);
        Robot.link_[Robot.contact_part[i]].Set_Sensor_Position(Robot.q_virtual_, Robot.link_[Robot.contact_part[i]].sensor_point);
        Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.link_[Robot.contact_part[i]].Jac_Contact;
    }
    Robot.Lambda_c = (Robot.J_C * Robot.A_matrix_inverse * (Robot.J_C.transpose())).inverse();
    Robot.J_C_INV_T = Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse;
    Robot.N_C.setZero(MODEL_DOF + 6, MODEL_DOF + 6);
    Robot.I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    Robot.N_C = Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T;
    Robot.Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Robot.Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Robot.Slc_k_T = Robot.Slc_k.transpose();
    //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
    Robot.W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; //2 types for w matrix
    Robot.W_inv = DyrosMath::pinv_SVD(Robot.W);
    Robot.contact_force_predict.setZero();
    Robot.contact_calc = true;
}

void WholebodyController::set_contact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand, bool right_hand)
{
    Robot.ee_[0].contact = left_foot;
    Robot.ee_[1].contact = right_foot;
    Robot.ee_[2].contact = left_hand;
    Robot.ee_[3].contact = right_hand;

    Robot.contact_index = 0;
    if (left_foot)
    {
        Robot.contact_part[Robot.contact_index] = Left_Foot;
        Robot.ee_idx[Robot.contact_index] = 0;
        Robot.contact_index++;
    }
    if (right_foot)
    {
        Robot.contact_part[Robot.contact_index] = Right_Foot;
        Robot.ee_idx[Robot.contact_index] = 1;
        Robot.contact_index++;
    }
    if (left_hand)
    {
        Robot.contact_part[Robot.contact_index] = Left_Hand;
        Robot.ee_idx[Robot.contact_index] = 2;
        Robot.contact_index++;
    }
    if (right_hand)
    {
        Robot.contact_part[Robot.contact_index] = Right_Hand;
        Robot.ee_idx[Robot.contact_index] = 3;
        Robot.contact_index++;
    }
    //contact_set(contact_index, contact_part);

    Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Robot.link_[Robot.contact_part[i]].Set_Contact(Robot.q_virtual_, Robot.link_[Robot.contact_part[i]].contact_point);
        Robot.link_[Robot.contact_part[i]].Set_Sensor_Position(Robot.q_virtual_, Robot.link_[Robot.contact_part[i]].sensor_point);
        Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.link_[Robot.contact_part[i]].Jac_Contact;
    }

    Robot.ee_[0].cp_ = Robot.link_[Left_Foot].xpos_contact;
    Robot.ee_[1].cp_ = Robot.link_[Right_Foot].xpos_contact;
    Robot.ee_[2].cp_ = Robot.link_[Left_Hand].xpos_contact;
    Robot.ee_[3].cp_ = Robot.link_[Right_Hand].xpos_contact;

    Robot.ee_[0].xpos = Robot.link_[Left_Foot].xpos;
    Robot.ee_[1].xpos = Robot.link_[Right_Foot].xpos;
    Robot.ee_[2].xpos = Robot.link_[Left_Hand].xpos;
    Robot.ee_[3].xpos = Robot.link_[Right_Hand].xpos;

    Robot.ee_[0].rotm = Robot.link_[Left_Foot].Rotm;
    Robot.ee_[1].rotm = Robot.link_[Right_Foot].Rotm;
    Robot.ee_[2].rotm = Robot.link_[Left_Hand].Rotm;
    Robot.ee_[3].rotm = Robot.link_[Right_Hand].Rotm;

    Robot.ee_[0].sensor_xpos = Robot.link_[Left_Foot].xpos_sensor;
    Robot.ee_[1].sensor_xpos = Robot.link_[Right_Foot].xpos_sensor;
    Robot.ee_[2].sensor_xpos = Robot.link_[Left_Hand].xpos_sensor;
    Robot.ee_[3].sensor_xpos = Robot.link_[Right_Hand].xpos_sensor;

    Robot.ee_[0].cs_x_length = 0.12;
    Robot.ee_[0].cs_y_length = 0.04;
    Robot.ee_[1].cs_x_length = 0.12;
    Robot.ee_[1].cs_y_length = 0.04;
    Robot.ee_[2].cs_x_length = 0.02;
    Robot.ee_[2].cs_y_length = 0.02;
    Robot.ee_[3].cs_x_length = 0.02;
    Robot.ee_[3].cs_y_length = 0.02;

    Robot.Lambda_c = (Robot.J_C * Robot.A_matrix_inverse * (Robot.J_C.transpose())).inverse();
    Robot.J_C_INV_T = Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse;
    Robot.N_C.setZero(MODEL_DOF + 6, MODEL_DOF + 6);
    Robot.I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    Robot.N_C = Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T;
    Robot.Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Robot.Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Robot.Slc_k_T = Robot.Slc_k.transpose();
    //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
    Robot.W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; //2 types for w matrix
    Robot.W_inv = DyrosMath::pinv_SVD(Robot.W);
    Robot.contact_force_predict.setZero();
    Robot.contact_calc = true;
}

Matrix2d matpower(Matrix2d mat, int i)
{
    Matrix2d m;
    m.setIdentity();
    if (i == 0)
    {
        return m;
    }
    else
    {
        for (int j = 0; j < i; j++)
        {
            m = m * mat;
        }
    }
    return m;
}

Vector2d WholebodyController::getcpref(RobotData &Robot, double task_time, double future_time)
{
    double time_segment = 1.0;
    double step_length = 0.1;
    double w_ = sqrt(9.81 / 0.081);
    double b_ = exp(w_ * (time_segment));

    Vector2d CP_ref[9];
    CP_ref[0] << -0.04, 0;
    CP_ref[1] << -0.04, 0.096256;
    CP_ref[2] << 0.06, -0.096256;
    CP_ref[3] << 0.16, 0.096256;
    CP_ref[4] << 0.26, -0.096256;
    CP_ref[5] << 0.36, 0.096256;
    CP_ref[6] << 0.46, -0.096256;
    CP_ref[7] << 0.46, 0.0;
    CP_ref[8] << 0.46, 0.0;

    Vector2d ZMP_ref[8];
    for (int i = 0; i < 8; i++)
    {
        ZMP_ref[i] = 1 / (1 - b_) * CP_ref[i + 1] - b_ / (1 - b_) * CP_ref[i];
    }
    //non gui mode?

    //(int)time
    double left_time = 1 - task_time + (int)task_time;

    double t_see;

    t_see = task_time + future_time;
    Vector2d CP_t;
    CP_t = exp(w_ * (t_see - (int)t_see)) * CP_ref[(int)t_see] + (1.0 - exp(w_ * (t_see - (int)t_see))) * ZMP_ref[(int)t_see];

    if ((task_time - (int)task_time) + future_time < 1)
    {
        double b = exp(w_ * left_time);
        Vector2d zmp_ = 1 / (1 - b) * CP_ref[(int)task_time + 1] - b / (1 - b) * Robot.com_.CP;

        CP_t = exp(w_ * future_time) * Robot.com_.CP + (1.0 - exp(w_ * future_time)) * zmp_;
    }

    return CP_t;
}

// Vector2d WholebodyController::getcptraj(double time, Vector2d zmp) //task_time
// {
//   double time_segment = 1.0;
//   double step_length = 0.1;

//   int n_sample = 30;
//   double t_sample = 0.005; //milliseconds

//   double task_time = control_time_ - tc_.taskcommand_.command_time;

//   double w_ = sqrt(9.81 / 0.81);

//   Matrix2d A, B;
//   A << exp(w_ * t_sample), 0, 0, exp(w_ * t_sample);
//   B << 1 - exp(w_ * t_sample), 0, 0, 1 - exp(w_ * t_sample);

//   MatrixXd F_xi, F_p, F_p_temp;
//   F_xi.setZero(n_sample * 2, 2);
//   F_p.setZero(n_sample * 2, n_sample * 2);
//   F_p_temp.setZero(n_sample * 2, 2);

//   for (int i = 0; i < n_sample; i++)
//   {
//     F_xi.block(i * 2, 0, 2, 2) = matpower(A, i + 1);
//     for (int j = i; j < n_sample; j++)
//     {
//       F_p.block(j * 2, i * 2, 2, 2) = matpower(A, j - i) * B;
//     }
//   }

//   MatrixXd THETA;
//   THETA.setIdentity(n_sample * 2, n_sample * 2);

//   Matrix2d I2;
//   I2.setIdentity();

//   for (int i = 0; i < n_sample - 1; i++)
//   {
//     THETA.block(i * 2 + 2, i * 2, 2, 2) = -I2;
//   }

//   MatrixXd e1;
//   e1.setZero(2 * n_sample, 2);
//   e1.block(0, 0, 2, 2) = I2;

//   double q_par = 1.0;
//   double r_par = 0.4;

//   MatrixXd H, I_nsample;
//   I_nsample.setIdentity(2 * n_sample, 2 * n_sample);
//   MatrixXd Q, R;
//   Q = q_par * I_nsample;
//   R = r_par * I_nsample;

//   H = THETA.transpose() * R * THETA + F_p.transpose() * Q * F_p;

//   VectorXd g;
//   VectorXd cp_ref_t;
//   cp_ref_t.setZero(n_sample * 2);
//   for (int i = 0; i < n_sample; i++)
//   {
//     cp_ref_t.segment(i * 2, 2) = getcpref(time, i * t_sample);
//   }
//   std::cout << " pk-1" << std::endl;
//   std::cout << p_k_1 << std::endl;
//   std::cout << " cp_ref_t : " << std::endl;
//   std::cout << cp_ref_t << std::endl;
//   std::cout << " com_CP : " << com_.CP << std::endl;
//   std::cout << " g1 : " << std::endl
//             << F_p.transpose() * Q * (F_xi * com_.CP.segment(0, 2) - cp_ref_t) << std::endl;
//   std::cout << " g2 : " << std::endl
//             << THETA.transpose() * R * e1 * p_k_1 << std::endl;

//   g = F_p.transpose() * Q * (F_xi * com_.CP.segment(0, 2) - cp_ref_t) - THETA.transpose() * R * e1 * p_k_1;

//   VectorXd r(n_sample * 2);
//   Vector2d sf[8];
//   sf[0] << -0.04, 0;
//   sf[1] << -0.04, 0.1024;
//   sf[2] << 0.06, -0.1024;
//   sf[3] << 0.16, 0.1024;
//   sf[4] << 0.26, -0.1024;
//   sf[5] << 0.36, 0.1024;
//   sf[6] << 0.46, -0.1024;
//   sf[7] << 0.46, 0.0;

//   for (int i = 0; i < n_sample; i++)
//   {
//     r.segment(i * 2, 2) = sf[(int)(time + t_sample * i)];
//   }

//   VectorXd lb, ub, f;
//   f.setZero(n_sample * 2);

//   for (int i = 0; i < n_sample * 2; i++)
//   {
//     f(i) = 0.1;
//   }
//   lb = lb.setZero(n_sample * 2);
//   lb = r - f;
//   ub = ub.setZero(n_sample * 2);
//   ub = r + f;

//   Vector2d res_2;
//   res_2.setZero();
//   VectorXd res;
//   res.setZero(n_sample * 2);
//   if (mpc_init)
//   {
//     QP_mpc.InitializeProblemSize(2 * n_sample, 1);
//     QP_mpc.UpdateMinProblem(H, g);
//     QP_mpc.PrintMinProb();
//     QP_mpc.PrintSubjectTox();
//     QP_mpc.SolveQPoases(1000);
//     set_defaults();
//     setup_indexing();
//     settings.verbose = 0;

//     for (int i = 0; i < 2 * n_sample; i++)
//       for (int j = 0; j < 2 * n_sample; j++)
//         params.H[i + j * 2 * n_sample] = H(i, j);

//     for (int i = 0; i < 2 * n_sample; i++)
//     {
//       params.g[i] = g(i);
//       params.r[i] = r(i);
//     }

//     params.f[0] = 0.025;
//     int num_inters = solve();

//     std::cout << "ANSWER IS " << vars.P[0] << vars.P[1] << std::endl;

//     res_2(0) = vars.P[0];
//     res_2(1) = vars.P[1];

//     p_k_1 = res_2;
//   }

//   if (mpc_init == false)
//   {
//     p_k_1 = zmp;
//   }
//   mpc_init = true;

//   return res_2;
// }

VectorQd WholebodyController::task_control_torque_QP(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
}

VectorQd WholebodyController::task_control_torque_QP2(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    VectorQd task_torque;
    VectorXd f_star_qp_;

    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);
    double friction_ratio = 0.3;
    //qptest
    double foot_x_length = 0.12;
    double foot_y_length = 0.04;

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * fstar_com(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * fstar_com(1) / 9.81;

    double dist_l, dist_r;
    dist_l = sqrt((Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y));
    dist_r = sqrt((Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y));

    double ratio_r, ratio_l;
    ratio_r = dist_l / (dist_l + dist_r);
    ratio_l = dist_r / (dist_l + dist_r);

    static int task_dof, contact_dof;
    int constraint_per_contact = 12;
    bool qpt_info = false;

    if ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof + task_dof;
    int constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::
    W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = W;                              // + 0.01 * MatrixXd::Identity(MODEL_DOF,MODEL_DOF);
    g.segment(0, MODEL_DOF) = -Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;

    //fstar regulation ::
    H.block(MODEL_DOF + contact_dof, MODEL_DOF + contact_dof, task_dof, task_dof) = 100 * MatrixXd::Identity(task_dof, task_dof);
    g.segment(MODEL_DOF + contact_dof, task_dof) = -100 * f_star_;

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Fsl(6 * i + 0, 6 * i + 0) = 0.0001;
        Fsl(6 * i + 1, 6 * i + 1) = 0.0001;
        //Fsl(6 * i + 2, 6 * i + 2) = 1E-6;
        Fsl(6 * i + 3, 6 * i + 3) = 0.001;
        Fsl(6 * i + 4, 6 * i + 4) = 0.001;
        Fsl(6 * i + 5, 6 * i + 5) = 0.00001;
    }

    double rr = DyrosMath::minmax_cut(ratio_r / ratio_l * 10, 1, 10);
    double rl = DyrosMath::minmax_cut(ratio_l / ratio_r * 10, 1, 10);
    //std::cout << "left : " << rr << "\t right : " << rl << std::endl;

    if (Robot.qp2nd)
    {
        Fsl(0, 0) = 0.0001 * rr;
        Fsl(1, 1) = 0.0001 * rr;

        Fsl(3, 3) = 0.001 * rr;
        Fsl(4, 4) = 0.001 * rr;

        Fsl(6, 6) = 0.0001 * rl;
        Fsl(7, 7) = 0.0001 * rl;

        Fsl(9, 9) = 0.001 * rl;
        Fsl(10, 10) = 0.001 * rl;
    }

    /*

    Vector3d P_right = Robot.link_[Right_Foot].xpos - Robot.com_.pos;
    Vector3d P_left = Robot.link_[Left_Foot].xpos - Robot.com_.pos;

    double pr = sqrt(P_right(0) * P_right(0) + P_right(1) * P_right(1));
    double pl = sqrt(P_left(0) * P_left(0) + P_left(1) * P_left(1));

    double prr = pr / (pl + pr);
    double pll = pl / (pl + pr);

    prr = DyrosMath::minmax_cut(prr, 0.1, 0.9);
    pll = DyrosMath::minmax_cut(pll, 0.1, 0.9);

    bool hr_v = false;
    if (hr_v)
    {
        Fsl(3, 3) = 0.001 * pll;
        Fsl(4, 4) = 0.001 * pll;

        Fsl(9, 9) = 0.001 * prr;
        Fsl(10, 10) = 0.001 * prr;
    }
    else
    {
        Fsl(3, 3) = 0.001;
        Fsl(4, 4) = 0.001;

        Fsl(9, 9) = 0.001;
        Fsl(10, 10) = 0.001;
    } */

    H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task_inv_T * Robot.Slc_k_T;
    A.block(0, MODEL_DOF + contact_dof, task_dof, task_dof) = -Robot.lambda;
    lbA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;
    ubA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //std::cout << "calc done!" << std::endl;
    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 3 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 4 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 1000.0;
    }

    //std::cout << "calc done!" << std::endl;
    //Torque bound setting
    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -1000;
        ub(MODEL_DOF + i) = 1000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = -20.0;
        ub(MODEL_DOF + 6 * i + 5) = 0.05;
        lb(MODEL_DOF + 6 * i + 5) = -0.05;
    }

    for (int i = 0; i < task_dof; i++)
    {
        lb(MODEL_DOF + contact_dof + i) = -1000;
        ub(MODEL_DOF + contact_dof + i) = 1000;
    }

    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
    QP_torque.EnableEqualityCondition(0.0001);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres = QP_torque.SolveQPoases(100);

    std::chrono::duration<double> toc = std::chrono::high_resolution_clock::now() - tic;
    task_torque = qpres.segment(0, MODEL_DOF);
    VectorXd fc = qpres.segment(MODEL_DOF, contact_dof);

    static double t100mean;
    static int t100count;
    t100mean += toc.count();
    t100count++;

    if (t100count > 500)
    {
        //std::cout << "QP calc time : " << t100mean / 500 * 1000 << " ms " << std::endl;

        t100mean = 0.0;
        t100count = 0;
    }
    //std::cout << "calc done!! first solution! " << toc.count() * 1000 << " ms" << std::endl;

    qpt_info = false;
    if (qpt_info)
    {
        //std::cout << "calc done!! first solution! " << toc - tic << std::endl;
        //std::cout << " torque result : " << std::endl;
        //std::cout << task_torque << std::endl;
        std::cout << "fc result " << std::endl;
        std::cout << qpres.segment(MODEL_DOF, contact_dof) << std::endl;
        //std::cout << "fstar result : " << std::endl;
        //std::cout << qpres.segment(MODEL_DOF + contact_dof, task_dof) << std::endl;
        //std::cout << "fstar desired : " << std::endl;
        //std::cout << f_star_ << std::endl;
    }

    if (false)
    {
        Fsl(0, 0) = Fsl(0, 0) * fc(8) / (fc(2) + fc(8));
        Fsl(1, 1) = Fsl(1, 1) * fc(8) / (fc(2) + fc(8));

        Fsl(3, 3) = Fsl(3, 3) * fc(8) / (fc(2) + fc(8));
        Fsl(4, 4) = Fsl(4, 4) * fc(8) / (fc(2) + fc(8));

        Fsl(6, 6) = Fsl(6, 6) * fc(2) / (fc(2) + fc(8));
        Fsl(7, 7) = Fsl(7, 7) * fc(2) / (fc(2) + fc(8));

        Fsl(9, 9) = Fsl(9, 9) * fc(2) / (fc(2) + fc(8));
        Fsl(10, 10) = Fsl(10, 10) * fc(2) / (fc(2) + fc(8));

        H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;

        QP_torque.UpdateMinProblem(H, g);
        //tic = getCPUtime();
        qpres = QP_torque.SolveQPoases(100);
        //toc = getCPUtime();

        task_torque = qpres.segment(0, MODEL_DOF);
        fc = qpres.segment(MODEL_DOF, contact_dof);
    }

    qpt_info = false;
    if (qpt_info)
    {
        //std::cout << "calc done! second solution! " << toc - tic << std::endl;
        //std::cout << " torque result : " << std::endl;
        //std::cout << task_torque << std::endl;
        std::cout << "fc result " << std::endl;
        std::cout << qpres.segment(MODEL_DOF, contact_dof) << std::endl;
        //std::cout << "fstar result : " << std::endl;
        //std::cout << qpres.segment(MODEL_DOF + contact_dof, task_dof) << std::endl;
        //std::cout << "fstar desired : " << std::endl;
        //std::cout << f_star_ << std::endl;
        std::cout << "##########################" << std::endl;
    }

    MatrixXd W_fr;

    if (Robot.ee_[0].contact && Robot.ee_[1].contact)
    {
        W_fr.setZero(6, 12);
        W_fr.block(0, 0, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 3, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 0, 3, 3) = Robot.link_[Left_Foot].Rotm * DyrosMath::skm(Robot.link_[Left_Foot].xpos_contact - Robot.com_.pos);

        W_fr.block(0, 6, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 9, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 6, 3, 3) = Robot.link_[Right_Foot].Rotm * DyrosMath::skm(Robot.link_[Right_Foot].xpos_contact - Robot.com_.pos);
    }
    else if(Robot.ee_[0].contact)
    {
        W_fr.setZero(6, 6);
        W_fr.block(0, 0, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 3, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 0, 3, 3) = Robot.link_[Left_Foot].Rotm * DyrosMath::skm(Robot.link_[Left_Foot].xpos_contact - Robot.com_.pos);


    }
    else if(Robot.ee_[1].contact)
    {
        W_fr.setZero(6, 6);
        W_fr.block(0, 0, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 3, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 0, 3, 3) = Robot.link_[Right_Foot].Rotm * DyrosMath::skm(Robot.link_[Right_Foot].xpos_contact - Robot.com_.pos);
    }

    VectorXd fr = W_fr * fc;

    Vector3d r_zmp = GetZMPpos(Robot, fc);
    qpt_info = false;
    if (qpt_info)
    {
        std::cout << "##########################" << std::endl;
        std::cout << "contact info ! " << std::endl
                  //<< "fc result : " << std::endl
                  //<< fc << std::endl
                  //<< "fc local : " << std::endl
                  //<< Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(0, 3) << std::endl
                  //<< Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(3, 3) << std::endl
                  //<< Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(6, 3) << std::endl
                  //<< Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(9, 3) << std::endl
                  //<< "qp fz : " << fc(2) + fc(8) << std::endl
                  //<< " fstar*lambda = " << (Robot.lambda * qpres.segment(MODEL_DOF + contact_dof, task_dof))(2) << std::endl
                  //<< " qp + lambda f* : " << (Robot.lambda * qpres.segment(MODEL_DOF + contact_dof, task_dof))(2) + fc(2) + fc(8) << std::endl
                  //<< " fstar desired : " << std::endl
                  //<< f_star_ << std::endl
                  //<< "fstar qp : " << std::endl
                  //<< qpres.segment(MODEL_DOF + contact_dof, task_dof) << std::endl
                  << "ft fz : " << Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8) << std::endl
                  << "resultant fz : " << std::endl
                  << fr << std::endl
                  << "resultant zmp x : " << r_zmp(0) << "\t y : " << r_zmp(1) << std::endl
                  << "estimate zmp x : " << zmp_com_e_x << "\t y : " << zmp_com_e_y << std::endl
                  << "com pos x : " << Robot.com_.pos(0) << "\t y : " << Robot.com_.pos(1) << std::endl
                  << "zmp result : " << std::endl
                  << "Left foot  x :" << fc(4) / fc(2) << "\t y :" << fc(3) / fc(2) << std::endl
                  << "Right foot x :" << fc(10) / fc(8) << "\t y :" << fc(9) / fc(8) << std::endl

                  << "fstar com induced : " << std::endl
                  << fstar_com << std::endl
                  << "fstar from torque : " << std::endl
                  << J_com * Robot.A_matrix_inverse * Robot.N_C * (Robot.Slc_k_T * task_torque - Robot.G) << std::endl;
    }

    fc.segment(0, 3) = Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(0, 3);
    fc.segment(3, 3) = Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(3, 3);
    fc.segment(6, 3) = Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(6, 3);
    fc.segment(9, 3) = Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(9, 3);

    double ft_zmp_l = Robot.ContactForce_FT(3) / Robot.ContactForce_FT(2);
    double ft_zmp_r = Robot.ContactForce_FT(9) / Robot.ContactForce_FT(8);

    if (abs(ft_zmp_l) > 0.04)
    {
        //std::cout << "lf zmp over limit : " << ft_zmp_l << std::endl;
    }
    if (abs(ft_zmp_r) > 0.04)
    {
        //std::cout << "rf zmp over limit : " << ft_zmp_r << std::endl;
    }

    Robot.ContactForce = fc;

    qpt_info = false;
    if (qpt_info)
    {
        std::cout << "##########################" << std::endl
                  << "fc result : " << std::endl
                  << fc << std::endl
                  << "zmp result : " << std::endl
                  << "Left foot  x :" << fc(4) / fc(2) << "\t y :" << fc(3) / fc(2) << std::endl
                  << "Right foot x :" << fc(10) / fc(8) << "\t y :" << fc(9) / fc(8) << std::endl
                  << "L/(L+R) : " << fc(2) / (fc(2) + fc(8)) << std::endl
                  //<< "pl/(pl+pr) : " << pr / (pl + pr) << std::endl

                  << "zmp from ft : " << std::endl
                  << "Left foot  x :" << Robot.ContactForce_FT(4) / Robot.ContactForce_FT(2) << "\t y :" << Robot.ContactForce_FT(3) / Robot.ContactForce_FT(2) << std::endl
                  << "Right foot x :" << Robot.ContactForce_FT(10) / Robot.ContactForce_FT(8) << "\t y :" << Robot.ContactForce_FT(9) / Robot.ContactForce_FT(8) << std::endl

                  << "com acc induce : " << std::endl;
        MatrixXd Jcom = Robot.link_[COM_id].Jac_COM_p;
        MatrixXd QQ = Jcom * Robot.A_matrix_inverse * Jcom.transpose() * (Jcom * Robot.A_matrix_inverse * Robot.N_C * Jcom.transpose()).inverse() * Jcom * Robot.A_matrix_inverse * Robot.N_C;

        std::cout << QQ * Robot.Slc_k_T * task_torque << std::endl;
        std::cout << "com acc - grav induce " << std::endl
                  << QQ * (Robot.Slc_k_T * task_torque - Robot.G) << std::endl;
    }
    return task_torque; // + gravity_torque;
}

VectorQd WholebodyController::task_control_torque_QP_gravity(RobotData &Robot)
{
    VectorQd task_torque;
    VectorXd f_star_qp_;
    Eigen::MatrixXd J_task;
    J_task.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);
    J_task.block(0, 6, MODEL_DOF, MODEL_DOF) = MatrixXd::Identity(MODEL_DOF, MODEL_DOF);
    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);
    double friction_ratio = 0.3;
    //qptest
    double foot_x_length = 0.12;
    double foot_y_length = 0.04;

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    static int task_dof, contact_dof;
    int constraint_per_contact = 8;
    bool qpt_info = false;

    if ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof;
    int constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::
    W = Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = W + MatrixXd::Identity(MODEL_DOF, MODEL_DOF);
    g.segment(0, MODEL_DOF) = -Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        //Fsl(6 * i + 0, 6 * i + 0) = 0.0001;
        //Fsl(6 * i + 1, 6 * i + 1) = 0.0001;
        Fsl(6 * i + 3, 6 * i + 3) = 0.001;
        Fsl(6 * i + 4, 6 * i + 4) = 0.001;
        Fsl(6 * i + 5, 6 * i + 5) = 0.001;
    }
    //H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;
    lbA.segment(0, task_dof) = Robot.J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    ubA.segment(0, task_dof) = Robot.J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 1000.0;
    }

    //std::cout << "calc done!" << std::endl;
    //Torque bound setting
    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }

    //ContactForce bound setting
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -1000;
        ub(MODEL_DOF + i) = 1000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = 0.0;
        //ub(MODEL_DOF + 6 * i + 5) = 0.05;
        //lb(MODEL_DOF + 6 * i + 5) = -0.05;
    }

    QP_torque.EnableEqualityCondition(1E-6);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres = QP_torque.SolveQPoases(100);
    task_torque = qpres.segment(0, MODEL_DOF);

    return task_torque; // + gravity_torque;
}

VectorXd WholebodyController::check_fstar(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * fstar_com(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * fstar_com(1) / 9.81;

    bool b1 = (zmp_com_e_x > Robot.link_[Left_Foot].xpos_contact(0) + Robot.ee_[0].cs_x_length) && (zmp_com_e_x > Robot.link_[Right_Foot].xpos_contact(0) + Robot.ee_[0].cs_x_length);

    bool b2 = (zmp_com_e_x < Robot.link_[Left_Foot].xpos_contact(0) - Robot.ee_[0].cs_x_length) && (zmp_com_e_x < Robot.link_[Right_Foot].xpos_contact(0) - Robot.ee_[0].cs_x_length);

    bool b3 = (zmp_com_e_y > Robot.link_[Left_Foot].xpos_contact(1) + Robot.ee_[0].cs_y_length) && (zmp_com_e_y > Robot.link_[Right_Foot].xpos_contact(1) + Robot.ee_[0].cs_y_length);

    bool b4 = (zmp_com_e_y < Robot.link_[Left_Foot].xpos_contact(1) - Robot.ee_[0].cs_y_length) && (zmp_com_e_y < Robot.link_[Right_Foot].xpos_contact(1) - Robot.ee_[0].cs_y_length);

    if (b1 || b2 || b3 || b4)
    {
        std::cout << "Control command out of support polygon! " << std::endl;
    }
}

VectorQd WholebodyController::contact_torque_calc_from_QP(RobotData &Robot, VectorQd command_torque)
{
    if (Robot.ee_[0].contact && Robot.ee_[1].contact)
    {
        VectorXd ContactForce__ = get_contact_force(Robot, command_torque);

        double a1 = 0.0;
        double a2 = 1.0;
        double friction_ratio = 0.3;
        //qptest
        double foot_x_length = 0.12;
        double foot_y_length = 0.04;

        int constraint_per_contact = 8;
        QP_test.InitializeProblemSize(6 * Robot.contact_index, 6 + constraint_per_contact * Robot.contact_index);

        MatrixXd H, A, M;
        H.setZero(6 * Robot.contact_index, 6 * Robot.contact_index);
        M.setZero(6 * Robot.contact_index, 6 * Robot.contact_index);
        for (int i = 0; i < Robot.contact_index; i++)
        {
            M(6 * i, 6 * i) = 10;
            M(6 * i + 1, 6 * i + 1) = 10;
            M(6 * i + 2, 6 * i + 2) = 0.0;
            M(6 * i + 3, 6 * i + 3) = 1000;
            M(6 * i + 4, 6 * i + 4) = 1000;
            M(6 * i + 5, 6 * i + 5) = 1000;
        }
        H = a1 * MatrixXd::Identity(Robot.contact_index * 6, Robot.contact_index * 6) + a2 * M;

        A.setZero(6 + constraint_per_contact * Robot.contact_index, 6 * Robot.contact_index);
        for (int i = 0; i < Robot.contact_index; i++)
        {
            A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
            A.block(3, 6 * i, 3, 3) = DyrosMath::skm(Robot.link_[Robot.contact_part[i]].xpos_contact - Robot.com_.pos);
        }

        for (int i = 0; i < Robot.contact_index; i++)
        {
            A(6 + i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
            A(6 + i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
            A(6 + i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

            A(6 + i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
            A(6 + i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
            A(6 + i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

            A(6 + i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
            A(6 + i * constraint_per_contact + 4, 2 + 6 * i) = -friction_ratio;
            A(6 + i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 5, 2 + 6 * i) = -friction_ratio;

            A(6 + i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
            A(6 + i * constraint_per_contact + 6, 2 + 6 * i) = -friction_ratio;
            A(6 + i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 7, 2 + 6 * i) = -friction_ratio;
        }

        VectorXd force_res = A.block(0, 0, 6, Robot.contact_index * 6) * ContactForce__;
        VectorXd g, lb, ub, lbA, ubA;
        g.setZero(Robot.contact_index * 6);
        g = -a1 * ContactForce__;

        lbA.setZero(6 + constraint_per_contact * Robot.contact_index);
        ubA.setZero(6 + constraint_per_contact * Robot.contact_index);
        lbA.segment(0, 6) = force_res;
        ubA.segment(0, 6) = force_res;
        ub.setZero(6 * Robot.contact_index);
        lb.setZero(6 * Robot.contact_index);
        for (int i = 0; i < 6 * Robot.contact_index; i++)
        {
            lb(i) = -1000;
            ub(i) = 1000;
        }
        for (int i = 0; i < Robot.contact_index; i++)
        {
            ub(6 * i + 2) = 0.0;
            ub(6 * i + 5) = 0.0;
            lb(6 * i + 5) = 0.0;
        }

        for (int i = 0; i < Robot.contact_index; i++)
        {
            ub(i * 6 + 2) = -15.0;
        }

        for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
        {
            lbA(6 + i) = 0.0;
            ubA(6 + i) = 1000.0;
        }

        QP_test.EnableEqualityCondition(0.0001);
        QP_test.UpdateMinProblem(H, g);
        QP_test.UpdateSubjectToAx(A, lbA, ubA);
        QP_test.UpdateSubjectToX(lb, ub);

        //ROS_INFO("l8");
        VectorXd force_redistribute = QP_test.SolveQPoases(100);

        //ROS_INFO("l9");
        result_temp = force_redistribute;

        //ROS_INFO("l10");
        VectorXd torque_contact_ = contact_force_custom(Robot, command_torque, ContactForce__, force_redistribute);

        //std::cout << "###########################" << std::endl;
        //std::cout << "redistribute" << std::endl;
        //std::cout << force_redistribute << std::endl;
        //std::cout << "position of lHand" <<std::endl;
        //std::cout << Robot.link_[Left_Hand].xpos_contact <<std::endl;
        // std::cout << "resultant force" << std::endl;
        // std::cout << force_res << std::endl;
        // std::cout << "A matrix " << std::endl;
        // std::cout << A << std::endl;
        // std::cout << "lbA" << std::endl;
        // std::cout << lbA << std::endl;
        // std::cout << "ubA" << std::endl;
        // std::cout << ubA << std::endl;
        //std::cout << "ub" << std::endl;
        //std::cout << ub << std::endl;
        // std::cout << "lb" << std::endl;
        // std::cout << lb << std::endl;

        //ROS_INFO("l2");
        return torque_contact_;
    }
    return VectorXd::Zero(MODEL_DOF);
}
/*
VectorQd WholebodyController::contact_torque_calc_from_QP_wall(VectorQd command_torque, double wall_friction_ratio)
{
    VectorXd ContactForce__ = get_contact_force(command_torque);
    QP_test.InitializeProblemSize(contact_index * 6, 6 + contact_index);
    MatrixXd H, A;
    H.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        H(6 * i, 6 * i) = 1;
        H(6 * i + 1, 6 * i + 1) = 1;
        H(6 * i + 2, 6 * i + 2) = 0.01;
        H(6 * i + 3, 6 * i + 3) = 100;
        H(6 * i + 4, 6 * i + 4) = 100;
        H(6 * i + 5, 6 * i + 5) = 100;
    }
    A.setZero(6 + contact_index, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(link_[contact_part[i]].xpos_contact - com_.pos);
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);
    lbA.setZero(6 + contact_index);
    ubA.setZero(6 + contact_index);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(contact_index * 6);
    lb.setZero(contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i, 1 + 6 * i) = 1.0;

        ubA(6 + i) = 0.0;
        lbA(6 + i) = 0.0;
        if (contact_part[i] == Right_Foot)
        {
            A(6 + i, 2 + 6 * i) = -wall_friction_ratio;
        }
        if (contact_part[i] == Right_Hand)
        {
            A(6 + i, 2 + 6 * i) = -wall_friction_ratio;
        }
        /*if (contact_part[i] == Left_Foot)
        {
            A(6 + i, 2 + 6 * i) = wall_friction_ratio;
        }
        if (contact_part[i] == Left_Hand)
        {
            A(6 + i, 2 + 6 * i) = wall_friction_ratio;
        } 
}
for (int i = 0; i < contact_index * 6; i++)
{
    lb(i) = -1000;
    ub(i) = 1000;
}

for (int i = 0; i < contact_index; i++)
    ub(2 + 6 * i) = 0;

QP_test.EnableEqualityCondition(0.001);
QP_test.UpdateMinProblem(H, g);
QP_test.UpdateSubjectToAx(A, lbA, ubA);
QP_test.UpdateSubjectToX(lb, ub);
VectorXd force_redistribute = QP_test.SolveQPoases(100);

std::cout << "Contact Force now :  " << std::endl;
std::cout << ContactForce__ << std::endl;
std::cout << "Contact Force Redistribution : " << std::endl;
std::cout << force_redistribute << std::endl;

VectorQd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);
return torque_contact_;
}

VectorQd WholebodyController::contact_torque_calc_from_QP_wall_mod2(VectorQd command_torque, double wall_friction_ratio)
{
    double a1, a2;

    a1 = 10.0; //Contactforce control ratio
    a2 = 1.0;  //moment minimize ratio

    VectorXd ContactForce__ = get_contact_force(command_torque);
    QP_test.InitializeProblemSize(contact_index * 6, 6 + contact_index * 5);
    MatrixXd H, A;
    H.setZero(contact_index * 6, contact_index * 6);

    MatrixXd M;
    M.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        M(6 * i, 6 * i) = 1.0;
        M(6 * i + 1, 6 * i + 1) = 1.0;
        M(6 * i + 2, 6 * i + 2) = 0.1;
        M(6 * i + 3, 6 * i + 3) = 100.0;
        M(6 * i + 4, 6 * i + 4) = 100.0;
        M(6 * i + 5, 6 * i + 5) = 100.0;
    }

    H = a1 * MatrixXd::Identity(contact_index * 6, contact_index * 6) + a2 * M;

    A.setZero(6 + contact_index * 5, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(link_[contact_part[i]].xpos_contact - com_.pos);
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);

    g = -a1 * ContactForce__;
    lbA.setZero(6 + contact_index * 5);
    ubA.setZero(6 + contact_index * 5);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(contact_index * 6);
    lb.setZero(contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i * 5, 1 + 6 * i) = 1.0;
        A(6 + i * 5 + 1, 3 + 6 * i) = 1.0;
        A(6 + i * 5 + 2, 3 + 6 * i) = 1.0;
        A(6 + i * 5 + 3, 4 + 6 * i) = 1.0;
        A(6 + i * 5 + 4, 4 + 6 * i) = 1.0;

        if (contact_part[i] == Right_Foot)
        {
            A(6 + i * 5, 2 + 6 * i) = -wall_friction_ratio;
            ubA(6 + i * 5) = 0.0;
            lbA(6 + i * 5) = -1000.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.03;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.03;
            lbA(6 + i * 5 + 1) = 0.0;
            ubA(6 + i * 5 + 1) = 1000.0;
            lbA(6 + i * 5 + 2) = -1000.0;
            ubA(6 + i * 5 + 2) = 0;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.05;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.05;
            lbA(6 + i * 5 + 3) = 0.0;
            ubA(6 + i * 5 + 3) = 1000.0;
            lbA(6 + i * 5 + 4) = -1000.0;
            ubA(6 + i * 5 + 4) = 0;
        }
        if (contact_part[i] == Right_Hand)
        {
            A(6 + i * 5, 2 + 6 * i) = -wall_friction_ratio;

            ubA(6 + i * 5) = 0.0;
            lbA(6 + i * 5) = -1000.0;

            A(6 + i * 5 + 1, 6 * i + 1) = -0.02;
            A(6 + i * 5 + 2, 6 * i + 1) = 0.02;
            lbA(6 + i * 5 + 1) = 0.0;
            ubA(6 + i * 5 + 1) = 1000.0;
            lbA(6 + i * 5 + 2) = -1000.0;
            ubA(6 + i * 5 + 2) = 0;

            A(6 + i * 5 + 3, 6 * i + 1) = -0.02;
            A(6 + i * 5 + 4, 6 * i + 1) = 0.02;
            lbA(6 + i * 5 + 3) = 0.0;
            ubA(6 + i * 5 + 3) = 1000.0;
            lbA(6 + i * 5 + 4) = -1000.0;
            ubA(6 + i * 5 + 4) = 0;
        }
        if (contact_part[i] == Left_Foot)
        {
            A(6 + i * 5, 2 + 6 * i) = wall_friction_ratio;

            ubA(6 + i * 5) = 1000.0;
            lbA(6 + i * 5) = 0.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.03;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.03;
            lbA(6 + i * 5 + 1) = -1000.0;
            ubA(6 + i * 5 + 1) = 0.0;
            lbA(6 + i * 5 + 2) = 0.0;
            ubA(6 + i * 5 + 2) = 1000.0;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.05;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.05;
            lbA(6 + i * 5 + 3) = -1000.0;
            ubA(6 + i * 5 + 3) = 0.0;
            lbA(6 + i * 5 + 4) = 0.0;
            ubA(6 + i * 5 + 4) = 1000.0;
        }
        if (contact_part[i] == Left_Hand)
        {
            A(6 + i * 5, 2 + 6 * i) = wall_friction_ratio;

            ubA(6 + i * 5) = 1000.0;
            lbA(6 + i * 5) = 0.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.02;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.02;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.02;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.02;

            lbA(6 + i * 5 + 1) = -1000.0;
            ubA(6 + i * 5 + 1) = 0.0;
            lbA(6 + i * 5 + 2) = 0.0;
            ubA(6 + i * 5 + 2) = 1000.0;

            lbA(6 + i * 5 + 3) = -1000.0;
            ubA(6 + i * 5 + 3) = 0.0;
            lbA(6 + i * 5 + 4) = 0.0;
            ubA(6 + i * 5 + 4) = 1000.0;
        }
    }
    for (int i = 0; i < contact_index * 6; i++)
    {
        lb(i) = -1000;
        ub(i) = 1000;
    }

    for (int i = 0; i < contact_index; i++)
        ub(2 + 6 * i) = 0;

    QP_test.EnableEqualityCondition(0.01);
    QP_test.UpdateMinProblem(H, g);
    QP_test.UpdateSubjectToAx(A, lbA, ubA);
    QP_test.UpdateSubjectToX(lb, ub);
    VectorXd force_redistribute = QP_test.SolveQPoases(200);

    std::cout << "Contact Force now :  " << std::endl;
    std::cout << ContactForce__ << std::endl;
    std::cout << "Contact Force Redistribution : " << std::endl;
    std::cout << force_redistribute << std::endl;

    VectorQd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);
    result_temp = force_redistribute;
    return torque_contact_;
}
*/

VectorQd WholebodyController::CP_control_init(RobotData &Robot, double dT)
{
    double w_ = sqrt(9.81 / Robot.com_.pos(2));
    double b_ = exp(w_ * dT);

    Vector2d CP_displace;
    CP_displace(0) = 0.0;
    CP_displace(1) = 0.015;

    CP_ref[0] = Robot.com_.pos.segment(0, 2);
    CP_ref[1] = Robot.link_[Left_Foot].xpos.segment(0, 2) - CP_displace;
    CP_ref[2] = Robot.link_[Right_Foot].xpos.segment(0, 2) + CP_displace;
    CP_ref[3] = Robot.com_.pos.segment(0, 2);
}

VectorQd WholebodyController::CP_controller()
{
}

Vector6d WholebodyController::zmp_controller(RobotData &Robot, Vector2d ZMP, double height)
{
    double w_ = sqrt(9.81 / Robot.com_.pos(2));
    Vector3d desired_accel;
    desired_accel.segment(0, 2) = pow(w_, 2) * (Robot.com_.pos.segment(0, 2) - ZMP);
    desired_accel(2) = 0.0;
    Vector3d desired_vel = Robot.com_.vel + desired_accel * abs(Robot.d_time_);
    desired_vel(2) = 0.0;
    Vector3d desired_pos = Robot.com_.pos + desired_vel * abs(Robot.d_time_);
    desired_pos(2) = height;
    Eigen::Vector3d kp_, kd_;
    kp_ << 400, 400, 400;
    kd_ << 40, 40, 40;
    Vector3d fstar = getfstar(Robot, kp_, kd_, desired_pos, Robot.com_.pos, desired_vel, Robot.com_.vel);

    Vector3d fstar_r;
    fstar_r(0) = -ZMP(1) / (Robot.com_.mass * 9.81);
    fstar_r(1) = -ZMP(0) / (Robot.com_.mass * 9.81);
    fstar_r(2) = 0;

    Vector6d r_z;
    r_z.segment(0, 3) = fstar;
    r_z.segment(3, 3) = fstar_r;

    return r_z;
}

VectorQd WholebodyController::gravity_compensation_torque(RobotData &Robot, bool fixed, bool redsvd)
{
    if (Robot.contact_calc == false)
    {
        set_contact(Robot);
    }

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }
    if (fixed)
        return Robot.G.segment(6, MODEL_DOF);

    Eigen::MatrixXd J_g;
    J_g.setZero(MODEL_DOF, MODEL_DOF + 6);
    J_g.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();

    Eigen::VectorXd torque_grav(MODEL_DOF);
    Eigen::MatrixXd aa = J_g * Robot.A_matrix_inverse * Robot.N_C * J_g.transpose();
    /*
    double epsilon = 1e-7;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(aa.cols(), aa.rows()) *svd.singularValues().array().abs()(0);
    Eigen::MatrixXd ppinv = svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

    Eigen::MatrixXd ppinv = aa.completeOrthogonalDecomposition().pseudoInverse();
    torque_grav = (J_g*A_matrix.inverse()*N_C*J_g.transpose()).completeOrthogonalDecomposition().pseudoInverse()*J_g*A_matrix.inverse()*N_C*G;
    torque_grav.setZero();
    Eigen::MatrixXd ppinv = DyrosMath::pinv_QR(aa);
    */
    Eigen::MatrixXd ppinv;
    double epsilon = 1e-7;
    if (redsvd)
    {
        RedSVD::RedSVD<Eigen::MatrixXd> svd(aa);
        double tolerance = epsilon * std::max(aa.cols(), aa.rows()) * svd.singularValues().array().abs()(0);
        ppinv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
    else
    {
        ppinv = DyrosMath::pinv_SVD(aa);
    }

    Eigen::MatrixXd tg_temp = ppinv * J_g * Robot.A_matrix_inverse * Robot.N_C;
    torque_grav = tg_temp * Robot.G;

    Robot.contact_calc = false;
    return torque_grav;
}

VectorQd WholebodyController::task_control_torque(RobotData &Robot, MatrixXd J_task, VectorXd f_star_)
{
    Robot.task_dof = J_task.rows();

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();

    Robot.J_task_T = J_task.transpose();

    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;

    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    Robot.Q = Robot.J_task_inv_T * Robot.Slc_k_T;
    Robot.Q_T_ = Robot.Q.transpose();

    Robot.Q_temp = Robot.Q * Robot.W_inv * Robot.Q_T_;

    Robot.Q_temp_inv = DyrosMath::pinv_SVD(Robot.Q_temp);

    //_F=lambda*(f_star);
    //Jtemp=J_task_inv_T*Slc_k_T;
    //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
    //Q.svd(s2,u2,v2);

    VectorQd torque_task;
    if (Robot.task_force_control)
    {
        VectorXd F_;
        F_.resize(Robot.task_dof);
        F_ = Robot.lambda * Robot.task_selection_matrix * f_star_;
        VectorXd F_2;
        F_2 = F_ + Robot.task_desired_force;
        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * F_2;
    }
    else if (Robot.task_force_control_feedback)
    {
        VectorXd F_;
        F_.resize(Robot.task_dof);

        static double right_i, left_i;

        double pd = 0.1;
        double pi = 4.0;

        double left_des = -50.0;
        double right_des = 50.0;

        double right_err = Robot.task_desired_force(10) + Robot.task_feedback_reference(1);
        double left_err = Robot.task_desired_force(16) + Robot.task_feedback_reference(7);

        right_i += right_err * Robot.d_time_;
        left_i += left_err * Robot.d_time_;

        VectorXd fc_fs;
        fc_fs = Robot.task_desired_force;
        fc_fs.setZero();

        fc_fs(10) = pd * right_err + pi * right_i;
        fc_fs(16) = pd * left_err + pi * left_i;
        F_ = Robot.lambda * (Robot.task_selection_matrix * f_star_ + fc_fs);

        VectorXd F_2;
        F_2 = F_ + Robot.task_desired_force;

        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * F_2;
    }
    else if (Robot.zmp_control)
    {
        int zmp_dof = 6;
        VectorXd F_;
        F_.resize(Robot.task_dof);
        Robot.task_selection_matrix.setIdentity(Robot.task_dof, Robot.task_dof);
        Robot.task_selection_matrix.block(0, 0, 2, 2).setZero();
        //Robot.task_selection_matrix.block(1, 1, 1, 1).setZero();

        F_ = Robot.lambda * Robot.task_selection_matrix * f_star_;

        //double kpf = 200.0;
        //double kvf = 1.0;

        Vector2d Fd_com;
        Fd_com.setZero();
        Fd_com = 9.81 / Robot.com_.pos(2) * (Robot.com_.pos.segment(0, 2) - Robot.ZMP_desired.segment(0, 2));

        //rk_.ZMP_error(1) = 0.0;

        //Robot.ZMP_command = Robot.ZMP_command + 0.5 * Robot.ZMP_error; //+ rk_.ZMP_mod;
        //Fd_com(1) = Robot.zmp_gain * 9.81 / (Robot.com_.pos(2) - Robot.link_[Right_Foot].xpos(2) * 0.5 - Robot.link_[Left_Foot].xpos(2) * 0.5) * (Robot.com_.pos(1) - Robot.ZMP_command(1));
        //rk_.ZMP_command = rk_.ZMP_command + 0.5 * rk_.ZMP_error + rk_.ZMP_mod;

        //Fd_com(1) = zmp_gain * 9.81 / rk_.com_.pos(2) * (rk_.com_.pos(1) - ZMP_task(1) - 1.0*rk_.ZMP_error(1)) * rk_.com_.mass;

        //Robot.task_desired_force.setZero(Robot.task_dof);

        //Robot.task_desired_force(1) = Fd_com(1); // - kpf*rk_.ZMP_error(1);//-kvf*rk_.com_.vel(1);
        //std::cout << Fd_com(1) << "\t" << rk_.ZMP_error(1) << std::endl;
        //task_desired_force(1) = Fd_com(1);// + kpf*rk_.ZMP_error(1)-kvf*rk_.com_.vel(1);
        //task_desired_force.segment(0, 2) = Fd_com;
        //task_desired_force(3) = ZMP_task(1) * (com_.mass * 9.81);
        //task_desired_force(4) = ZMP_task(0) * (com_.mass * 9.81);
        //F_2 = F_ + task_desired_force;

        f_star_.segment(0, 2) = Fd_com;

        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_;
    }
    else if (Robot.zmp_feedback_control)
    {
        int zmp_dof = 6;
        VectorXd F_;
        F_.resize(Robot.task_dof);
        Robot.task_selection_matrix.setIdentity(Robot.task_dof, Robot.task_dof);
        Robot.task_selection_matrix.block(0, 0, 2, 2).setZero();
        //Robot.task_selection_matrix.block(1, 1, 1, 1).setZero();

        F_ = Robot.lambda * Robot.task_selection_matrix * f_star_;

        //double kpf = 200.0;
        //double kvf = 1.0;

        Vector2d Fd_com;
        Fd_com.setZero();
        Fd_com = 9.81 / Robot.com_.pos(2) * (Robot.com_.pos.segment(0, 2) - Robot.ZMP_command.segment(0, 2));

        //rk_.ZMP_error(1) = 0.0;

        //Robot.ZMP_command = Robot.ZMP_command + 0.5 * Robot.ZMP_error; //+ rk_.ZMP_mod;
        //Fd_com(1) = Robot.zmp_gain * 9.81 / (Robot.com_.pos(2) - Robot.link_[Right_Foot].xpos(2) * 0.5 - Robot.link_[Left_Foot].xpos(2) * 0.5) * (Robot.com_.pos(1) - Robot.ZMP_command(1));
        //rk_.ZMP_command = rk_.ZMP_command + 0.5 * rk_.ZMP_error + rk_.ZMP_mod;

        //Fd_com(1) = zmp_gain * 9.81 / rk_.com_.pos(2) * (rk_.com_.pos(1) - ZMP_task(1) - 1.0*rk_.ZMP_error(1)) * rk_.com_.mass;

        //Robot.task_desired_force.setZero(Robot.task_dof);

        //Robot.task_desired_force(1) = Fd_com(1); // - kpf*rk_.ZMP_error(1);//-kvf*rk_.com_.vel(1);
        //std::cout << Fd_com(1) << "\t" << rk_.ZMP_error(1) << std::endl;
        //task_desired_force(1) = Fd_com(1);// + kpf*rk_.ZMP_error(1)-kvf*rk_.com_.vel(1);
        //task_desired_force.segment(0, 2) = Fd_com;
        //task_desired_force(3) = ZMP_task(1) * (com_.mass * 9.81);
        //task_desired_force(4) = ZMP_task(0) * (com_.mass * 9.81);
        //F_2 = F_ + task_desired_force;

        f_star_.segment(0, 2) = Fd_com;

        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_;
    }
    else
    {
        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_;
    }

    //W.svd(s,u,v);
    //V2.resize(28,6);
    //V2.zero();

    return torque_task;
}
/*
VectorQd WholebodyController::task_control_torque_custom_force(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(MODEL_DOF + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  F_ = lambda * selection_matrix * f_star_;

  VectorXd F_2;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}

VectorQd WholebodyController::task_control_torque_custom_force_feedback(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(MODEL_DOF + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  static double right_i, left_i;

  double pd = 0.1;
  double pi = 4.0;

  double left_des = -50.0;
  double right_des = 50.0;

  double right_err = desired_force(10) + ft_hand(1);
  double left_err = desired_force(16) + ft_hand(7);

  right_i += right_err * d_time_;
  left_i += left_err * d_time_;

  VectorXd fc_fs; // = desired_force;
  fc_fs = desired_force;
  fc_fs.setZero();

  fc_fs(10) = pd * right_err + pi * right_i;
  fc_fs(16) = pd * left_err + pi * left_i;

  //std::cout << "right : " << fc_fs(10) << std::endl;
  //std::cout << "left : " << fc_fs(16) << std::endl;

  F_ = lambda * (selection_matrix * f_star_ + fc_fs);

  //F_ = selection_matrix * lambda * f_star_;

  VectorXd F_2;

  //desired_force(10) = desired_force(10) + right_des;
  //desired_force(16) = desired_force(16) + left_des;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}
*/
void WholebodyController::set_force_control(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force)
{
    Robot.task_force_control = true;
    Robot.task_selection_matrix = selection_matrix;
    Robot.task_desired_force = desired_force;
}
void WholebodyController::set_force_control_feedback(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{
    Robot.task_force_control_feedback = true;
    Robot.task_selection_matrix = selection_matrix;
    Robot.task_desired_force = desired_force;
    Robot.task_feedback_reference = ft_hand;
}

void WholebodyController::set_zmp_control(RobotData &Robot, Vector2d ZMP, double gain)
{
    Robot.zmp_control = true;
    Robot.ZMP_desired(0) = ZMP(0);
    Robot.ZMP_desired(1) = ZMP(1);
    Robot.zmp_gain = gain;
}

void WholebodyController::set_zmp_feedback_control(RobotData &Robot, Vector2d ZMP, bool &reset_error)
{
    Robot.zmp_feedback_control = true;
    Robot.ZMP_ft = GetZMPpos_fromFT(Robot);
    Robot.ZMP_ft.segment(0, 2) = Robot.com_.ZMP;
    Robot.ZMP_error = Robot.ZMP_desired - Robot.ZMP_ft;

    Robot.ZMP_desired.segment(0, 2) = ZMP;
    if (reset_error)
    {
        std::cout << "error reset!" << std::endl;
        Robot.ZMP_command = Robot.ZMP_desired;
        Robot.ZMP_error.setZero();
        reset_error = false;
    }
    else
    {
        Robot.ZMP_command = Robot.ZMP_command + 0.01 * Robot.ZMP_error;
    }
}

Vector3d WholebodyController::getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now)
{

    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = kp(i) * (p_desired(i) - p_now(i)) + kd(i) * (d_desired(i) - d_now(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now)
{

    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(Robot.yaw);
    Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(r_now, r_desired);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (kp(i) * angle_d_global(i) - kd(i) * w_now(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar_tra(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt)
{
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = kpt(i) * (Robot.link_[link_id].x_traj(i) - Robot.link_[link_id].xpos(i)) + kdt(i) * (Robot.link_[link_id].v_traj(i) - Robot.link_[link_id].v(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar_tra(RobotData &Robot, int link_id)
{
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = Robot.link_[link_id].a_traj(i) + Robot.link_[link_id].pos_p_gain(i) * (Robot.link_[link_id].x_traj(i) - Robot.link_[link_id].xpos(i)) + Robot.link_[link_id].pos_d_gain(i) * (Robot.link_[link_id].v_traj(i) - Robot.link_[link_id].v(i));

        if (i == 1)
        {
            //std::cout<<"xtraj y : "<<rk_.link_[link_id].x_traj(i) <<"\t xpos y : "<<rk_.link_[link_id].xpos(i)<<"\t f_star : "<<fstar_(i)<<std::endl;
        }
        //fstar_(i) = rk_.link_[link_id].a_traj(i) + rk_.link_[link_id].pos_p_gain(i) * (rk_.link_[link_id].x_traj(i) - rk_.link_[link_id].xpos(i)) + rk_.link_[link_id].pos_d_gain(i) * (rk_.link_[link_id].v_traj(i) - rk_.link_[link_id].v(i));
    }
    Robot.fstar = fstar_;
    return fstar_;
}

Vector3d WholebodyController::getfstar_rot(RobotData &Robot, int link_id, Vector3d kpa, Vector3d kda)
{
    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(Robot.yaw);

    Vector3d angle_d_global = -Rotyaw * DyrosMath::getPhi(Robot.link_[link_id].Rotm, Robot.link_[link_id].r_traj);

    //Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw);

    //Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(link_[link_id].Rotm, link_[link_id].r_traj);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (kpa(i) * angle_d_global(i) - kda(i) * Robot.link_[link_id].w(i));
    }
    /*
  std::cout << "fstar check " << std::endl
            << link_[link_id].name << std::endl
            << " rotation now " << std::endl
            << link_[link_id].Rotm << std::endl
            << "desired rotation " << std::endl
            << link_[link_id].r_traj << std::endl
            << "angle d " << std::endl
            << angle_d << std::endl
            << "global angle d " << std::endl
            << angle_d_global << std::endl
            << "fstar " << std::endl
            << fstar_ << std::endl
            << " ////////////////////////////////////////////////////////////////" << std::endl;
  */

    return fstar_;
}

Vector3d WholebodyController::getfstar_rot(RobotData &Robot, int link_id)
{
    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(Robot.yaw);

    Vector3d angle_d_global = -Rotyaw * DyrosMath::getPhi(Robot.link_[link_id].Rotm, Robot.link_[link_id].r_traj);

    //Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw);

    //Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(link_[link_id].Rotm, link_[link_id].r_traj);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (Robot.link_[link_id].rot_p_gain(i) * angle_d_global(i) - Robot.link_[link_id].rot_d_gain(i) * Robot.link_[link_id].w(i));
    }
    /*
    std::cout << "fstar check " << std::endl
              << Robot.link_[link_id].name << std::endl
              << " rotation now " << std::endl
              << Robot.link_[link_id].Rotm << std::endl
              << "desired rotation " << std::endl
              << Robot.link_[link_id].r_traj << std::endl
              //<< "angle d " << std::endl
              //<< angle_d << std::endl
              << "global angle d " << std::endl
              << angle_d_global << std::endl
              << "fstar " << std::endl
              << fstar_ << std::endl
              << " ////////////////////////////////////////////////////////////////" << std::endl;*/

    return fstar_;
}

Vector6d WholebodyController::getfstar6d(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda)
{
    Vector6d f_star;
    f_star.segment(0, 3) = getfstar_tra(Robot, link_id, kpt, kdt);
    f_star.segment(3, 3) = getfstar_rot(Robot, link_id, kpa, kda);
    return f_star;
}

Vector6d WholebodyController::getfstar6d(RobotData &Robot, int link_id)
{
    Vector6d f_star;
    f_star.segment(0, 3) = getfstar_tra(Robot, link_id);
    f_star.segment(3, 3) = getfstar_rot(Robot, link_id);

    Robot.link_[link_id].fstar = f_star;
    return f_star;
}

VectorQd WholebodyController::contact_force_custom(RobotData &Robot, VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired)
{
    JacobiSVD<MatrixXd> svd(Robot.W, ComputeThinU | ComputeThinV);
    Robot.svd_U = svd.matrixU();

    MatrixXd V2;

    int singular_dof = 6;
    int contact_dof = Robot.J_C.rows();
    V2.setZero(MODEL_DOF, contact_dof - singular_dof);
    V2 = Robot.svd_U.block(0, MODEL_DOF - contact_dof + singular_dof, MODEL_DOF, contact_dof - singular_dof);

    MatrixXd Scf_;
    Scf_.setZero(contact_dof - singular_dof, contact_dof);
    Scf_.block(0, 0, contact_dof - singular_dof, contact_dof - singular_dof).setIdentity();

    // std::cout << contact_force_desired << std::endl
    //           << std::endl
    //           << std::endl;
    // std::cout << contact_force_now << std::endl
    //           << std::endl
    //           << std::endl;
    // std::cout << std::endl;

    VectorXd desired_force = contact_force_desired - contact_force_now;

    MatrixXd temp = Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * V2;
    MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);
    MatrixXd Vc_ = V2 * temp_inv;

    VectorXd reduced_desired_force = Scf_ * desired_force;
    VectorQd torque_contact_ = Vc_ * reduced_desired_force;

    return torque_contact_;
}

VectorXd WholebodyController::get_contact_force(RobotData &Robot, VectorQd command_torque)
{
    VectorXd contactforce = Robot.J_C_INV_T * Robot.Slc_k_T * command_torque - Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G;
    return contactforce;
}

VectorQd WholebodyController::contact_force_redistribution_torque(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    //Contact Jacobian task : rightfoot to leftfoot

    int contact_dof_ = Robot.J_C.rows();

    VectorQd torque_contact_;

    ForceRedistribution.setZero();

    if (contact_dof_ == 12)
    {

        Vector12d ContactForce_ = Robot.J_C_INV_T * Robot.Slc_k_T * command_torque - Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G;

        Vector3d P1_, P2_;

        P1_ = Robot.link_[Left_Foot].xpos_contact - Robot.link_[COM_id].xpos;
        P2_ = Robot.link_[Right_Foot].xpos_contact - Robot.link_[COM_id].xpos;

        Matrix3d Rotyaw = DyrosMath::rotateWithZ(-Robot.yaw);

        Vector3d P1_local, P2_local;
        P1_local = Rotyaw * P1_;
        P2_local = Rotyaw * P2_;

        MatrixXd force_rot_yaw;
        force_rot_yaw.setZero(12, 12);
        for (int i = 0; i < 4; i++)
        {
            force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
        }

        Vector6d ResultantForce_;
        ResultantForce_.setZero();

        Vector12d ResultRedistribution_;
        ResultRedistribution_.setZero();

        torque_contact_.setZero();

        double eta_cust = 0.99;
        double foot_length = 0.26;
        double foot_width = 0.1;

        Vector12d ContactForce_Local_yaw;
        ContactForce_Local_yaw = force_rot_yaw * ContactForce_; //Robot frame based contact force

        //ZMP_pos = GetZMPpos(P1_local, P2_local, ContactForce_Local_yaw);

        ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, P1_local, P2_local, ContactForce_Local_yaw, ResultantForce_, ResultRedistribution_, eta);

        //std::cout << "fres - calc" << std::endl
        //          << ResultantForce_ << std::endl;

        //J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
        ForceRedistribution = force_rot_yaw.transpose() * ResultRedistribution_;

        JacobiSVD<MatrixXd> svd(Robot.W, ComputeThinU | ComputeThinV);
        Robot.svd_U = svd.matrixU();

        MatrixXd V2;

        int singular_dof = 6;
        int contact_dof = Robot.J_C.rows();

        V2.setZero(MODEL_DOF, singular_dof);
        V2 = Robot.svd_U.block(0, MODEL_DOF - contact_dof + 6, MODEL_DOF, contact_dof - 6);

        Vector12d desired_force;

        desired_force.setZero();
        MatrixXd Scf_;

        bool right_master = false;

        if (right_master)
        {
            Scf_.setZero(6, 12);
            Scf_.block(0, 0, 6, 6).setIdentity();

            for (int i = 0; i < 6; i++)
            {
                desired_force(i) = -ContactForce_(i) + ForceRedistribution(i);
            }
        }
        else
        {

            Scf_.setZero(6, 12);
            Scf_.block(0, 6, 6, 6).setIdentity();

            for (int i = 0; i < 6; i++)
            {
                desired_force(i + 6) = -ContactForce_(i + 6) + ForceRedistribution(i + 6);
            }
        }

        MatrixXd temp = Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * V2;
        MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);
        MatrixXd Vc_ = V2 * temp_inv;

        Vector6d reduced_desired_force = Scf_ * desired_force;
        torque_contact_ = Vc_ * reduced_desired_force;
    }
    else
    {
        torque_contact_.setZero();
    }

    return torque_contact_;
}

Vector3d WholebodyController::GetZMPpos(RobotData &Robot, bool Local)
{
    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();
    static Vector3d zmp_pos_max = (Eigen::Vector3d() << 0, 0, 0).finished();
    static Vector3d zmp_pos_min = (Eigen::Vector3d() << 0, 0, 0).finished();

    if (Local)
    {
        zmp_pos(0) = (-Robot.ContactForce(4) - (Robot.ee_[0].cp_(2) - P_(2)) * Robot.ContactForce(0) + Robot.ee_[0].cp_(0) * Robot.ContactForce(2) - Robot.ContactForce(10) - (Robot.ee_[1].cp_(2) - P_(2)) * Robot.ContactForce(6) + Robot.ee_[1].cp_(0) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
        zmp_pos(1) = (Robot.ContactForce(3) - (Robot.ee_[0].cp_(2) - P_(2)) * Robot.ContactForce(1) + Robot.ee_[0].cp_(1) * Robot.ContactForce(2) + Robot.ContactForce(9) - (Robot.ee_[1].cp_(2) - P_(2)) * Robot.ContactForce(7) + Robot.ee_[1].cp_(1) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
    }
    else
    {
        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (-Robot.ContactForce(4) - (Robot.ee_[0].cp_(2) - P_(2)) * Robot.ContactForce(0) + Robot.ee_[0].cp_(0) * Robot.ContactForce(2) - Robot.ContactForce(10) - (Robot.ee_[1].cp_(2) - P_(2)) * Robot.ContactForce(6) + Robot.ee_[1].cp_(0) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
            zmp_pos(1) = (Robot.ContactForce(3) - (Robot.ee_[0].cp_(2) - P_(2)) * Robot.ContactForce(1) + Robot.ee_[0].cp_(1) * Robot.ContactForce(2) + Robot.ContactForce(9) - (Robot.ee_[1].cp_(2) - P_(2)) * Robot.ContactForce(7) + Robot.ee_[1].cp_(1) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
        }
        else if (Robot.ee_[0].contact) //left contact
        {
            //std::cout << " f0 : " << rk_.ContactForce(0) << " f1 : " << rk_.ContactForce(1) << " f2 : " << rk_.ContactForce(2) << " f3 : " << rk_.ContactForce(3) << " f4 : " << rk_.ContactForce(4) << " f5 : " << rk_.ContactForce(5) << std::endl;
            //std::cout<<"rk_.ContactForce(4) : "<<rk_.ContactForce(4)<<"rk_.ContactForce(2)"
            //std::cout << "x : " << rk_.ContactForce(4) / rk_.ContactForce(2) << "\t";
            //std::cout << "y : " << rk_.ContactForce(3) / rk_.ContactForce(2) << "\t cp x: " << rk_.link_[Left_Foot].xpos_contact(0) << "\t cp y : " << rk_.link_[Left_Foot].xpos_contact(1) << std::endl;
            zmp_pos(0) = -Robot.ContactForce(4) / Robot.ContactForce(2) + Robot.link_[Left_Foot].xpos_contact(0);
            zmp_pos(1) = -Robot.ContactForce(3) / Robot.ContactForce(2) + Robot.link_[Left_Foot].xpos_contact(1);

            //zmp_pos(0) = (-ContactForce(4) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(0) + rk_.ee_[1].cp_(0) * ContactForce(2) - ContactForce(10) - (rk_.ee_[0].cocp_ntact(2) - P_(2)) * ContactForce(6) + rk_.ee_[0].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            //zmp_pos(1) = (ContactForce(3) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(1) + rk_.ee_[1].cp_(1) * ContactForce(2) + ContactForce(9) - (rk_.ee_[0].cp_(2) - P_(2)) * ContactForce(7) + rk_.ee_[0].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[1].contact) //right contact
        {
            zmp_pos(0) = -Robot.ContactForce(4) / Robot.ContactForce(2) + Robot.ee_[1].cp_(0);
            zmp_pos(1) = -Robot.ContactForce(3) / Robot.ContactForce(2) + Robot.ee_[1].cp_(1);
        }
    }

    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));

    //std::cout << "ZMP position : " << zmp_pos(0) << "\t" << zmp_pos(1) << "\t" << zmp_pos(2) << " " << std::endl;
    //printf("ZMP position : %8.4f  %8.4f  %8.4f  max : %8.4f  %8.4f  %8.4f  min : %8.4f  %8.4f  %8.4f\n", zmp_pos(0), zmp_pos(1), zmp_pos(2), zmp_pos_max(0), zmp_pos_max(1), zmp_pos_max(2), zmp_pos_min(0), zmp_pos_min(1), zmp_pos_min(2));
    for (int i = 0; i < 3; i++)
    {
        if (zmp_pos_max(i) < zmp_pos(i))
            zmp_pos_max(i) = zmp_pos(i);
        if (zmp_pos_min(i) > zmp_pos(i))
            zmp_pos_min(i) = zmp_pos(i);
    }

    if (Robot.control_time_ < 0.01)
    {
        zmp_pos_min.setZero();
        zmp_pos_max.setZero();
    }
    return (zmp_pos);
}

Vector3d WholebodyController::GetZMPpos_fromFT(RobotData &Robot, bool Local)
{
    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();
    static Vector3d zmp_pos_max = (Eigen::Vector3d() << 0, 0, 0).finished();
    static Vector3d zmp_pos_min = (Eigen::Vector3d() << 0, 0, 0).finished();

    if (Local)
    {
        zmp_pos(0) = (-Robot.ContactForce_FT(4) - (Robot.ee_[0].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(0) + Robot.ee_[0].sensor_xpos(0) * Robot.ContactForce_FT(2) - Robot.ContactForce_FT(10) - (Robot.ee_[1].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(6) + Robot.ee_[1].sensor_xpos(0) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
        zmp_pos(1) = (Robot.ContactForce_FT(3) - (Robot.ee_[0].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(1) + Robot.ee_[0].sensor_xpos(1) * Robot.ContactForce_FT(2) + Robot.ContactForce_FT(9) - (Robot.ee_[1].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(7) + Robot.ee_[1].sensor_xpos(1) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
    }
    else
    {
        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (-Robot.ContactForce_FT(4) - (Robot.ee_[0].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(0) + Robot.ee_[0].sensor_xpos(0) * Robot.ContactForce_FT(2) - Robot.ContactForce_FT(10) - (Robot.ee_[1].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(6) + Robot.ee_[1].sensor_xpos(0) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
            zmp_pos(1) = (Robot.ContactForce_FT(3) - (Robot.ee_[0].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(1) + Robot.ee_[0].sensor_xpos(1) * Robot.ContactForce_FT(2) + Robot.ContactForce_FT(9) - (Robot.ee_[1].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(7) + Robot.ee_[1].sensor_xpos(1) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
        }
        else if (Robot.ee_[0].contact) //left contact
        {
            //std::cout << " f0 : " << rk_.ContactForce(0) << " f1 : " << rk_.ContactForce(1) << " f2 : " << rk_.ContactForce(2) << " f3 : " << rk_.ContactForce(3) << " f4 : " << rk_.ContactForce(4) << " f5 : " << rk_.ContactForce(5) << std::endl;
            //std::cout<<"rk_.ContactForce(4) : "<<rk_.ContactForce(4)<<"rk_.ContactForce(2)"
            //std::cout << "x : " << rk_.ContactForce(4) / rk_.ContactForce(2) << "\t";
            //std::cout << "y : " << rk_.ContactForce(3) / rk_.ContactForce(2) << "\t cp x: " << rk_.link_[Left_Foot].xpos_contact(0) << "\t cp y : " << rk_.link_[Left_Foot].xpos_contact(1) << std::endl;
            zmp_pos(0) = -Robot.ContactForce_FT(4) / Robot.ContactForce_FT(2) + Robot.ee_[0].sensor_xpos(0);
            zmp_pos(1) = -Robot.ContactForce_FT(3) / Robot.ContactForce_FT(2) + Robot.ee_[0].sensor_xpos(1);

            //zmp_pos(0) = (-ContactForce(4) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(0) + rk_.ee_[1].cp_(0) * ContactForce(2) - ContactForce(10) - (rk_.ee_[0].cocp_ntact(2) - P_(2)) * ContactForce(6) + rk_.ee_[0].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            //zmp_pos(1) = (ContactForce(3) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(1) + rk_.ee_[1].cp_(1) * ContactForce(2) + ContactForce(9) - (rk_.ee_[0].cp_(2) - P_(2)) * ContactForce(7) + rk_.ee_[0].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[1].contact) //right contact
        {
            zmp_pos(0) = -Robot.ContactForce_FT(4 + 6) / Robot.ContactForce_FT(2 + 6) + Robot.ee_[1].sensor_xpos(0);
            zmp_pos(1) = -Robot.ContactForce_FT(3 + 6) / Robot.ContactForce_FT(2 + 6) + Robot.ee_[1].sensor_xpos(1);
        }
    }

    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));

    //std::cout << "ZMP position : " << zmp_pos(0) << "\t" << zmp_pos(1) << "\t" << zmp_pos(2) << " " << std::endl;
    //printf("ZMP position : %8.4f  %8.4f  %8.4f  max : %8.4f  %8.4f  %8.4f  min : %8.4f  %8.4f  %8.4f\n", zmp_pos(0), zmp_pos(1), zmp_pos(2), zmp_pos_max(0), zmp_pos_max(1), zmp_pos_max(2), zmp_pos_min(0), zmp_pos_min(1), zmp_pos_min(2));
    for (int i = 0; i < 3; i++)
    {
        if (zmp_pos_max(i) < zmp_pos(i))
            zmp_pos_max(i) = zmp_pos(i);
        if (zmp_pos_min(i) > zmp_pos(i))
            zmp_pos_min(i) = zmp_pos(i);
    }

    if (Robot.control_time_ < 0.01)
    {
        zmp_pos_min.setZero();
        zmp_pos_max.setZero();
    }
    return (zmp_pos);
}

Vector3d WholebodyController::GetZMPpos(RobotData &Robot, VectorXd ContactForce, bool Local)
{
    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();
    static Vector3d zmp_pos_max = (Eigen::Vector3d() << 0, 0, 0).finished();
    static Vector3d zmp_pos_min = (Eigen::Vector3d() << 0, 0, 0).finished();

    Vector3d RightFootPos, LeftFootPos;

    LeftFootPos = Robot.ee_[0].cp_;
    RightFootPos = Robot.ee_[1].cp_;

    if (Local)
    {
        zmp_pos(0) = (-ContactForce(4) - (Robot.ee_[0].cp_(2) - P_(2)) * ContactForce(0) + Robot.ee_[0].cp_(0) * ContactForce(2) - ContactForce(10) - (Robot.ee_[1].cp_(2) - P_(2)) * ContactForce(6) + Robot.ee_[1].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        zmp_pos(1) = (ContactForce(3) - (Robot.ee_[0].cp_(2) - P_(2)) * ContactForce(1) + Robot.ee_[0].cp_(1) * ContactForce(2) + ContactForce(9) - (Robot.ee_[1].cp_(2) - P_(2)) * ContactForce(7) + Robot.ee_[1].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
    }
    else
    {
        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (-ContactForce(4) - (LeftFootPos(2) - P_(2)) * ContactForce(0) + LeftFootPos(0) * ContactForce(2) - ContactForce(10) - (RightFootPos(2) - P_(2)) * ContactForce(6) + RightFootPos(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            zmp_pos(1) = (ContactForce(3) - (LeftFootPos(2) - P_(2)) * ContactForce(1) + LeftFootPos(1) * ContactForce(2) + ContactForce(9) - (RightFootPos(2) - P_(2)) * ContactForce(7) + RightFootPos(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[0].contact) //left contact
        {
            //std::cout << " f0 : " << rk_.ContactForce(0) << " f1 : " << rk_.ContactForce(1) << " f2 : " << rk_.ContactForce(2) << " f3 : " << rk_.ContactForce(3) << " f4 : " << rk_.ContactForce(4) << " f5 : " << rk_.ContactForce(5) << std::endl;
            //std::cout<<"rk_.ContactForce(4) : "<<rk_.ContactForce(4)<<"rk_.ContactForce(2)"
            //std::cout << "x : " << rk_.ContactForce(4) / rk_.ContactForce(2) << "\t";
            //std::cout << "y : " << rk_.ContactForce(3) / rk_.ContactForce(2) << "\t cp x: " << rk_.link_[Left_Foot].xpos_contact(0) << "\t cp y : " << rk_.link_[Left_Foot].xpos_contact(1) << std::endl;
            zmp_pos(0) = -ContactForce(4) / ContactForce(2) + LeftFootPos(0);
            zmp_pos(1) = -ContactForce(3) / ContactForce(2) + LeftFootPos(1);

            //zmp_pos(0) = (-ContactForce(4) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(0) + rk_.ee_[1].cp_(0) * ContactForce(2) - ContactForce(10) - (rk_.ee_[0].cocp_ntact(2) - P_(2)) * ContactForce(6) + rk_.ee_[0].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            //zmp_pos(1) = (ContactForce(3) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(1) + rk_.ee_[1].cp_(1) * ContactForce(2) + ContactForce(9) - (rk_.ee_[0].cp_(2) - P_(2)) * ContactForce(7) + rk_.ee_[0].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[1].contact) //right contact
        {
            if (ContactForce.size() == 12)
            {
                zmp_pos(0) = -ContactForce(4 + 6) / ContactForce(2 + 6) + RightFootPos(0);
                zmp_pos(1) = -ContactForce(3 + 6) / ContactForce(2 + 6) + RightFootPos(1);
            }
            else
            {

                zmp_pos(0) = -ContactForce(4) / ContactForce(2) + RightFootPos(0);
                zmp_pos(1) = -ContactForce(3) / ContactForce(2) + RightFootPos(1);
            }
        }
    }

    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));

    //std::cout << dc.time << "ZMP position : " << zmp_pos(0) << "\t" << zmp_pos(1) << "\t" << zmp_pos(2) << " " << std::endl;
    //printf("ZMP position : %8.4f  %8.4f  %8.4f  max : %8.4f  %8.4f  %8.4f  min : %8.4f  %8.4f  %8.4f\n", zmp_pos(0), zmp_pos(1), zmp_pos(2), zmp_pos_max(0), zmp_pos_max(1), zmp_pos_max(2), zmp_pos_min(0), zmp_pos_min(1), zmp_pos_min(2));
    for (int i = 0; i < 3; i++)
    {
        if (zmp_pos_max(i) < zmp_pos(i))
            zmp_pos_max(i) = zmp_pos(i);
        if (zmp_pos_min(i) > zmp_pos(i))
            zmp_pos_min(i) = zmp_pos(i);
    }

    if (Robot.control_time_ < 0.01)
    {
        zmp_pos_min.setZero();
        zmp_pos_max.setZero();
    }
    return (zmp_pos);
}

void WholebodyController::ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    Eigen::MatrixXd W;
    W.setZero(6, 12);

    Eigen::Matrix3d P1_hat, P2_hat;
    P1_hat = DyrosMath::skm(P1);
    P2_hat = DyrosMath::skm(P2);

    for (int i = 0; i < 3; i++)
    {
        W(i, i) = 1.0;
        W(i + 3, i + 3) = 1.0;
        W(i, i + 6) = 1.0;
        W(i + 3, i + 9) = 1.0;

        for (int j = 0; j < 3; j++)
        {
            W(i + 3, j) = P1_hat(i, j);
            W(i + 3, j + 6) = P2_hat(i, j);
        }
    }
    ResultantForce.resize(6);
    ResultantForce = W * F12; //F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    //printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else //sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }

    //printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else //sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }

    //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else //sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }
    //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    eta = eta_s;
    if (eta_s > eta_ub)
    {
        eta = eta_ub;
    }
    else if (eta_s < eta_lb)
    {
        eta = eta_lb;
    }

    if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
    {
        eta = 0.5;
    }

    //std::cout<<"ETA :: "<<eta<<std::endl;

    //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

    //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
    //double etaMx = eta*Mx1Mx2;
    //printf("%f %f \n", Mx1Mx2,etaMx);
    //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
    //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
    //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

    ForceRedistribution(0) = eta * ResultantForce(0);
    ForceRedistribution(1) = eta * ResultantForce(1);
    ForceRedistribution(2) = eta * ResultantForce(2);
    ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
    ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
    ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
    ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
    ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
    ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
    ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
    ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
    ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
    //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

void WholebodyController::ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{

    Eigen::Matrix3d P1_hat, P2_hat;
    P1_hat = DyrosMath::skm(P1);
    P2_hat = DyrosMath::skm(P2);

    Eigen::MatrixXd W;
    W.setZero(6, 12);

    W.block(0, 0, 6, 6) = Eigen::Matrix6d::Identity();
    W.block(0, 6, 6, 6) = Eigen::Matrix6d::Identity();
    W.block(3, 0, 3, 3) = P1_hat;
    W.block(3, 6, 3, 3) = P2_hat;

    // link_[Right_Leg].Rotm;

    // for (int i = 0; i < 3; i++)
    // {
    //   W(i, i) = 1.0;
    //   W(i + 3, i + 3) = 1.0;
    //   W(i, i + 6) = 1.0;
    //   W(i + 3, i + 9) = 1.0;

    //   for (int j = 0; j < 3; j++)
    //   {
    //     W(i + 3, j) = P1_hat(i, j);
    //     W(i + 3, j + 6) = P2_hat(i, j);
    //   }
    // }

    ResultantForce.resize(6);
    ResultantForce = W * F12; //F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    double A_threshold = 0.001;
    ////printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0.");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1.");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2.");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3.");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4.");
            }
        }
    }

    ////printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0;");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1;");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2;");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3;");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4;");
            }
        }
    }

    //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0,");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1,");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2,");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3,");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4,");
            }
        }
    }
    //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    if (eta_s > eta_ub)
    {
        eta = eta_ub;
    }
    else if (eta_s < eta_lb)
    {
        eta = eta_lb;
    }
    else
    {
        eta = eta_s;
    }

    if (eta_ub < eta_lb) //임시...roundoff error로 정확한 해가 안나올때
    {
        //printf("-");
    }
    else if (sqrt(eta_ub * eta_ub + eta_lb * eta_lb) > 1.0) //너무 큰 경계값이 섞여 있을 때
    {
        //printf("_");
    }

    //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

    //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
    //double etaMx = eta*Mx1Mx2;
    //printf("%f %f \n", Mx1Mx2,etaMx);
    //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
    //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
    //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

    ForceRedistribution(0) = eta * ResultantForce(0);
    ForceRedistribution(1) = eta * ResultantForce(1);
    ForceRedistribution(2) = eta * ResultantForce(2);
    ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
    ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
    ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
    ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
    ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
    ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
    ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
    ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
    ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
    //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

Vector3d WholebodyController::COM_traj_with_zmp(RobotData &Robot)
{
    double tc = sqrt(Robot.com_.pos(2) / 9.81);

    //rk_.link_[COM_id].x_traj(1) = ()
    //rk_.link_[COM_id].a_traj = 9.81/rk_.com_.pos(2)*()
}