#include "tocabi_controller/osqp_rapper.h"
#include <iostream>
osQuadraticProgram::osQuadraticProgram()
{
}

int osQuadraticProgram::setup(const int variable_number, const int constraint_number, const Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> &ObjectiveMatrix, const VectorXd &ObjectiveVector, const Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> &ConstraintMatrix, const VectorXd &alb, const VectorXd &aub)
{
    SparseMatrix<double, Eigen::ColMajor, c_int> P_uppertriangle = ObjectiveMatrix.triangularView<Eigen::Upper>();
    VectorXd clb = alb; //.cwiseMax(-OSQP_INFTY);
    VectorXd cub = aub; //.cwiseMax(OSQP_INFTY);

    data.n = variable_number;
    data.m = constraint_number;

    ::csc objective_matrix = {P_uppertriangle.outerIndexPtr()[variable_number], variable_number, variable_number, const_cast<c_int *>(P_uppertriangle.outerIndexPtr()), const_cast<c_int *>(P_uppertriangle.innerIndexPtr()), const_cast<double *>(P_uppertriangle.valuePtr()), -1};
    ::csc constraint_matrix = {ConstraintMatrix.outerIndexPtr()[variable_number], constraint_number, variable_number, const_cast<c_int *>(ConstraintMatrix.outerIndexPtr()), const_cast<c_int *>(ConstraintMatrix.innerIndexPtr()), const_cast<double *>(ConstraintMatrix.valuePtr()), -1};

    data.P = &objective_matrix;
    data.q = const_cast<double *>(ObjectiveVector.data());
    data.A = &constraint_matrix;
    data.l = clb.data();
    data.u = cub.data();

    osqp_set_default_settings(&settings);
    settings.verbose = false;
    //settings.alpha = 1.0; // Change alpha parameter

    return osqp_setup(&work, &data, &settings);
}

int osQuadraticProgram::setup(const MatrixXd &ObjectiveMatrix, const VectorXd &ObjectiveVector, const MatrixXd &ConstraintMatrix, const VectorXd &alb, const VectorXd &aub, const VectorXd &lb, const VectorXd &ub)
{
    int variable_number = ObjectiveVector.size();

    int constraint_a = alb.size();
    int constraint_b = lb.size();

    int constraint_number = constraint_a + constraint_b;

    Eigen::MatrixXd A_new(constraint_number, variable_number);

    A_new.block(0, 0, constraint_a, variable_number) = ConstraintMatrix;
    A_new.block(constraint_a, 0, variable_number, variable_number) = MatrixXd::Identity(variable_number, variable_number);

    SparseMatrix<double, Eigen::ColMajor, c_int> ConstraintMatrix_s = A_new.sparseView();

    SparseMatrix<double, Eigen::ColMajor, c_int> ObjectiveMatrix_s = ObjectiveMatrix.sparseView();
    SparseMatrix<double, Eigen::ColMajor, c_int> P_uppertriangle = ObjectiveMatrix_s.triangularView<Eigen::Upper>();

    VectorXd clb(constraint_number);
    clb.segment(0, constraint_a) = alb; //.cwiseMax(-OSQP_INFTY);
    clb.segment(constraint_a, variable_number) = lb;
    VectorXd cub(constraint_number);
    cub.segment(0, constraint_a) = aub; //.cwiseMax(-OSQP_INFTY);
    cub.segment(constraint_a, variable_number) = ub;

    data.n = variable_number;
    data.m = constraint_number;

    ::csc objective_matrix = {P_uppertriangle.outerIndexPtr()[variable_number], variable_number, variable_number, const_cast<c_int *>(P_uppertriangle.outerIndexPtr()), const_cast<c_int *>(P_uppertriangle.innerIndexPtr()), const_cast<double *>(P_uppertriangle.valuePtr()), -1};
    ::csc constraint_matrix = {ConstraintMatrix_s.outerIndexPtr()[variable_number], constraint_number, variable_number, const_cast<c_int *>(ConstraintMatrix_s.outerIndexPtr()), const_cast<c_int *>(ConstraintMatrix_s.innerIndexPtr()), const_cast<double *>(ConstraintMatrix_s.valuePtr()), -1};

    data.P = &objective_matrix;
    data.q = const_cast<double *>(ObjectiveVector.data());
    data.A = &constraint_matrix;
    data.l = clb.data();
    data.u = cub.data();
    OSQP_ERROR_MESSAGE;

    osqp_set_default_settings(&settings);
    settings.eps_prim_inf = 1.0E-2;
    settings.verbose = false;
    settings.alpha = 1.0; // Change alpha parameter

    osqp_setup(&work, &data, &settings);
    return work->info->status_val;
}

int osQuadraticProgram::solve(VectorXd &primal_solution)
{
    osqp_solve(work);
    primal_solution.resize(data.n);


    if (work->info->status_val == OSQP_SOLVED)
    {
        for (int i = 0; i < data.n; i++)
        {
            primal_solution[i] = work->solution->x[i];
        }
    }
    else
    {
    }

    return work->info->status_val;
}

void osQuadraticProgram::test()
{ /*
    c_float P_x[3] = {
        4.0,
        1.0,
        2.0,
    };
    c_int P_nnz = 3;
    c_int P_i[3] = {
        0,
        0,
        1,
    };
    c_int P_p[3] = {
        0,
        1,
        3,
    };
    c_float q[2] = {
        1.0,
        1.0,
    };
    c_float A_x[4] = {
        1.0,
        1.0,
        1.0,
        1.0,
    };
    c_int A_nnz = 4;
    c_int A_i[4] = {
        0,
        1,
        0,
        2,
    };
    c_int A_p[3] = {
        0,
        2,
        4,
    };
    c_float l[3] = {
        1.0,
        0.0,
        0.0,
    };
    c_float u[3] = {
        1.0,
        0.7,
        0.7,
    };
    c_int n = 2;
    c_int m = 3;

    c_int exitflag = 0;

    Matrix2d P;

    P << 4, 1, 1, 2;

    SparseMatrix<double, Eigen::ColMajor, c_int> P_spars;

    SparseMatrix<double, Eigen::ColMajor, c_int> P_sparse;

    P_spars = P.sparseView(); //.triangularView<Eigen::Upper>(); //.triangularView<Eigen::Upper>();

    P_sparse = P_spars.triangularView<Eigen::Upper>();

    //osqp_update_P(work, P_sparse.valuePtr(), OSQP_NULL, nnza);

    if (data)
    {
        data->n = n;
        data->m = m;

        ::csc objective_matrix = {P_sparse.outerIndexPtr()[2], 2, 2, const_cast<c_int *>(P_sparse.outerIndexPtr()), const_cast<c_int *>(P_sparse.innerIndexPtr()), const_cast<double *>(P_sparse.valuePtr()), -1};

        data->P = &objective_matrix;
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    if (settings)
    {
        osqp_set_default_settings(settings);
        settings->verbose = false;
        settings->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem

    std::cout << "solution status : " << osqp_solve(work) << std::endl;
    std::cout << work->solution->x[0] << std::endl
              << work->solution->x[1] << std::endl;

    std::cout << "test1" << std::endl;

    // Cleanup
    if (data)
    {
        delete (data);
    }
    std::cout << "test1.5" << std::endl;
    if (settings)
        delete (settings);

    std::cout << "test2" << std::endl;*/
}