#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SparseCore>
using namespace Eigen;
class osQuadraticProgram
{
    OSQPWorkspace *work;
    OSQPSettings *settings;
    OSQPData *data;
    int variable_number; // = ObjectiveVector.size();

    int constraint_a; // = alb.size();
    int constraint_b; // = lb.size();

    int constraint_number;

    //c_float *primal_solution_p;
    //c_float *dual_solution_p;
    ::csc objective_matrix;
    ::csc constraint_matrix;

public:
    osQuadraticProgram();
    //void test();
    int solve(VectorXd &primal_solution);
    //int setup(const int variable_number, const int constraint_number, const Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> &ObjectiveMatrix, const VectorXd &ObjectiveVector, const Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> &ConstraintMatrix, const VectorXd &alb, const VectorXd &aub);
    int setup(const MatrixXd &ObjectiveMatrix, const VectorXd &ObjectiveVector, const MatrixXd &ConstraintMatrix, const VectorXd &alb, const VectorXd &aub, const VectorXd &lb, const VectorXd &ub);

    int setup(const MatrixXd &ObjectiveMatrix, const VectorXd &ObjectiveVector, const MatrixXd &ConstraintMatrix, const VectorXd &alb, const VectorXd &aub);
    
    int hotstart(const MatrixXd &ObjectiveMatrix, const VectorXd &ObjectiveVector, const MatrixXd &ConstraintMatrix, const VectorXd &alb, const VectorXd &aub, const VectorXd &lb, const VectorXd &ub);
    //int warmstart(VectorXd &primal_solution);
};