#include "tocabi_controller/dynamics_manager.h"
#include "tocabi_controller/terminal.h"
using namespace Eigen;

MatrixXd mat_inv(MatrixXd mat)
{

    JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
    return mat;
}

DynamicsManager::DynamicsManager(DataContainer &dc_global) : dc(dc_global)
{
}

void DynamicsManager::dynamicsThread(void)
{
    int testval = 1000;
    while (ros::ok())
    {
        mtx.lock();
        MatrixXd mat1;
        ros::Time start = ros::Time::now();
        for (int i = 0; i < testval; i++)
        {
            mat1 = MatrixXd::Random(40, 40);
            JacobiSVD<MatrixXd> svd(mat1, ComputeThinU | ComputeThinV);
        }
        mtx.unlock();
        double d1 = (ros::Time::now() - start).toNSec() / 1.0E+6;
        rprint(dc, "single thread dyn calc : %8.4f ms              ", d1);
        if (dc.shutdown)
        {
            //std::cout<<"
            rprint(dc, "thread calc end ");
            break;
        }
    }
}

void DynamicsManager::testThread()
{
    int testval = 500;
    std::future<MatrixXd> ret[testval];

    rprint(dc, "Calc SVD %d times ", testval);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    while (ros::ok())
    {

        ros::Time start = ros::Time::now();
        MatrixXd mat1, mat2;
        for (int i = 0; i < testval; i++)
        {
            mat1 = MatrixXd::Random(40, 40);
            ret[i] = std::async(&mat_inv, mat1);
        }
        for (int i = 0; i < testval; i++)
        {
            mat2 = ret[i].get();
        }
        double d1, d2;
        d1 = (ros::Time::now() - start).toNSec() / 1.0E+6;

        start = ros::Time::now();

        for (int i = 0; i < testval; i++)
        {
            mat1.Random(40, 40);
            JacobiSVD<MatrixXd> svd(mat1, ComputeThinU | ComputeThinV);
        }
        d2 = (ros::Time::now() - start).toNSec() / 1.0E+6;

        rprint(dc, "single thread : %8.4f ms   multi thread : %8.4f ms              ", d2, d1);

        if (dc.shutdown)
        {
            rprint(dc, "thread calc end ");
            rprint(dc, " ");
            break;
        }
    }
}