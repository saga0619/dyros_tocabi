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
        printf("single thread dyn calc : %8.4f ms              \n", d1);
        if (shutdown_tocabi_bool)
        {
            //std::cout<<"
            printf("thread calc end \n");
            break;
        }
    }
}

void DynamicsManager::testThread()
{
    int testval = 500;
    std::future<MatrixXd> ret[testval];

    printf("Calc SVD %d times \n", testval);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    while (ros::ok())
    {





        if (shutdown_tocabi_bool)
        {
            printf("thread calc end \n");
            break;
        }
    }
}