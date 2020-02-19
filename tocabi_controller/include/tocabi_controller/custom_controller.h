#include <tocabi_controller/data_container.h>
#include "math_type_define.h"

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();
    void compute();
    
    RobotData &rd_;

private:
    Eigen::VectorQd ControlVal_;
};