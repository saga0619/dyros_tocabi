#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();
    void compute_slow();
    void compute_fast();
    
    RobotData &rd_;

private:
    Eigen::VectorQd ControlVal_;
};