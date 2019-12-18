#include "tocabi_controller/data_container.h"

extern std::mutex mtx;
extern std::mutex mtx_dc;

class DynamicsManager
{
private:
  DataContainer &dc;

public:
  DynamicsManager(DataContainer &dc_global);
  void dynamicsThread();
  void testThread();
};
