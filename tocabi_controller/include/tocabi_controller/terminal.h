#ifndef TERMINAL_H
#define TERMINAL_H
#include "tocabi_controller/data_container.h"

#include <stdarg.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

extern std::mutex mtx;
extern std::mutex mtx_dc;
extern std::mutex mtx_terminal;
extern std::mutex mtx_ncurse;

class Tui
{
public:
  Tui(DataContainer &dc_global);
  DataContainer &dc;
  int que;
  void que_clear();
  bool ncurse_;

  void ReadAndPrint(int y, int x, std::string buff);
  void testThread();
  void tuiThread();
  void endTui();
};

void pub_to_gui(DataContainer &dc, const char *str, ...);
void wait_for_keypress();
void wait_for_ms(int ms);

//void rprint(DataContainer &dc, const char *str, ...);

int kbhit();

#endif