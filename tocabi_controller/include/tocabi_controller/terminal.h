#ifndef TERMINAL_H
#define TERMINAL_H
#include "tocabi_controller/data_container.h"

#include <stdarg.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>


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

  std::string menu(int y_start, int x_start, int y_dis, int x_dis, int y_length, int x_length, std::string menu_list[10][10]);
};

void wait_for_keypress();
void wait_for_ms(int ms);

void rprint(DataContainer &dc, int y, int x, const char *str, ...);
void rprint(DataContainer &dc, bool clr_line, int y, int x, const char *str, ...);
void rprint(DataContainer &dc, const char *str, ...);

void rprint_sol(bool ncurse, int y, int x, const char *str, ...);

int kbhit();


#endif