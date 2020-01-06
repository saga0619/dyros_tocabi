#include "tocabi_controller/terminal.h"
#include <ros/package.h>
#include <fstream>

Tui::Tui(DataContainer &dc_global) : dc(dc_global)
{
}

void Tui::tuiThread()
{ //100hz
}

void rprint(DataContainer &dc, const char *str, ...)
{
    va_list lst;
    va_start(lst, str);

    for (int i = 0; i < 50; i++)
    {
        if (!dc.Tq_[i].update)
        {
            mtx_terminal.lock();
            dc.Tq_[i].update = true;
            vsnprintf(dc.Tq_[i].text, 255, str, lst);
            mtx_terminal.unlock();
            break;
        }
    }

    va_end(lst);
}

int kbhit()
{
  struct termios oldt, newt;
  int ch;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

return ch;
}

void wait_for_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
