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

int kbhit(void)
{

    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO | ISIG);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    int nread = read(0, &ch, 1);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (nread >= 1)
    {
        return ch;
    }
    else
    {
        return -1;
    }
}

void wait_for_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
