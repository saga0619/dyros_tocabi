#include "tocabi_controller/terminal.h"
#include <ros/package.h>
#include <fstream>

Tui::Tui(DataContainer &dc_global) : dc(dc_global)
{
    mtx_terminal.lock();
    initscr();
    nodelay(stdscr, TRUE);
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    start_color();

    init_pair(1, -1, -1);
    init_pair(2, COLOR_BLACK, COLOR_WHITE);
    mtx_terminal.unlock();
}

void Tui::tuiThread()
{ //100hz
}

std::string Tui::menu(int y_start, int x_start, int y_dis, int x_dis, int y_length, int x_length, std::string cs[10][10])
{

    int cp[2] = {0, 0};

    int getch;
    while (true)
    {
        getch = getch();
        if (getch == KEY_DOWN)
        {
            cp[0]++;
            if (cp[0] > y_length - 1)
                cp[0] = 0;
        }
        else if (getch == KEY_UP)
        {
            cp[0]--;
            if (cp[0] < 0)
                cp[0] = y_length - 1;
        }
        else if (getch == KEY_LEFT)
        {
            cp[1]--;
            if (cp[1] < 0)
                cp[1] = x_length - 1;
        }
        else if (getch == KEY_RIGHT)
        {
            cp[1]++;
            if (cp[1] > x_length - 1)
                cp[1] = 0;
        }

        for (int i = 0; i < y_length; i++)
        {
            for (int j = 0; j < x_length; j++)
            {
                if ((cp[0] == i) && (cp[1] == j))
                    attron(COLOR_PAIR(2));
                mvprintw(y_start + y_dis * i, x_start + x_dis * j, cs[i][j].c_str());
                if ((cp[0] == i) && (cp[1] == j))
                    attroff(COLOR_PAIR(2));
            }
        }

        if (getch == 10)
        {
            return cs[cp[0]][cp[1]];
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

void rprint(DataContainer &dc_q, int y, int x, const char *str, ...)
{
    va_list lst;
    va_start(lst, str);
    //char buff[1024];
    for (int i = 0; i < 50; i++)
    {

        if (!dc_q.Tq_[i].update)
        {
            mtx_terminal.lock();
            dc_q.Tq_[i].update = true;
            dc_q.Tq_[i].y = y;
            dc_q.Tq_[i].x = x;
            vsnprintf(dc_q.Tq_[i].text, 255, str, lst);
            dc_q.Tq_[i].clr_line = false;
            mtx_terminal.unlock();
            break;
        }
    }
    va_end(lst);
}

void rprint(DataContainer &dc_q, bool clr_line, int y, int x, const char *str, ...)
{

    va_list lst;
    va_start(lst, str);
    //char buff[1024];
    for (int i = 0; i < 50; i++)
    {

        if (!dc_q.Tq_[i].update)
        {
            mtx_terminal.lock();
            dc_q.Tq_[i].update = true;
            dc_q.Tq_[i].y = y;
            dc_q.Tq_[i].x = x;
            vsnprintf(dc_q.Tq_[i].text, 255, str, lst);
            dc_q.Tq_[i].clr_line = clr_line;
            mtx_terminal.unlock();
            break;
        }
    }
    va_end(lst);
}

void rprint_sol(bool ncurse, int y, int x, const char *str, ...)
{
    va_list lst;
    va_start(lst, str);
    //char buff[1024];

    char text[256];
    vsnprintf(text, 255, str, lst);
    if (ncurse)
    {
        mtx_terminal.lock();
        mvprintw(y, x, text);
        refresh();
        mtx_terminal.unlock();
    }
    else
    {
        std::cout << text << std::endl;
    }
    va_end(lst);
}

int kbhit()
{
    struct termios oldt, newt;
    int ch;

 

    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;

 

    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );

 

    ch = getchar();

 

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

 

    return ch;
}

void rprint(DataContainer &dc, const char *str, ...)
{
    va_list lst;
    va_start(lst, str);

    for (int i = 0; i < 100; i++)
    {
        if (dc.Tq_[i].update == false)
        {
            dc.Tq_[i].update = true;
            vsnprintf(dc.Tq_[i].text, 255, str, lst);
        }
    }

    va_end(lst);
}

void Tui::que_clear()
{
    que = 0;
}

void Tui::ReadAndPrint(int y, int x, std::string buff)
{

    if (dc.ncurse_mode)
    {
        std::string desc_package_path = ros::package::getPath("dyros_red_controller");
        std::string text_path = desc_package_path + "/ascii/" + buff;
        std::ifstream my_file(text_path.c_str());
        std::string wc, t_temp;

        if (my_file.is_open())
        {
            while (!my_file.eof())
            {
                getline(my_file, t_temp);
                mvprintw(y++, x, t_temp.c_str());
            }
        }
    }
}

void Tui::endTui()
{
    endwin();
}

void wait_for_keypress()
{
    while (1)
    {
        if (!(getch() == -1))
            break;
    }
}

void wait_for_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
