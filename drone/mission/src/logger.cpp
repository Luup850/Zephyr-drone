#include "logger.h"

Logger::Logger(char name[])
{
    // Make log file with current time as name
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    char *time = ctime(&now_c);
    time[strlen(time)-1] = '\0';
    char filename[100];
    sprintf(filename, "logs/%s.log", time);
    lg = fopen(filename, "a+");

    fprintf(lg, "Time");
    fprintf(lg, ", %s", name);
    fprintf(lg, "\n");
}

void Logger::log(double var[], int len)
{
    double time;
    if(firstLog)
    {
        m_time = clock();
        m_prev_time = clock();
        firstLog = false;
    }

    if((difftime(clock(), m_prev_time)/CLOCKS_PER_SEC) > 0.250) 
    {
        m_prev_time = clock();
        time = (difftime(clock(), m_time)/CLOCKS_PER_SEC);

        fprintf(lg, "%.3f", time);

        for(int i = 0; i < len; i++)
        {
            fprintf(lg, ",%.3f", var[i]);
        }
        fprintf(lg, "\n");
    }
}