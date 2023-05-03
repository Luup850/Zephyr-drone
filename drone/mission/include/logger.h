#ifndef LOGGER_H
#define LOGGER_H
#include <stdio.h>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <time.h>

class Logger {

private:
    bool firstLog = true;
    clock_t m_time, m_prev_time;
    FILE *lg;
    bool m_logOn;

public:
    Logger(char name[], bool logOn);
    void log(double var[], int len);
    void log_params(char name[], double var[], int len);
};


#endif // LOGGER_H