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

public:
    Logger(char name[]);
    void log(double var[], int len);
};


#endif // LOGGER_H