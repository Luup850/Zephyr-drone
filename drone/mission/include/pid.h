#ifndef MY_PID
#define MY_PID
#include <time.h>

class PID
{
private:
    /* data */
    time_t m_prev_time;
    double m_kp, m_ki, m_kd;
    bool m_initialized;
    double setpoint;
    double* measurement;
    double m_error, m_i_error, m_d_error, m_prev_error;
    double m_min, m_max, windup_min, windup_max;

public:
    PID();
    void set_gains(double kp, double ki, double kd);
    void tick();
    void set_setpoint(double sp);
    void set_measurement(double* m);
    void set_minmax(double min, double max);
    void windup_limit(double, double);

    double out;
};
#endif // MY_PID