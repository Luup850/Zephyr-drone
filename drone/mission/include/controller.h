#include <time.h>

class Controller
{
private:
    /* data */
    time_t m_prev_time;
    double m_kp, m_ki, m_kd;
    bool m_initialized;
    double setpoint;
    double* measurement;
    double m_error, m_i_error, m_d_error, m_prev_error;

public:
    Controller();
    void set_gains(double kp, double ki, double kd);
    void tick();
    void set_setpoint(double sp);
    void set_measurement(double* m);

    double out;
};