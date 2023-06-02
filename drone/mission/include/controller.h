#include "pid.h"

// Controller implementation, that converts global positional error to pitch/roll commands in drone inertial frame.
class Controller
{
    private:
        double *pos_x, *pos_y, *pos_z;
        double *phi, *theta, *psi;
        PID *m_pid_x, *m_pid_y;

    public:
        double setpoint_x, setpoint_y, setpoint_z;
        double out_pitch, out_roll;

        double min = -10.0,max = 10.0;

        void tick();
        void tick_matlab();
        Controller(double *px, double *py, double *pz, double *roll, double *pitch, double *yaw, PID *pid_x, PID *pid_y);
};