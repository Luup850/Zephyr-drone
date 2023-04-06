#include "controller.h"
#include <math.h>

// Phi, theta, psi. Roll, pitch yaw.
// Roll: cos(theta) sin(psi) PID_x + (sin(phi) sin(theta) sin(psi) + cos(phi) cos(theta)) PID_y

// Pitch: -cos(theta) cos(psi) PID_x + (-sin(phi) sin(theta) cos(psi) + cos(phi) sin(theta)) PID_y

Controller::Controller(double *px, double *py, double *pz, double *roll, double *pitch, double *yaw, PID *pid_x, PID *pid_y)
{
    pos_x = px;
    pos_y = py;
    pos_z = pz;
    phi = roll;
    theta = pitch;
    psi = yaw;
    m_pid_x = pid_x;
    m_pid_y = pid_y;
}

void Controller::tick()
{
    m_pid_x->tick();
    m_pid_y->tick();

    out_roll = cos(*theta) * sin(*psi) * m_pid_x->out + (sin(*phi) * sin(*theta) * sin(*psi) + cos(*phi) * cos(*theta)) * m_pid_y->out;
    out_pitch = (-cos(*theta)) * cos(*psi) * m_pid_x->out + (-sin(*phi) * sin(*theta) * cos(*psi) + cos(*phi) * sin(*theta)) * m_pid_y->out;
}