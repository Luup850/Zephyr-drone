#include "controller.h"
#include <cmath>

// Phi, theta, psi. Roll, pitch, yaw.
// Roll: cos(theta) sin(psi) PID_x + (sin(phi) sin(theta) sin(psi) + cos(phi) cos(theta)) PID_y

// Pitch: -cos(theta) cos(psi) PID_x + (-sin(phi) sin(theta) cos(psi) + cos(phi) sin(theta)) PID_y

/// @brief Position controller for x and y.
/// @param px Drones global X-position
/// @param py Drones global Y-position
/// @param pz Drones global Z-position
/// @param roll Drones current roll along the x-axis 
/// @param pitch 
/// @param yaw 
/// @param pid_x 
/// @param pid_y 
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
    double pitch_offset = 0;//-0.0523598776; // In radians
    double roll_offset = 0;//-0.0523598776; // In radians
    m_pid_x->tick();
    m_pid_y->tick();

    // Wrong formular. Fix tomorrow! 10-04-2023
    /*out_roll =  (cos(*theta) * sin(*psi) * m_pid_x->out + 
                (sin(*phi) * sin(*theta) * sin(*psi) + cos(*phi) * cos(*theta)) * -m_pid_y->out) + roll_offset;

    out_pitch = (-cos(*theta) * cos(*psi) * m_pid_x->out + 
                (-sin(*phi) * sin(*theta) * cos(*psi) + cos(*phi) * sin(*theta)) * -m_pid_y->out) + pitch_offset;*/


    // Accidentally switched a theta and a psi. This should work.
    // Formular: [0, 1, 0; 1, 0, 0]*R*[PID_x; PID_y; 0]
    out_roll =  cos(*theta) * sin(*psi) * m_pid_x->out +
                (sin(*phi) * sin(*theta) * sin(*psi) + cos(*phi) * cos(*psi)) * m_pid_y->out;

    out_pitch = cos(*theta) * cos(*psi) * m_pid_x->out +
                (sin(*phi) * sin(*theta) * cos(*psi) - cos(*phi) * sin(*psi)) * m_pid_y->out;
}

void Controller::tick_matlab()
{
    m_pid_x->tick();
    m_pid_y->tick();

    out_roll = m_pid_x->out * sin(*psi) + cos(*psi) * m_pid_y->out;

    out_pitch = m_pid_x->out * cos(*psi) + sin(*psi) * m_pid_y->out;
}