#include "controller.h"

void Controller::set_gains(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void Controller::tick()
{
    // Update error value
    // Do PID calc
    // Update I error
}