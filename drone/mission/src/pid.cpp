#include "pid.h"
#include <iostream>

PID::PID()
{
    m_kp = 1.0;
    m_ki = 0.0;
    m_kd = 0.0;
    m_initialized = false;
    out = 0;
    m_min = -99999;
    m_max = 99999;
    windup_min = -99999;
    windup_max = 99999;
}

void PID::set_minmax(double min, double max)
{
    m_min = min;
    m_max = max;
}

void PID::set_gains(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PID::set_setpoint(double sp)
{
    setpoint = sp;
}

// Set the address of the var that is responsible for the measurement.
void PID::set_measurement(double* m)
{
    measurement = m;
}

// Tick will run on a different thread.
void PID::tick()
{
    if(m_initialized == false)
    {
        m_i_error = 0.0;
        m_d_error = 0.0;
        m_prev_time = clock();
        m_initialized = true;
    }
    double dt = difftime(clock(), m_prev_time)/CLOCKS_PER_SEC;
    //printf("DT %.3f", dt);
    m_error = setpoint - (*measurement);
    m_i_error = m_i_error + m_error * dt;
    
    if(m_i_error > windup_max)
        m_i_error = windup_max;
    else if(m_i_error < windup_min)
        m_i_error = windup_min;

    m_d_error = (m_error - m_prev_error) / dt;

    // Set the controllers Out var.
    double tmp = m_kp * m_error + m_ki * m_i_error + m_kd * m_d_error;
    if(tmp > m_max)
        out = m_max;
    else if(tmp < m_min)
        out = m_min;
    else
        out = tmp;
    //printf("Out: %.5f\n", out);
    m_prev_error = m_error;
    m_prev_time = clock(); 
}

// Limit Integral term, such that it doesn't wind up.
void PID::windup_limit(double min, double max)
{
    windup_min = min;
    windup_max = max;
}