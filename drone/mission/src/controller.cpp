#include "controller.h"

Controller::Controller()
{
    m_kp = 1.0;
    m_ki = 0.0;
    m_kd = 0.0;
    m_initialized = false;
}

void Controller::set_gains(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void Controller::set_setpoint(double sp)
{
    setpoint = sp;
}

// Set the address of the var that is responsible for the measurement.
void Controller::set_measurement(double* m)
{
    measurement = m;
}

// Tick will run on a different thread.
void Controller::tick()
{
    if(m_initialized == false)
    {
        m_i_error = 0.0;
        m_d_error = 0.0;
        m_prev_time = clock();
        m_initialized = true;
    }
    double dt = difftime(clock(), m_prev_time)/CLOCKS_PER_SEC;
    m_error = setpoint - (*measurement);
    m_i_error = m_i_error + m_error * dt;
    m_d_error = (m_error - m_prev_error) / dt;

    // Set the controllers Out var.
    double out = m_kp * m_error + m_ki * m_i_error + m_kd * m_d_error;
    m_prev_error = m_error;
    m_prev_time = clock(); 
}