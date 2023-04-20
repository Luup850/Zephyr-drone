#include "pid.h"
#include <stdio.h>

PID::PID(RegType type, double interval)
{
    Ts = interval;
    yaw_control = false;
    m_type = type;

    // Initialize Lead values
    yl[0] = 0;
    yl[1] = 0;
    ul[0] = 0;
    ul[1] = 0;

    // Initialize Integral values
    yi[0] = 0;
    yi[1] = 0;
    ui[0] = 0;
    ui[1] = 0;

    // Initialize min/max
    m_min = -99999;
    m_max = 99999;

    enable_i = false;
    enable_lead = false;

    if(type == RegPI or type == RegPILead)
        enable_i = true;

    if(type == RegPLead or type == RegPILead)
        enable_lead = true;
}

double PID::lead()
{
    /*
        tf = (Tau * s + 1) / (alpha * Tau * s + 1)
    
    */

    if (enable_lead == false)
    {
        yl[0] = *measurement;
        return *measurement;
    }

    if(tick_count == 0)
    {
        ul[0] = *measurement;
        //printf("Lead: a=%.3f, tau_d=%.3f, Ts=%.9f, yl[1]=%.3f, yl[0]=%.3f, ul[1]=%.3f ul[0]=%.3f\n\n",alpha, tau_d, Ts, yl[1], yl[0], ul[1], ul[0]);
    }
    else
    {
        ul[1] = ul[0];
        ul[0] = *measurement;
        yl[1] = yl[0];
        //printf("Lead: a=%.3f, tau_d=%.3f, Ts=%.9f, yl[1]=%.3f, yl[0]=%.3f, ul[1]=%.3f ul[0]=%.3f\n\n",alpha, tau_d, Ts, yl[1], yl[0], ul[1], ul[0]);
    }
    //printf("Lead: a=%.3f, tau_d=%.3f, Ts=%.9f, yl[1]=%.3f, yl[0]=%.3f, ul[1]=%.3f ul[0]=%.3f\n\n",alpha, tau_d, Ts, yl[1], yl[0], ul[1], ul[0]);
    // Lead |            1            |   |   2      |   |       3         |   |        4        |   |          5              |
    yl[0] = ((2 * alpha * tau_d * yl[1]) - (Ts * yl[1]) + (2 * tau_d * ul[1]) - (2 * tau_d * ul[1]) + 
            (Ts * ul[0]) + (Ts * ul[1])) / (2 * alpha * tau_d + Ts);
    return yl[0];
}

double PID::integral()
{
    /*
        tf = (1/tau) * (1/s)
    
    */

    if (enable_i == false)
    {
        yi[0] = 0;
        return 0;
    }

    if(tick_count == 0)
    {
        if (yaw_control == false)
        {
        // Controller input, minus lead output, times kp.
        ui[0] = (ref - yl[0]) * kp;
        }
        else
            ui[0] = angle_diff(ref, yl[0]) * kp;
    }
    else
    {
        ui[1] = ui[0];
        
        if (yaw_control == false)
            ui[0] = (ref - yl[0]) * kp;
        else
            ui[0] = angle_diff(ref, yl[0]) * kp;
        
        yi[1] = yi[0];
    }

    // Integral
    yi[0] = ((2 * tau_i * yi[1]) + (Ts * ui[1]) + (Ts * ui[0])) / (2 * tau_i);
    
    return yi[0];
}

double PID::output()
{
    double res;

    if(yaw_control == false)
        res = (ref - yl[0]) * kp + yi[0];
    else
        res = angle_diff(ref, yl[0]) * kp + yi[0];
    
    if(res < m_min)
    {
        out = m_min;
        return m_min;
    }
    else if(res > m_max)
    {
        out = m_max;
        return m_max;
    }
    else
    {
        out = res;
        return res;
    }
}

// Need to run at the same speed as Ts
void PID::tick()
{
    if(m_type == RegHeight)
    {
        yl[0] = *measurement;
    }
    else
    {
        lead();
    }

    integral();
    output();
    // Last thing to do.
    tick_count++;
}

void PID::set_gains(double Kp, double taui, double taud, double a)
{
    tau_d = taud;
    alpha = a; // Lead time constant and lead gain
    tau_i = taui; // Integral time constant
    kp = Kp; // Proportional gain
}

void PID::limit_output(double max, double min)
{
    m_min = min;
    m_max = max;
}

// x = ref, y = measurement.
double PID::angle_diff(double x, double y)
{
    double out = 0.0;
    double a = x + 180.0;
    double b = y + 180.0;

    if (abs(a - b) <= 180)
        if(a > b)
            out = (b - a);
        else
            out = (b - a);
    else
        if(a > b)
            out = (-360) + (a - b);
        else
            out = 360 + (a - b);
        out = (a - b);

    //printf("Test1 %.3f\n", out);
    return out;
}