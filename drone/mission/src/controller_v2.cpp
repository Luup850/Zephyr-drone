#include "controller_v2.h"
#include <math.h>

// Roll: cos(theta) sin(psi) PID_x + (sin(phi) sin(theta) sin(psi) + cos(phi) cos(theta)) PID_y

// Pitch: -cos(theta) cos(psi) PID_x + (-sin(phi) sin(theta) cos(psi) + cos(phi) sin(theta)) PID_y

ControllerV2::ControllerV2(double *px, double *py, double *pz, double *vx, double *vy, double *vz, double *ax, double *ay, double *az)
{
    pos_x = px;
    pos_y = py;
    pos_z = pz;
    vel_x = vx;
    vel_y = vy;
    vel_z = vz;
    acc_x = ax;
    acc_y = ay;
    acc_z = az;
}

void ControllerV2::tick()
{
    double r32 = k_p * (pz)
    double psi = atan2()
}