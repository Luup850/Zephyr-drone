#include "drone.h"
#include <iostream>
#include <pthread.h>

#define HOVER_THRUST 82

void Drone::link_drone_position(double* x, double* y, double* z)
{
    m_x = x;
    m_y = y;
    m_z = z;
}

void Drone::print_drone_position()
{
    std::cout << "Drone position: " << *m_x << ", " << *m_y << ", " << *m_z << std::endl;
}

bool Drone::set_height(double h)
{

}