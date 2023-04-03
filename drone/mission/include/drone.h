class Drone
{
private:
    /* data */
    // Address of the drones x, y, z coordinates.
    double* m_x; 
    double* m_y; 
    double* m_z;

    double* x_vel;
    double* y_vel;
    double* z_vel;

public:
    // Methods
    void link_drone_position(double* x, double* y, double* z);
    void print_drone_position();
    bool set_height(double h);
};