class ControllerV2
{
    private:
        double *pos_x, *pos_y, *pos_z;
        double *vel_x, *vel_y, *vel_z;
        double *acc_x, *acc_y, *acc_z;

    public:
        double setpoint_x, setpoint_y, setpoint_z;
        double setpoint_vx, setpoint_vy, setpoint_vz;
        double setpoint_ax, setpoint_ay, setpoint_az;
        void tick();
        ControllerV2(double *px, double *py, double *pz, double *vx, double *vy, double *vz, double *ax, double *ay, double *az);
};