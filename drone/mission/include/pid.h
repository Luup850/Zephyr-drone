#ifndef PID_H
#define PID_H
#include <cstdlib>
enum RegType { RegP, RegPI, RegPLead, RegPILead };
// Create a class called PID
class PID {

private:
    // Lead values, Y output, U input
    //double yl[2], ul[2];

    // Integral values
    double yi[2], ui[2];

    int tick_count = 0;

    RegType m_type;

    double m_min, m_max;

    double angle_diff(double x, double y);

public:
    // Public only for testing
    double yl[2], ul[2];

    double *measurement;
    double out;
    double ref;
    double Ts;
    bool enable_i, enable_lead, yaw_control;

    /*
    * Coefficients
    */ 
    double tau_d, alpha; // Lead time constant and lead gain
    double tau_i; // Integral time constant
    double kp; // Proportional gain

    /*
    * Constructor
    */
    PID(RegType type, double interval);

    // Returns the measurement with the lead applied
    double lead();
    double integral();
    double output();
    void set_gains(double Kp, double taui, double taud, double a);
    void limit_output(double max, double min);
    void tick();
};
#endif // PID_H