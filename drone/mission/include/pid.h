#ifndef PID_H
#define PID_H
enum RegType { RegP, RegPI, RegPLead, RegPILead };
// Create a class called PID
class PID {

private:
    // Lead values, Y output, U input
    double yl[2], ul[2];

    // Integral values
    double yi[2], ui[2];

    int tick_count = 0;

    double m_min, m_max;

public:

    double *measurement;
    double out;
    double ref;
    double Ts;
    bool enable_i, enable_lead;

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