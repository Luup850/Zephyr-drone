class Controller
{
private:
    /* data */
    double m_kp, m_ki, m_kd;
public:
    void set_gains(double kp, double ki, double kd);
    void tick();
};