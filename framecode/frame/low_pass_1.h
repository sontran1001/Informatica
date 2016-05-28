#ifndef LOW_PASS_1
#define LOW_PASS_1

class Low_pass {
public:
    void init(double f_c, double y_0, double dt); // f_c = frequency cut, y_0 = initial value, dt = time step
    Low_pass(double f_c, double dt);
    Low_pass(double f_c, double y_0, double dt);
    double update(double u);
private:
    double _y_p;
    double _omega_cut;
    double _alpha;
};








#endif