class Kalman
{
private:
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain

public:
    Kalman(double _q, double _r, double _p, double _initial_value);
    void update(double measurement);
    double GetValue() {return x;}
    void reset(double _q, double _r, double _p, double _initial_value);
};
