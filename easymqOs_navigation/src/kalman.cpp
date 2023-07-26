#include "kalman.h"

Kalman::Kalman(double _q, double _r, double _p, double _initial_value)
{
    q = _q;
    r = _r;
    p = _p;
    x = _initial_value;
}

void Kalman::update(double measurement)
{
    //prediction update
    //omit x = x
    this->p = this->p + this->q;

    //measurement update
    this->k = this->p / (this->p + this->r);
    this->x = this->x + this->k * (measurement - this->x);
    this->p = (1 - this->k) * this->p;
}
void Kalman::update_2(double measurement)
{
    //prediction update
     this->x = this->x+ measurement;
    //omit x = x
    this->p = this->p + this->q;

    //measurement update
    this->k = this->p / (this->p + this->r);
    this->x = this->x + this->k * (measurement - this->x);
    this->p = (1 - this->k) * this->p;
}
void Kalman::update(double r_measurement,double measurement)
{
    //prediction update
    //omit x = x
    this->p = this->p + this->q;

    //measurement update
    this->k = this->p / (this->p + this->r);
    this->x = measurement + this->k * (r_measurement -measurement  );
    this->p = (1 - this->k) * this->p;
}

void Kalman::update_avrage(double r_measurement,double measurement)
{
    //prediction update

    this->x =  (r_measurement + measurement  )/2;
    
}
void Kalman::reset(double _q, double _r, double _p, double _initial_value)
{
    q = _q;
    r = _r;
    p = _p;
    x = _initial_value;
}
