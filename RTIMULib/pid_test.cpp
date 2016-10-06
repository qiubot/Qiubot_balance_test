/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
    loop_counter = 0;
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    error = setpoint - pv;
    
    errors[3] = errors[2];
    errors[2] = errors[1];
    errors[1] = errors[0];
    errors[0] = error;

    // Proportional term
    Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    Iout = _Ki * _integral;

    // Derivative term
    error_deriv[2] = error_deriv[1];
    error_deriv[1] = error_deriv[0];
    //error_deriv[0] = (errors[0] + 3 * errors[1] - 3 * errors[2] - errors[3])/6;
    error_deriv[0] = (errors[0] - errors[1])/_dt;
    
    filtered_error_deriv[2] = filtered_error_deriv[1];
    filtered_error_deriv[1] = filtered_error_deriv[0];

    if ( loop_counter>2 ) // Let some data accumulate
      filtered_error_deriv[0] = (1/(1+c*c+1.414*c))*(error_deriv[2]+2*error_deriv[1]+error_deriv[0]-(c*c-1.414*c+1)*filtered_error_deriv[2]-(-2*c*c+2)*filtered_error_deriv[1]);
    else
      loop_counter++;
    
    Dout = _Kd * error_deriv[0];

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;
    
    //cout << "P: " << Pout << " I: " << Iout << " D: " << Dout << endl;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif