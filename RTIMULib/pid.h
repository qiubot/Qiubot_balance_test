/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pid.h
 * Author: User
 *
 * Created on August 19, 2016, 4:11 PM
 */

#ifndef PID_H
#define PID_H

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _pre_pre_error;
        double _pre_pre_pre_error;
        double error;
        double _integral;
        
        double Pout;
        double Iout;
        double Dout;
};

class PID
{
    public:
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif /* PID_H */

