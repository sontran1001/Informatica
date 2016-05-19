#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_
//#include <iostream>
#include "clockwork.h"

class pidController
{
 public:
    pidController(float *Input, float *Output, float *Setpoint, int Kp, int Ki, int Kd, unsigned int T);
    
    void setInput( float *input);
    float getInput();
    
    void setOutput( float *output);
    float getOutput();
    
    void setSetpoint(float *setpoint);
    float getSetpoint();
    
    void setKp( int Kp );
    int  getKp();
    
    void setKi( int Ki );
    int  getKi();
    
    void setKd( int Kd );
    int  getKd();

    void setErrorSum( float errs);
    float getErrorSum();
    
    void setLastInput(float lastip);
    float getLastInput();
    
    void setTime( unsigned int T);
    unsigned int getTime();
    
    void setTunningParameters( int Kp, int Ki, int Kd);

    float pidCompute();
    
    void setRightMotorSpeed( int rightmotorspeed);
    int  getRightMotorSpeed();
  
    void setLeftMotorSpeed( int leftmotorspeed);
    int  getLeftMotorSpeed();

    void pidControl( int RightBaseSpeed, int LeftBaseSpeed, int Min, int Max);
 private:
 float *Input;
 float *Output;
 float *Setpoint;
 int Kp;
 int Ki;
 int Kd; 
 int RightMotorSpeed;
 int LeftMotorSpeed;
 unsigned int time;
 float  errorSum;
 float lastInput;
};

#endif
