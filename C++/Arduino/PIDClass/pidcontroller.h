#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

//#include "clockwork.h"

class pidController
{
 public:
    pidController(float *input, float *output, float *setpoint, float Kp, float Ki, float Kd, float T);
    
    void setInput( float *input);
    float getInput();
    
    void setOutput( float *output);
    float getOutput();
    
    void setSetpoint(float *setpoint);
    float getSetpoint();
    
    void setKp( float Kp );
    float  getKp();
    
    void setKi( float Ki );
    float  getKi();
    
    void setKd( float Kd );
    float  getKd();

    void setErrorSum( float errs);
    float getErrorSum();
    
    void setLastError(float lasterror);
    float getLastError();
    
    void setTime( float T);
    float getTime();
    
    void setTunningParameters( float Kp, float Ki, float Kd);

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
 float Kp;
 float Ki;
 float Kd; 
 int RightMotorSpeed;
 int LeftMotorSpeed;
 float time;
 float  errorSum;
 float lastError;
};

#endif
