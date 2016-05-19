#include "pidcontroller.h"
#include "clockwork.h"

pidController::pidController(float *input, float *output, float *setpoint, int ki, int kp, int kd, unsigned int T)
{
       setInput( input );
       setOutput( output );
       setSetpoint( setpoint );
       setKp( kp );
       setKi( ki );
       setKd( kd ); 
       setErrorSum(0);
       setLastInput(0);
       setTime(T);
}

void pidController::setInput ( float *input )
{
       Input         =      input;
}

float pidController::getInput ()
{
 return *Input;
}

void pidController::setOutput ( float *output )
{
       Output        =      output;
}

float pidController::getOutput ()
{
 return *Output;
}

void pidController::setSetpoint ( float *setpoint )
{
       Setpoint      =    setpoint;
}

float pidController::getSetpoint ()
{
 return *Setpoint;
}

void pidController::setKp ( int kp )
{
       Kp            =      kp;
}

int pidController::getKp ()
{
       return Kp;
}

void pidController::setKi ( int ki )
{
       Ki            =      ki;
}

int pidController::getKi ()
{
       return Ki;
}

void pidController::setKd ( int kd )
{
       Kd            =      kd;
}

int pidController::getKd ()
{
       return Kd;
}

void pidController::setErrorSum(float errs)
{
       errorSum = errs;
}

float pidController::getErrorSum()
{
      return errorSum;
}

void pidController::setLastInput(float lastip)
{
       lastInput = lastip;
}

float pidController::getLastInput()
{
      return lastInput; 
}

void pidController::setTime( unsigned int T)
{
       time = T;
}

unsigned int pidController::getTime()
{
       return time;
}

void pidController::setTunningParameters( int kp, int ki, int kd)
{
       setKp(kp);
       setKi(ki);
       setKd(kd);
}

float pidController::pidCompute()
{
  float error;
  float dInput; 
  float output;
  float errs;
  error = getSetpoint() - getInput();
  errorSum += error;
  dInput = getInput() - getLastInput();
  output = (getKp())*error + (getKi())*getErrorSum()*getTime() - (getKd())*dInput/getTime(); 
  setOutput(&output);
  setLastInput(getInput()); 
  return output;
}

void pidController::setRightMotorSpeed (int rightmotorspeed)
{
  RightMotorSpeed = rightmotorspeed;
}

int pidController::getRightMotorSpeed()
{
       return RightMotorSpeed;
}

void pidController::setLeftMotorSpeed (int leftmotorspeed)
{
  LeftMotorSpeed = leftmotorspeed;
}

int pidController::getLeftMotorSpeed()
{
       return LeftMotorSpeed;
}

void pidController::pidControl( int RightBaseSpeed, int LeftBaseSpeed, int Min, int Max)
{
      
       int rsp;
       int lsp;
       
       rsp = RightBaseSpeed + int (pidCompute());
       if( rsp > Max) rsp = Max;
       if( rsp < Min) rsp = Min;
       setRightMotorSpeed(rsp);
       
       lsp = LeftBaseSpeed  - int (pidCompute());
       if( lsp > Max) lsp = Max;
       if( lsp < Min) lsp = Min;
       setLeftMotorSpeed(lsp);
}
