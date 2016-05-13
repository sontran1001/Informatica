  
#include "MeOrion.h"
#include <SoftwareSerial.h>

#define Kp 20 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line

MeLineFollower left_lineFinder(PORT_3);
MeLineFollower right_lineFinder(PORT_4);

MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); 
  // calibration code here
  
} 


int lastError = 0;
void loop()
{
  unsigned int sensors[6];
  int position = readPosition(); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  {
      MotorL.run(leftMotorSpeed);
      MotorR.run(rightMotorSpeed);
  }
}

int readPosition(){
  int left_sensorState = left_lineFinder.readSensors();
  int right_sensorState = right_lineFinder.readSensors();
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_OUT) return -3;
  if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_OUT_S2_OUT) return -2;
  if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_OUT_S2_OUT) return -1;
  if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_IN_S2_OUT) return 0;
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_IN_S2_IN) return 1;
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_IN) return 2;
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_OUT) return 3;
  
}
