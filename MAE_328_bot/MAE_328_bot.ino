#include <Encoder.h>


///////////////////////////////////////////////////////////
//Motor Constants
///////////////////////////////////////////////////////////

//User has chosen the constants associated with the standard rear differential
//drive version of the robot with the Pololu 50:1 micro metal geared motors.

float encoderCountsPerRotation = 12; // Encoder counts per shaft rotation.
float MotorGearRatio = 51.45; // The gearing ratio of the drive motor being used.
float WheelDiameter = 3.2; // Wheel Diameter in cm.
float AxelLength = 12.9; // Axel length in cm.

int R1 = 9;
int R2 = 10;
int L1 = 11;
int L2 = 12;

///////////////////////////////////////////////////////////
//Motor PID Control Constants
///////////////////////////////////////////////////////////

//Motor Controller Timers
float PIDMotorsTimeStartLeft;
float PIDMotorsTimeStartRight;

//Integral Error Holders
float integralRight;
float integralLeft;
float diffRight;
float diffLeft;
float errorRight;

//Motor PID Control Constants
float kpRightMotor = 1;
float kiRightMotor = 0;
float kdRightMotor = 0;
 
float kpLeftMotor = 1;
float kiLeftMotor = 0;
float kdLeftMotor = 0;


///////////////////////////////////////////////////////////
//Encoder Things
///////////////////////////////////////////////////////////

/* Change these two numbers to the pins connected to your encoder.
   Best Performance: both pins have interrupt capability
   Good Performance: only the first pin has interrupt capability
   Low Performance:  neither pin has interrupt capability */
Encoder RightEnc(2, 7);
Encoder LeftEnc(3, 6);

int motorLeftVolt;//Right HBridge, Left Motor Motor Input Voltage (Arduino PWM Units, int 0-255)
int motorRightVolt;//Right HBridge, Right Motor Motor Input Voltage (Arduino PWM Units, int 0-255)

float oldSpeedErrorLeft; //Left HBridge, Left Motor old rotation speed error for PIDMotorControl Function
float oldSpeedErrorRight; //Left HBridge, Right Motor old rotation speed error for PIDMotorControl Function

int oldMotorPIDEncoderCountLeft;//Left Motor old Encoder Count storage for PIDMotorControl Function
int oldMotorPIDEncoderCountRight;//Right Motor old Encoder Count storage for PIDMotorControl Function


void setup() {
  Serial.begin(9600);
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(L1,OUTPUT);
  pinMode(L2,OUTPUT);
}


void loop() {
  PIDMotorControlRight(10);
  PIDMotorControlLeft(10);

}

void PIDMotorControlRight(float desVel){
    //Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s).

    float timeStep = 10;

    if (millis() - PIDMotorsTimeStartRight >= timeStep){
      float PIDTimeStep = (millis() - PIDMotorsTimeStartRight)/1000.0;//Time step for controller to work on (s).
      
      int countRight = RightEnc.read();

      // Error on individual motors for vel control
      float errorRight = desVel - 2.0 * PI * (countRight - oldMotorPIDEncoderCountRight) / (encoderCountsPerRotation * MotorGearRatio * PIDTimeStep);
      integralRight = errorRight * PIDTimeStep;
      float diffRight = (errorRight - oldSpeedErrorRight) / PIDTimeStep;
      oldSpeedErrorRight = errorRight;
      oldMotorPIDEncoderCountRight = countRight;

      motorRightVolt += int(kpRightMotor*errorRight + kiRightMotor*integralRight + kdRightMotor*diffRight);

      if (motorRightVolt>255){
        motorRightVolt=255;
      }
      if (motorRightVolt<-255){
        motorRightVolt=-255;
      }
      if (motorRightVolt >= 0){
        analogWrite(R1, motorRightVolt);
        analogWrite(R2, 0);
      }
      if (motorRightVolt < 0){
        analogWrite(R2, motorRightVolt);
        analogWrite(R1, 0);
      }
      
      Serial.println(errorRight);
      PIDMotorsTimeStartRight = millis();
    }
}

void PIDMotorControlLeft(float desVel){
    //Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s).

    float timeStep = 10;

    if (millis() - PIDMotorsTimeStartLeft >= timeStep){
      float PIDTimeStep = (millis() - PIDMotorsTimeStartLeft)/1000.0;//Time step for controller to work on (s).
      
      int countLeft = LeftEnc.read();

      // Error on individual motors for vel control
      float errorLeft = desVel - 2.0 * PI * (countLeft - oldMotorPIDEncoderCountLeft) / (encoderCountsPerRotation * MotorGearRatio * PIDTimeStep);
      integralLeft = errorLeft * PIDTimeStep;
      float diffLeft = (errorLeft - oldSpeedErrorLeft) / PIDTimeStep;
      oldSpeedErrorLeft = errorLeft;
      oldMotorPIDEncoderCountLeft = countLeft;

      motorLeftVolt += int(kpLeftMotor*errorLeft + kiLeftMotor*integralLeft + kdLeftMotor*diffLeft);

      if (motorLeftVolt>255){
        motorLeftVolt=255;
      }
      if (motorLeftVolt<-255){
        motorLeftVolt=-255;
      }
      if (motorLeftVolt >= 0){
        analogWrite(R1, motorLeftVolt);
        analogWrite(R2, 0);
      }
      if (motorLeftVolt < 0){
        analogWrite(R2, motorLeftVolt);
        analogWrite(R1, 0);
      }
      
      Serial.println(errorLeft);
      PIDMotorsTimeStartLeft = millis();
    }
}
