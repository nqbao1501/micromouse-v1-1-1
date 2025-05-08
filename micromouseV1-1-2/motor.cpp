#include "motor.h"
#include "encoder.h"
#include "ir.h"


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
sensors_event_t event;
double currentAngle;
double targetAngle;
double yawError = 0;
double prevTime;
double yaw;
double ePrev = 0;
double eIntegral = 0;


void bnoCalibration(){
  sensors_event_t event;
  for (int i = 0; i <1000; i++){
    bno.getEvent(&event); 
    yawError += normalization(event.orientation.x);
    delay(3);
  }
  yawError = yawError/1000;
}
void motorLeftForward(int speed){
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(PWM_CHANNEL_A1, 0);    
  ledcWrite(PWM_CHANNEL_A2, speed); 
}
void motorLeftBackward(int speed){
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(PWM_CHANNEL_A2, 0);    
  ledcWrite(PWM_CHANNEL_A1, speed); 
}

void motorRightForward(int speed){
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(PWM_CHANNEL_B1, 0);    
  ledcWrite(PWM_CHANNEL_B2, speed);
}

void motorRightBackward(int speed){
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(PWM_CHANNEL_B2, 0);    
  ledcWrite(PWM_CHANNEL_B1, speed);
}
void motorForward(int speedRight, int speedLeft){
  motorLeftForward(speedLeft);
  motorRightForward(speedRight);
}
void motorStop(){
  ledcWrite(PWM_CHANNEL_A1, 0);    
  ledcWrite(PWM_CHANNEL_A2, 0);
  ledcWrite(PWM_CHANNEL_B1, 0);    
  ledcWrite(PWM_CHANNEL_B2, 0);
}

double normalization(double x)
{
    if (x > 180)
    {
        x = x - 360;
    }
    else if (x < -180)
    {
        x = x + 360;
    }
    return x;
}

void goStraight(float targetDistance){
  targetDistance = (targetDistance / (3.8* M_PI)) * 210;  
  encoderLeftCount = 0; 
  encoderRightCount = 0;
  long totalDistance = 0;
  double speed = 150;
  
  while (totalDistance < targetDistance && speed > 10) {
    bno.getEvent(&event); 
    yaw = event.orientation.x - yawError;
    yaw = normalization(yaw);
    totalDistance = (encoderLeftCount + encoderRightCount) / 2; 

    if (abs(totalDistance - targetDistance) < 0.35 * targetDistance){
      speed  = speed  - 1.3;
    }

    float errorAngle = normalization(yaw - targetAngle);
    double kpAngle = 3.45;
    double kiAngle = 0.01;
    double kdAngle = 0.12;
    double correctionAngle = pidController(errorAngle, kpAngle, kiAngle, kdAngle);  // Adjust angle correction


    int leftSpeed = constrain(speed - correctionAngle - (getIR(ir_tx1, ir_rx1) - getIR(ir_tx3, ir_rx3))*0.0055, MIN_SPEED, MAX_SPEED);
    int rightSpeed = constrain(speed + correctionAngle + (getIR(ir_tx1, ir_rx1) - getIR(ir_tx3, ir_rx3))*0.0055, MIN_SPEED, MAX_SPEED);
    motorForward(rightSpeed, leftSpeed);
  }
  motorStop();
  delay(500);
}

void turnRight(double angleSet){
  encoderLeftCount = 0, encoderRightCount = 0;
  double steps = angleSet * 1.2+10;
  double speed = 140;
  while (encoderLeftCount <= steps && speed > 10)
  {
      if (steps - encoderLeftCount < 35)
      {
          speed = speed - 2.5;
      }

      long errorEncoder = abs(encoderRightCount) - abs(encoderLeftCount);
      double offsetSpeed = 2 * errorEncoder;


      int leftMotorSpeed = constrain(speed + offsetSpeed, MIN_SPEED, MAX_SPEED);
      int rightMotorSpeed = constrain(speed - offsetSpeed, MIN_SPEED, MAX_SPEED);

      motorRightBackward(rightMotorSpeed);
      motorLeftForward(leftMotorSpeed);
  }
  motorStop();
  targetAngle = normalization(targetAngle + angleSet);
  delay(500);
}

void turnLeft(double angleSet){
  encoderLeftCount = 0, encoderRightCount = 0;
  double steps = angleSet * 1.2 + 20;
  double speed = 140;
  while (encoderRightCount<= steps && speed > 10)
  {
      if (steps - encoderRightCount < 50)
      {
          speed = speed - 10;
      }
      long errorEncoder = abs(encoderRightCount) - abs(encoderLeftCount);
      double offsetSpeed = 2 * errorEncoder;

      int leftMotorSpeed = constrain(speed + offsetSpeed, MIN_SPEED, MAX_SPEED);
      int rightMotorSpeed = constrain(speed - offsetSpeed, MIN_SPEED, MAX_SPEED);

      motorLeftBackward(leftMotorSpeed);
      motorRightForward(rightMotorSpeed);
  }
  motorStop();
  targetAngle = normalization(targetAngle - angleSet);
  delay(500);
}

double pidController(double error, double kp, double ki, double kd) { 
  double currentTime = micros();
  double dT = ((double)(currentTime - prevTime)) / (1.0e6);  // Time difference in seconds
  prevTime = currentTime;
  eIntegral += error * dT;  // Integral term (accumulated error)
  double eDerivative = (error - ePrev) / dT;  // Derivative term (rate of change of error)
  // PID control formula
  double u = (kp * error) + (ki * eIntegral) + (kd * eDerivative);
  // Save previous values for next iteration
  ePrev = error;
  return u;  
}
