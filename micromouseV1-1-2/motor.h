#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MOTOR_A1 16
#define MOTOR_A2 4
#define PWM_CHANNEL_A1 0       // PWM channel for MOTOR_A1 forward
#define PWM_CHANNEL_A2 1       // PWM channel for MOTOR_A2 reverse

#define MOTOR_B1 18            
#define MOTOR_B2 19            
#define PWM_CHANNEL_B1 2       // PWM channel for MOTOR_B1 forward
#define PWM_CHANNEL_B2 3       // PWM channel for MOTOR_B2 reverse

#define PWM_FREQ 5000          // PWM frequency in Hz
#define PWM_RESOLUTION 8       // 8-bit resolution (0-255 range)
#define MIN_SPEED 0
#define MAX_SPEED 220

extern Adafruit_BNO055 bno;
extern sensors_event_t event;
extern double yawError;

void bnoCalibration();

void motorLeftForward(int speed);

void motorLeftBackward(int speed);

void motorRightForward(int speed);

void motorRightBackward(int speed);

void motorStop();

double pidController(double error, double kp, double ki, double kd);
void goStraight(float targetDistance);
double normalization(double x);
void motorForward(int speedRight, int speedLeft);
void turnRight(double angleSet);
void turnLeft(double angleSet);


#endif