#ifndef ENCODER_H
#define ENCODER_H

#define En_LEFT_1 32             
#define En_LEFT_2 35            
#define En_RIGHT_1 25              
#define En_RIGHT_2 33              

#include <Arduino.h>
extern volatile long encoderLeftCount;
extern volatile long encoderRightCount;

void handleEncoderLeft();
void handleEncoderRight();
#endif