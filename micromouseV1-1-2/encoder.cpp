#include "encoder.h"

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

void handleEncoderLeft() {
  int b = digitalRead(En_LEFT_2);
  if (b> 0)
  {
    encoderLeftCount++;
  }
  else encoderLeftCount--;
}

void handleEncoderRight() {
  int b = digitalRead(En_RIGHT_2);
  if (b> 0)
  {
    encoderRightCount++;
  }
  else encoderRightCount--;
}