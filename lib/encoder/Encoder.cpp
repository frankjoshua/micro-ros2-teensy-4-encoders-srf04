#include "Encoder.h"
#include <Encoder_Buffer.h>

long encoderLeftLast = 0;
long encoderRightLast = 0;
long lastReadTime = 0;
double radius = 0;
long tpr = 0;

Encoder::Encoder(int leftPin, int rightPin, double wheelRadius, long ticksPerRevolution)
{
  radius = wheelRadius;
  tpr = ticksPerRevolution;
  encoderLeft = new Encoder_Buffer(leftPin);
  encoderRight = new Encoder_Buffer(rightPin);
  // Setup Encoders
  SPI.begin();
  encoderLeft->initEncoder();
  encoderRight->initEncoder();
}

Encoder::RPM Encoder::getRPM()
{
  Encoder::RPM rpm;
  long encoderLeftReading = -encoderLeft->readEncoder() / 4; //Read Encoder
  long encoderRightReading = encoderRight->readEncoder() / 4;
  double deltaLeft = encoderLeftReading - encoderLeftLast;
  double deltaRight = encoderRightReading - encoderRightLast;
  long readTime = millis();
  long deltaTime = readTime - lastReadTime;
  lastReadTime = readTime;
  encoderLeftLast = encoderLeftReading;
  encoderRightLast = encoderRightReading;

  rpm.left = (deltaLeft / tpr / deltaTime) * 60000.0;
  rpm.right = (deltaRight / tpr / deltaTime) * 60000.0;
  return rpm;
}