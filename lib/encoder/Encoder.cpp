#include "Encoder.h"
#include <Encoder_Buffer.h>

long encoderLeftLast = 0;
long encoderRightLast = 0;
long lastReadTimeLeft = 0;
long lastReadTimeRight = 0;
double radius = 0;
long tpr = 0;

Encoder::Encoder(int leftPin, int rightPin, double wheelRadius, long ticksPerRevolution)
{
  radius = wheelRadius;
  tpr = ticksPerRevolution;

  encoderLeft = new Encoder_Buffer(leftPin);
  encoderRight = new Encoder_Buffer(rightPin);

  // Initialize encoders
  SPI.begin();
  encoderLeft->initEncoder();
  encoderRight->initEncoder();
}

void Encoder::readEncoders(Encoder::EncoderData &encoderData) {
  long readTime = millis();
  // Encoders are in x4 mode
  long encoderLeftReading = -encoderLeft->readEncoder();
  encoderData.reading.left = encoderLeftReading;
  if(encoderLeftReading != 0){
    long deltaTime = readTime - lastReadTimeLeft;
    if(deltaTime > 0){
      long deltaLeft = encoderLeftReading - encoderLeftLast;
      lastReadTimeLeft = readTime;
      encoderLeftLast = encoderLeftReading;
      encoderData.rpm.left = (static_cast<double>(deltaLeft) * 60000.0) / (tpr * static_cast<double>(deltaTime));
    }
  }

  long encoderRightReading = encoderRight->readEncoder();
  encoderData.reading.right = encoderRightReading;
  if(encoderRightReading != 0){
    long deltaTime = readTime - lastReadTimeRight;
    if(deltaTime > 0){
      long deltaRight = encoderRightReading - encoderRightLast;
      lastReadTimeRight = readTime;
      encoderRightLast = encoderRightReading;
      encoderData.rpm.right = (static_cast<double>(deltaRight) * 60000.0) / (tpr * static_cast<double>(deltaTime));
    }
  }
}