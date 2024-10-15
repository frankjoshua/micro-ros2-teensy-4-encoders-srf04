#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <Encoder_Buffer.h>

class Encoder
{
public:
  
  struct EncoderReading
  {
    long left;
    long right;
  };

  struct RPM
  {
    // Meters per second
    double left;
    double right;
    double testLeft;
    double testRight;
  };

  struct EncoderData
  {
    EncoderReading reading;
    RPM rpm;
  };

  Encoder(int leftPin, int rightPin, double wheelRadius, long ticksPerRevolution);

  void readEncoders(Encoder::EncoderData &rpm);

private:
  Encoder_Buffer *encoderLeft;
  Encoder_Buffer *encoderRight;

  int leftPin;
  int rightPin;
  double wheelRadius;
  long ticksPerRevolution;
  
};

#endif
