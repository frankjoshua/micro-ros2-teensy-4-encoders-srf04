#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <Encoder_Buffer.h>

class Encoder
{
public:
  struct RPM
  {
    // Meters per second
    double left;
    double right;
  };

  Encoder(int leftPin, int rightPin, double wheelRadius, long ticksPerRevolution);

  Encoder::RPM getRPM();

private:
  Encoder_Buffer *encoderLeft;
  Encoder_Buffer *encoderRight;
};

#endif