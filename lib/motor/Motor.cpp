#include "Motor.h"
#include "Arduino.h"

Motor::Motor(int motorNumber, int dir)
{
  direction = dir;
  motorCommand = "M" + String(motorNumber) + ": ";
  Serial2.begin(9600);
}

void Motor::adjust(double delta)
{
  power = power + delta;
  // Adjust power based on motor direction
  if (direction > 0) {
    // Right motor: 128 to 255
    Serial2.write((int) map(power, -1, 1, 128, 255));
  } else {
    // Left motor: 1 to 127
    Serial2.write((int) map(power, -1, 1, 1, 127));
  }
  // power = constrain(power + delta, -2048, 2048) * direction;
  // Serial2.print(motorCommand);
  // Serial.print(power);
  
}