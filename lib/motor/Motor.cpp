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
  // Accumulate and clamp power in [-1, 1]
  power = constrain(power + delta, -1.0, 1.0);

  // Map to byte with float math, then clamp to the valid range per side
  uint8_t outByte;
  if (direction > 0) {
    // Right motor: map [-1..1] -> [128..255]
    float mapped = 128.0f + ((power + 1.0f) * 0.5f) * (255.0f - 128.0f);
    if (mapped < 128.0f) mapped = 128.0f;
    if (mapped > 255.0f) mapped = 255.0f;
    outByte = static_cast<uint8_t>(mapped + 0.5f);
  } else {
    // Left motor: map [-1..1] -> [1..127]
    float mapped = 1.0f + ((power + 1.0f) * 0.5f) * (127.0f - 1.0f);
    if (mapped < 1.0f) mapped = 1.0f;
    if (mapped > 127.0f) mapped = 127.0f;
    outByte = static_cast<uint8_t>(mapped + 0.5f);
  }

  Serial2.write(outByte);

}
