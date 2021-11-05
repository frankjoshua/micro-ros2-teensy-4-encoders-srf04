#include "Odometry.h"
#include "Arduino.h"

Odometry::Position position;
long lastUpdate = 0;

Odometry::Odometry(/* args */)
{
}

Odometry::~Odometry()
{
}

Odometry::Position Odometry::calculatePosition(double xLinearVelocity, double zAngularVelocity)
{
  double millisSinceLastUpdate = millis() - lastUpdate;
  lastUpdate = millis();
  double deltaTime = millisSinceLastUpdate / 1000.0;

  double dx = xLinearVelocity;
  double dtheta = zAngularVelocity;
  position.x += cos(position.theta) * dx * deltaTime;
  position.y += sin(position.theta) * dx * deltaTime;
  position.theta += dtheta * deltaTime;
  if (position.theta > 3.14)
    position.theta = -3.14;

  return position;
}

void Odometry::reset()
{
  position.theta = 0;
  position.x = 0;
  position.y = 0;
  lastUpdate = 0;
}