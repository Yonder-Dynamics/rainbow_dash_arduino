#include "Drive.h"
#include <math.h>

float DUTY_MULTI = 1;

Vector2f newVector2f(float x, float y)
{
  Vector2f v = {.x = x, .y = y};
  return v;
}

void drive(Vector2f vec)
{
  float norm = sqrt(vec.y*vec.y + vec.x*vec.x);
  float left = (vec.y/norm-vec.x/norm)/2 * DUTY_MULTI;
  float right = (vec.y/norm+vec.x/norm)/2 * DUTY_MULTI;

  drive_motor_duties(right, left, left, left, right , right);
}

Vector2f simple_command_to_direction(char command)
{
  switch (command) {
  case 'l': // LEFT
    return newVector2f(-1, 2);
  case 'u': // UP
    return newVector2f(0, 1);
  case 'r': // RIGHT
    return newVector2f(1, 2);
  case 'd': // DOWN
    return newVector2f(0, 1);
  default:
    return newVector2f(0, 0);
  }
}
