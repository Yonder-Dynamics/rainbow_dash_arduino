#pragma once
#include "Dash.h"


typedef struct Vector2f {
  float x;
  float y;
} Vector2f;

Vector2f newVector2f(float x, float y);

void drive(Vector2f vec);
Vector2f simple_command_to_direction(char command);
