/*


*/
#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
//#include <unistd.h>
#include <assert.h>
#include "Defs.h"
#include "Arduino.h"

extern struct current_state cstate;
//#include <Wire.h>s
//#include "FTE.h"
//#include <Servo.h>

//#ifdef __cplusplus
//extern "C" {
//#endif


/*
int wristCount;
int baseRotCount;
*/

/** INITIALIZE **/
int initialize_GPIO();
int sleep_pins(int i);
int systemwide_enable();             // arm rover for motion
int systemwide_disable();
int systemwide_reset();

/** DRIVE **/
int drive_enable();
int drive_disable();
int drive_reset();

void wheel_dir(Motor MOT, int in);
void wheel_pwm(Motor MOT, float duty);

void drive_halt();
void drive_motor_duties(float rf_d, float lf_d, float lm_d, float lr_d, float rr_d, float rm_d);
  void drive_allwheels_dir(int in);

int run_DCM_PUL(Motor MOT, int STATE);
int run_DCM_PWM(Motor MOT, float duty);

float mapfun(float x, float in_min, float in_max, float out_min, float out_max);
void* printer(void* ptr);     // Print Info to serial port
void print_state_message();
void state_update();

//Stepper wristStepper(200, WRI_DIR, WRI_PUL);
//Stepper baseRotStepper(200, BR_DIR, BR_PUL);

//#ifdef __cplusplus
//}
//#endif
