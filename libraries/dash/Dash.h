/*


*/
#ifndef COURAGE_H_
#define COURAGE_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
//#include <unistd.h>
#include <assert.h>
#include "Defs.h"
#include "Arduino.h"

//#include <Wire.h>s
//#include "FTE.h"
//#include <Servo.h>

#ifdef __cplusplus
extern "C" {
#endif

current_state cstate;
//target_state tstate;

//Servo srv;
// ///// DC PWM Motors /////
// // Drive Motors with 2 Encoders on LMP, RMP
const Motor LFP = {lLFP, DRIVE, SLEEP6, LFP_DIR, LFP_PWM, 0, 0, 0};	// G2 - Left Front Power: 	DIR P8.14 | PWM P8.13
const Motor LMP = {lLMP, DRIVE, SLEEP6, LMP_DIR, LMP_PWM, 0, 0, 0};	// G2 - Left Middle Power:	DIR P8.15 | PWM P8.17
const Motor LRP = {lLRP, DRIVE, SLEEP6, LRP_DIR, LRP_PWM, 0, 0, 0};	// G2 - Left Rear Power: 	DIR P8.43 | PWM P8.45
const Motor RFP = {lRFP, DRIVE, SLEEP6, RFP_DIR, RFP_PWM, 0, 0, 0};	// G2 - Right Front Power:	DIR P8.32 | PWM P8.34
const Motor RMP = {lRMP, DRIVE, SLEEP6, RMP_DIR, RMP_PWM, 0, 0, 0};	// G2 - Right Middle Power:	DIR P8.38 | PWM P8.36
const Motor RRP = {lRRP, DRIVE, SLEEP6, RRP_DIR, RRP_PWM, 0, 0, 0};	// G2 - Right Rear Power:	DIR P8.44 | PWM P8.46
const Motor LFS = {lLFS, SWERVE, 0, 0, 0, 0, LFS_IN1, LFS_IN2};	// L2 - Left Front Steer: 	_IN1 P8.27 | _IN2 P8.29
const Motor LRS = {lLRS, SWERVE, 0, 0, 0, 0, LRS_IN3, LRS_IN4};	// L2 - Left Rear Steer: 	_IN3 P8.39 | _IN4 P8.41
const Motor RFS = {lRFS, SWERVE, 0, 0, 0, 0, RFS_IN1, RFS_IN2};	// L2 - Right Front Steer:	_IN1 P8.31 | _IN2 P8.37
const Motor RRS = {lRRS, SWERVE, 0, 0, 0, 0, RRS_IN3, RRS_IN4};	// L2 - Right Rear Steer:	_IN3 P8.40 | _IN4 P8.42

// Arm Linear Actuators with Pot Feedback
const Motor BAS = {lBAS, ARM, 0, BAS_DIR, BAS_PUL, 0, 0, 0};	// MD - Base Linear Act:	DIR P9.28 | PWM P9.15
const Motor ELB = {lELB, ARM, 0, ELB_DIR, ELB_PUL, 0, 0, 0};	// MD - Elbow Linear Act:	DIR P9.27 | PWM P9.23
const Motor FOR = {lFOR, ARM, 0, FOR_DIR, FOR_PUL, 0, 0, 0};	// MD - Forearm Lin Act:	DIR P9.12 | PWM P9.14
//Sample Motor Rotator with limit Switch Click
const Motor SMP = {lSMP, SAMPLE, 0, 0, 0, 0, SMP_IN1, SMP_IN2};	// L2 - Sample Motor:		_IN1 P8.26 | _IN2 P8.28
// TB6600 Stepper Motor Drivers
const Motor BR     = {lBR, STEP, ENA_BR, BR_DIR, BR_PUL, SPDBR, 0, 0};	// TB - Base Rotation:		DIR P8.08 | PUL P8.10
const Motor WRI    = {lWRI, STEP, ENA_WRI, WRI_DIR, WRI_PUL, SPDWRI, 0, 0};	// TB - Wrist Rotation:		DIR P8.07 | PUL P8.09
const Motor CLA    = {lCLA, CLAMP, 0, 0, 0, 0, CLA_IN3, CLA_IN4};	// L2 - Hand Clamp:			DIR P8.16 | PUL P8.18
const Motor ELMAG  = {lELMAG, EMAG, 0, 0, EMACT, 0, 0, 0};		// SW - Electromagnet:		ACT P8.19

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

int run_DCM_PUL(Motor MOT, int STATE);
int run_DCM_PWM(Motor MOT, float duty);

float mapfun(float x, float in_min, float in_max, float out_min, float out_max);
void* printer(void* ptr);     // Print Info to serial port
void print_state_message();
void state_update();

//Stepper wristStepper(200, WRI_DIR, WRI_PUL);
//Stepper baseRotStepper(200, BR_DIR, BR_PUL);

#ifdef __cplusplus
}
#endif
#endif
