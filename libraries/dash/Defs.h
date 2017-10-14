#pragma once

#define ARMED 		1
#define DISARMED 	0

/**
MEGA PINOUTS
**/

#define LFP_PWM	10
#define LMP_PWM	8
#define LRP_PWM	9
#define RFP_PWM	7
#define RMP_PWM	6
#define RRP_PWM	5

#define SERVO	4

#define LFP_DIR	22
#define LMP_DIR	23
#define LRP_DIR	24
#define RFP_DIR	25
#define RMP_DIR	26
#define RRP_DIR	27

#define LFS_IN1	28
#define LFS_IN2	29
#define LRS_IN3	30
#define LRS_IN4	31
#define RFS_IN1	32
#define RFS_IN2	33
#define RRS_IN3	34
#define RRS_IN4	35

#define SLEEP1 63
#define SLEEP2 64
#define SLEEP3 65
#define SLEEP4 66
#define SLEEP5 67
#define SLEEP6  36


#define ENA_BR 	37
#define SMP_IN1	38
#define SMP_IN2	39
#define CLA_IN3	40
#define CLA_IN4	41

#define FOR_DIR	42
#define FOR_PUL	43
#define ELB_DIR	44
#define ELB_PUL	45
#define BAS_DIR	46
#define BAS_PUL	47
#define BR_DIR	48
#define BR_PUL	49

#define WRI_DIR	50
#define WRI_PUL	51
#define EMACT	52
#define ENA_WRI	53

/* Limit Switches */
#define BLSL	2
#define BLSR	3
#define CLST	18
#define CLSB	19
#define SLS	20
#define FTE	21

//Swerve Pots
#define LFS_POT	54
#define LRS_POT	55
#define RFS_POT	57
#define RRS_POT	56

//Arm Pots
#define FOR_POT	58
#define ELB_POT	59
#define BAS_POT	60
#define BR_POT	61
#define BATT_VOLT	62

/**
LABELS
**/
#define DRIVE		0
#define ARM	    1
#define SWERVE	2
#define CLAMP	  3
#define SAMPLE  4
#define STEP 		5
#define EMAG	  6
#define lLFP 	  0
#define lLMP 		1
#define lLRP 		2
#define lRFP 		3
#define lRMP 		4
#define lRRP 		5
#define lLFS		9 // Swerve
#define lLRS		10 // Swerve
#define lRFS		11 // Swerve
#define lRRS		12 //
#define lSMP		13 // Drill motor
#define lBR			14 // Base motor
#define lWRI		15 //
#define lCLA		16 // Clamp motor
#define lELMAG	17 // Electromagnet
#define lBAS		6 // Base arm segment
#define lELB		7 // Elbow arm segment
#define lFOR		8 // Forearm segment

#define SPDBR 		75			  // Default Speed in RPM for Stepper motors
#define SPDWRI		75
#define STPBR		10 			// Num steps to step for stepper motors
#define STPWRI		10

/**
DRIVE PARAMETERS - Swerve Drive Parameters, Range of -pi/2 to pi/2
**/
#define LFS_MIN		928			  // _MIN ADC Value - should be aligned to pi/2 (-90 degrees)
#define LFS_MAX		1023			  // _MAX ADC Value - should be aligned to pi/2 (90 degrees)
#define RFS_MIN		178			  // _MIN ADC Value
#define RFS_MAX		1023			  // _MAX ADC Value
#define LRS_MIN		0			  // _MIN ADC Value
#define LRS_MAX		1023			  // _MAX ADC Value
#define RRS_MIN		300			  // _MIN ADC Value
#define RRS_MAX		1023		  	// _MAX ADC Value

// Base Rotation Parameters
#define BRGR		    13			// 13:1 Stepper:Arm Gear ratio
#define BLStheta   -1			// Limit switch offset

#define ACTUATOR_FOR_LENGTH .24
#define ACTUATOR_ELB_LENGTH .31
#define ACTUATOR_BAS_LENGTH .31

// Base Parameters
#define BASLEN		  .1524		// Maximum Length 15.24 cm [6"]
#define BASMIN		  0			  // Min ADC Value
#define BASMAX		  4095		// Max ADC Value

// Elbow Parameters
#define ELBLEN		  .1524   // Maximum Length 15.24 cm [6"]
#define ELBMIN		  0		    // Min ADC Value
#define ELBMAX		  4095    // Max ADC Value

// Forearm Parameters
#define FORLEN		  .9		// Maximum Length 10.16 cm [4"]
#define FORMIN		  0			  // Min ADC Value
#define FORMAX		  4095		// Max ADC Value

// Hand Clamp Parameters
#define	CLAMPTIME  2		  	//Typical time till full open or close

/**
DRIVE PARAMETERS - Swerve Drive Parameters, Range of -pi/2 to pi/2
**/
#define MAX_RAD   (PI/2)

// Encoder clicks
#define ENCPPR		7			    // Pulses Per Revolution
#define WHEELD		29.972		// Wheel Diameter
#define TOL 		  1	     		// Tolerance on moving to a specific position arm length
#define RAD_TOL		0.003	  	// Tolerance on radian of Base
#define STREV		  200			    // Steps per revolution ofr stepper motors
#define CW 			  0
#define CCW			  1

typedef struct Motor {
	int LAB;		// label for Motor
	int TYPE;		// Motor TYPE and Driver (Drive, Arm, Swerve, Clamp, Sample, Step)
	int ENA;		// PIN Enable LOW | Disable HIGH
	int DIR;		// PIN HIGH or LOW to change directions
	int PULSE;		// PIN Pulse or PWM
	int SPD;		// Speed of Stepper motor ONly
	int INA;		// For L298N MOTORS ONLY _IN1|3
	int INB;		// For L298N MOTORS ONLY _IN2|4
	int REV;
	} Motor;

/*
int toggle_rover_state() {
	cstate.states=cstate.states & rover_state_bm;

}

// BITMASKS
#define rover_state_bm   0x1
#define arm_state_bm     0x2
#define drive_state_bm   0x4
#define stepper_state_bm 0x8
#define elmag_state_bm   0x10
#define base_limit_l_bm  0x20
#define base_limit_r_bm  0x40
#define carou_limit_bm   0x80
#define hand_limit_bm    0x100
#define reverse_bm       0x200
#define in_place_bm      0x400
*/

typedef struct current_state {

	int rover_state;
	int arm_state;				// ARMED or DISARMED for arm
	int drive_state;			// ARMED or DISARMED for drive
	int stepper_ena;			// Stepper Enable Pin ARMED or DISARMED*/
	float LFP_duty;				// Left Front Power Motor Duty
	float LMP_duty;				// Left Middle Power Motor Duty
	float LRP_duty;				// Left Rear Power Motor Duty
	float RFP_duty;				// Right Front Power Motor Duty
	float RMP_duty;				// Right Middle Power Motor Duty
	float RRP_duty;				// Right Rear Power Motor Duty
	int LMP_encoder;			// Left Middle Encoder Value (1 rev = 7)
	int RMP_encoder;			// Right Middle Encoder Value (1 rev = 7)
	float bas_length;			// Length of base actuator in cm
	float elb_length;			// length of elbow actuator in cm
	float for_length;			// length of forearm actuator in cm
	float base_theta;			// Base angle
	float LFS_theta;			// Left Front swerve angle
	float RFS_theta;			// Right Front swerve angle
	float LRS_theta;			// Left Rear Swerve angle
	float RRS_theta;			// Right rear swerve angle
	float batt_volt;			// Battery Voltage read from arduino
	int SMP_pos;				  // Position 1,2,3, or 4
	int ELMAG_state;			// Electromagnet on or off
	int ERR;					  // Code in Error Codes!
	int BLS_LS;					  // Base Limit Switch Left State
	int BLS_RS;					  // Base Limit Switch Right State
	int CLS_T;					  // Clamp Limit Switch Top State
	int CLS_B;					  // Clamp Limit Switch Bottom State
	int SCLS;					    // Sample Carousel Limit Switch State
	int HLSS;             // Hand Limit Switch
	int REVERSE;			// Reverse drive motors
	int FTE_LS;       // 5TE Switch
	int CLA;					// Clamp (1,0,-1 = open, close, chill)
	int WRI;					// Wrist (1,0,-1 = left, right, chill)
} current_state;

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
typedef struct target_state {
	float LFP_duty;				// Left Front Power Motor Duty
	float LMP_duty;				// Left Middle Power Motor Duty
	float LRP_duty;				// Left Rear Power Motor Duty
	float RFP_duty;				// Right Front Power Motor Duty
	float RMP_duty;				// Right Middle Power Motor Duty
	float RRP_duty;				// Right Rear Power Motor Duty
	float bas_length;			// Length of base actuator in cm
	float elb_length;			// length of elbow actuator in cm
	float FORLENgth;			// length of forearm actuator in cm float base_theta;			// Base angle
	float LFS_theta;			// Left Front swerve angle
	float RFS_theta;			// Right Front swerve angle
	float LRS_theta;			// Left Rear Swerve angle
	float RRS_theta;			// Right rear swerve angle
	float SMP_pos;				// Sample Carousel Position
} target_state;*/
