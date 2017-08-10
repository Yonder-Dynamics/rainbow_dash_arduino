#include "Dash.h"

/* UTILITIES */
float mapfun(float x, float inMIN, float inMAX, float outMIN, float outMAX) {
	float val = (x - inMIN) * (outMAX - outMIN) / (inMAX - inMIN) + outMIN;
	return val;
}

/**
INITIALIZATION
**/
int initialize_GPIO(){
/*
	int i=0;

  // OUTPUTS
  int output[]={
		LFP_DIR, LMP_DIR, LRP_DIR, RFP_DIR, RMP_DIR, RRP_DIR, // WHEEL DIRECTION
		LFS_IN1, LFS_IN2, LRS_IN3, LRS_IN4, RFS_IN1, RFS_IN2, RRS_IN3, RRS_IN4,
		SLEEP1, SLEEP2, SLEEP3, SLEEP4, SLEEP5, SLEEP6, // SLEEP
    ENA_BR, SMP_IN1, SMP_IN2, CLA_IN3, CLA_IN4,
		FOR_DIR, FOR_PUL, ELB_DIR, ELB_PUL, BAS_DIR, BAS_PUL, // ARM
		BR_DIR, BR_PUL, WRI_DIR, WRI_PUL, EMACT, ENA_WRI,
		LFP_PWM, LRP_PWM, LMP_PWM, RFP_PWM, RRP_PWM, RMP_PWM // WHEEL PWM
	};

	int output_c=sizeof(output)/sizeof(output[0]);
	for(i=0; i<output_c; i++) pinMode(output[i], OUTPUT);

  // INPUTS
	int input[]={
		BLSL, BLSR, CLST, CLSB, SLS, FTE,
		LFS_POT, LRS_POT, RFS_POT, RRS_POT,
		FOR_POT, ELB_POT, BAS_POT,
		BATT_VOLT
	};

	int input_c=sizeof(input)/sizeof(input[0]);
	for(i=0; i<input_c; i++) pinMode(input[i], INPUT);

  // SLEEP PINS
	//Serial.println("[SETUP] GPIO Initialized.");
	delay(5);
	*/
	// DIGITAL PINS
	pinMode(LFP_DIR, OUTPUT);
	pinMode(LMP_DIR, OUTPUT);
	pinMode(LRP_DIR, OUTPUT);
	pinMode(RFP_DIR, OUTPUT);
	pinMode(RMP_DIR, OUTPUT);
	pinMode(RRP_DIR, OUTPUT);
	pinMode(LFS_IN1, OUTPUT);
	pinMode(LFS_IN2, OUTPUT);
	pinMode(LRS_IN3, OUTPUT);
	pinMode(LRS_IN4, OUTPUT);
	pinMode(RFS_IN1, OUTPUT);
	pinMode(RFS_IN2, OUTPUT);
	pinMode(RRS_IN3, OUTPUT);
	pinMode(RRS_IN4, OUTPUT);
	pinMode(SLEEP1, OUTPUT);
	pinMode(SLEEP2, OUTPUT);
	pinMode(SLEEP3, OUTPUT);
	pinMode(SLEEP4, OUTPUT);
	pinMode(SLEEP5, OUTPUT);
	pinMode(SLEEP6, OUTPUT);

	pinMode(ENA_BR, OUTPUT);
	pinMode(SMP_IN1, OUTPUT);
	pinMode(SMP_IN2, OUTPUT);
	pinMode(CLA_IN3, OUTPUT);
	pinMode(CLA_IN4, OUTPUT);
	pinMode(FOR_DIR, OUTPUT);
	pinMode(FOR_PUL, OUTPUT);
	pinMode(ELB_DIR, OUTPUT);
	pinMode(ELB_PUL, OUTPUT);
	pinMode(BAS_DIR, OUTPUT);
	pinMode(BAS_PUL, OUTPUT);
	pinMode(BR_DIR, OUTPUT);
	pinMode(BR_PUL, OUTPUT);
	pinMode(WRI_DIR, OUTPUT);
	pinMode(WRI_PUL, OUTPUT);
	pinMode(EMACT, OUTPUT);
	pinMode(ENA_WRI, OUTPUT);

	/// INTERRUPT PINS
	pinMode(BLSL, INPUT);
	pinMode(BLSR, INPUT);
	pinMode(CLST, INPUT);
	pinMode(CLSB, INPUT);
	pinMode(SLS, INPUT);
	pinMode(FTE, INPUT); // 5TE?


	// PWM PINS
	pinMode(LFP_PWM, OUTPUT);
	pinMode(LMP_PWM, OUTPUT);
	pinMode(LRP_PWM, OUTPUT);
	pinMode(RFP_PWM, OUTPUT);
	pinMode(RMP_PWM, OUTPUT);
	pinMode(RRP_PWM, OUTPUT);

	// ANALOG PINS
	pinMode(LFS_POT, INPUT);
	pinMode(LRS_POT, INPUT);
	pinMode(RFS_POT, INPUT);
	pinMode(RRS_POT, INPUT);
	pinMode(FOR_POT, INPUT);
	pinMode(ELB_POT, INPUT);
	pinMode(BAS_POT, INPUT);
	//pinMode(BR_POT, INPUT);
	pinMode(BATT_VOLT, INPUT);

  delay(5);
  return 1;
}

int sleep_pins(int i) {
	digitalWrite(SLEEP1, i);
	digitalWrite(SLEEP2, i);
	digitalWrite(SLEEP3, i);
	digitalWrite(SLEEP4, i);
	digitalWrite(SLEEP5, i);
	digitalWrite(SLEEP6, i);
}

/**
SYSTEMWIDE
**/
int systemwide_disable() {
	sleep_pins(LOW);

	delay(5);
	drive_disable();
	disable_arm();
	disable_steppers();
	cstate.rover_state=DISARMED;
}

int systemwide_enable() {
	sleep_pins(HIGH);
	drive_enable();
	delay(5);

  //Serial.println("[SETUP] Systemwide enabled.")
  //cstate.rover_state=drive_enable() && enable_arm() && enable_steppers();
	cstate.rover_state=1;
	return cstate.rover_state;
}

int systemwide_reset() {
	drive_reset();
	arm_reset();
	/*
	arm_default_position();
	drive_halt();
	drive_allwheels_dir(1);
	rotate_SWR_angles(0,0,0,0);*/
}

/**
DRIVE
**/

int drive_reset() {
	//zero motors
	//all front
}

int drive_disable() {
	//printf("Disabling drive...\n");
	drive_halt();
	drive_allwheels_dir(1);
	cstate.drive_state = DISARMED;
	return 1;
}

int drive_enable(){
	//printf("Initializing the Drive System!\n");
	cstate.drive_state = ARMED;
	//drive_halt();
	//drive_allwheels_dir(1);
	//cstate.LMP_encoder = 0;
	//cstate.RMP_encoder = 0;
	return 1;
}

void wheel_dir(Motor MOT, int in) {
	MOT.REV=!in;
	digitalWrite(MOT.DIR, in);
	delay(5);
}

void wheel_pwm(Motor MOT, float duty) {
	//if(duty<0) wheel_dir(MOT,0);
	analogWrite(MOT.PULSE, abs(duty)*255);
	delay(5);
}

void drive_halt() {

	Motor drive_motors[] = {LFP, LMP, LRP, RFP, RMP, RRP};
	int drive_motors_c = sizeof(drive_motors)/sizeof(drive_motors[0]);
	int i=0;
	for(i=0; i<drive_motors_c; i++) wheel_pwm(drive_motors[i], 0);
	delay(5);
}

void drive_allwheels_dir(int in) {
	const Motor drive_motors[] = {LFP, LMP, LRP, RFP, RMP, RRP};
	const int drive_motors_c = sizeof(drive_motors)/sizeof(drive_motors[0]);
	int i=0;
	for(i=0; i<drive_motors_c; i++) wheel_dir(drive_motors[i], in);
	//cstate.REVERSE=!in;
	delay(5);
}

void drive_motor_duties(float rf_d, float lf_d, float lm_d, float lr_d, float rr_d, float rm_d) {
	wheel_pwm(LFP,lf_d);
	wheel_pwm(LRP,rf_d);
	wheel_pwm(LMP,lm_d);
	wheel_pwm(RMP,rm_d);
	wheel_pwm(RFP,lr_d);
	wheel_pwm(RRP,rr_d);
	delay(5);
}

/*
void state_update() {
	 cstate.bas_length=read_bas_length();
	 cstate.elb_length=read_elb_length();
	 cstate.for_length=read_for_length();
	 cstate.base_theta=get_base_theta();
	 cstate.LFS_theta=read_SWR(LFS);
	 cstate.RFS_theta=read_SWR(RFS);
	 cstate.LRS_theta=read_SWR(LRS);
	 cstate.RRS_theta=read_SWR(RRS);
}
*/

int run_DCM_PUL(Motor MOT, int STATE){
	if (cstate.arm_state==ARMED && MOT.TYPE == ARM) {

		int duty;
		uint8_t DIR = LOW;

		switch (STATE) {
			case 1:
				duty = HIGH;
				DIR = LOW;
				break;
			case 0:
				duty = LOW;
				DIR = LOW;
				break;
			case -1:
				duty = HIGH;
				DIR = HIGH;
				break;
			default:
				printf("State is 1, 0, -1 ONLY!!\n");
				return -1;
		}

		digitalWrite(MOT.DIR, DIR);
		delay(5);
		digitalWrite(MOT.PULSE, duty);
		delay(5);
		return 0;
	} else if (cstate.arm_state==ARMED &&
		(MOT.TYPE == SWERVE || MOT.TYPE == CLAMP || MOT.TYPE == SAMPLE)) {

		int A, B;

		switch (STATE) {
			case 1:
				A = HIGH;
				B = LOW;
				break;
			case 0:
				A = LOW;
				B = LOW;
				break;
			case -1:
				A = LOW;
				B = HIGH;
				break;
			default:
				printf("State is 1, 0, -1 ONLY!!\n");
				return -1;
		}

		digitalWrite(MOT.INA, A);
		delay(5);
		digitalWrite(MOT.INB, B);
		delay(5);
		return 0;
	} else {
		printf("Wrong motor nub or not armed, try again!\n");
		return -1;
	}
}

int run_DCM_PWM(Motor MOT, float duty){

// check that the duty cycle is within +-1
//	if(!(cstate.drive_state==ARMED && MOT.TYPE == DRIVE)) {
//		printf("Wrong Motor Type or Not ARMED! -___-\n");
//		return -1;
//	}
	uint8_t DIR = LOW;
	int ss;
	char ch;

  duty = (duty>1.0) ? 1 : duty;
	duty = (duty<-1.0) ? -1 : duty;
	DIR = (duty>=0) ? HIGH : LOW;
	if(DIR==LOW) duty=-duty;

	switch (MOT.PULSE) {
		case 22:
			ss = 2;
			ch = 'A';
			break;
		case 23:
			ss = 2;
			ch = 'B';
			break;
		case 50:
			ss = 1;
			ch = 'A';
			break;
		case 51:
			ss = 1;
			ch = 'B';
			break;
		case 2:
			ss = 0;
			ch = 'A';
			break;
		case 3:
			ss = 0;
			ch = 'B';
			break;
		default:
			printf("Motor Pulse Pin is not correct :/ \n");
			return -1;
	}

	digitalWrite(MOT.DIR, DIR);
	delay(5);

	return 0;
}

void print_state_message() {

  char msg[1000];

	/*msg[0]=cstate.rover_state;
	msg[1]=cstate.arm_state;
	msg[2]=cstate.drive_state;
	msg[3]=cstate.stepper_ena;
	msg[4]=cstate.LFP_duty;
	msg[5]=cstate.LMP_duty;
	msg[6]=cstate.LRP_duty;
	msg[7]=cstate.RFP_duty;
	msg[8]=cstate.RMP_duty;
	msg[9]=cstate.RRP_duty;
	msg[10]=cstate.LMP_encoder;
	msg[11]=cstate.RMP_encoder;
	msg[12]=cstate.bas_length;
	msg[13]=cstate.elb_length;
	msg[14]=cstate.for_length;
	msg[15]=cstate.base_theta;
	msg[16]=cstate.LFS_theta;
	msg[17]=cstate.RFS_theta;
	msg[18]=cstate.LRS_theta;
	msg[19]=cstate.RRS_theta;
	msg[20]=cstate.batt_volt;
	msg[21]=cstate.SMP_pos;
	msg[22]=cstate.ELMAG_state;
	msg[23]=cstate.ERR;
	msg[24]=cstate.BLS_LS;
	msg[25]=cstate.BLS_RS;
	msg[26]=cstate.CLS_T;
	msg[27]=cstate.CLS_B;
	msg[28]=cstate.SCLS;
	msg[29]=cstate.HLSS;
	msg[30]=cstate.REVERSE;
	msg[31]=cstate.FTE_LS;
	msg[32]=cstate.CLA;
	msg[33]=cstate.WRI;

*/
  sprintf(msg, "%f,",      cstate.rover_state);
	sprintf(msg, "%f,",       cstate.arm_state);
	sprintf(msg, "%f,",       cstate.drive_state);
	sprintf(msg, "%f,",       cstate.stepper_ena );
	sprintf(msg, "%f,",       cstate.LFP_duty);
	sprintf(msg, "%f,",       cstate.LMP_duty);
	sprintf(msg, "%f,",       cstate.LRP_duty);
	sprintf(msg, "%f,",       cstate.RFP_duty);
	sprintf(msg, "%f,",       cstate.RMP_duty);
	sprintf(msg, "%f,",       cstate.RRP_duty);
	sprintf(msg, "%f,",       cstate.LMP_encoder );
	sprintf(msg, "%f,",        cstate.RMP_encoder );
	sprintf(msg, "%f,",       cstate.bas_length);
	sprintf(msg, "%f,",       cstate.elb_length);
	sprintf(msg, "%f,",       cstate.for_length);
	sprintf(msg, "%f,",       cstate.base_theta);
	sprintf(msg, "%f,",       cstate.LFS_theta);
	sprintf(msg, "%f,",       cstate.RFS_theta);
	sprintf(msg, "%f,",       cstate.LRS_theta);
	sprintf(msg, "%f,",       cstate.RRS_theta);
	sprintf(msg, "%f,",       cstate.batt_volt);
	sprintf(msg, "%f,",       cstate.SMP_pos);
	sprintf(msg, "%f,",       cstate.ELMAG_state);
	sprintf(msg, "%f,",       cstate.ERR);
	sprintf(msg, "%f,",       cstate.BLS_LS);
	sprintf(msg, "%f,",       cstate.BLS_RS);
	sprintf(msg, "%f,",       cstate.CLS_T);
	sprintf(msg, "%f,",       cstate.CLS_B);
	sprintf(msg, "%f,",       cstate.SCLS);
	sprintf(msg, "%f,",       cstate.HLSS);
	sprintf(msg, "%f,",       cstate.REVERSE);
	sprintf(msg, "%f,",       cstate.FTE_LS);
	sprintf(msg, "%f,",       cstate.CLA);
	sprintf(msg, "%f\n",       cstate.WRI);

}
