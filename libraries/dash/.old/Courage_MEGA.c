#include "Courage.h"

float mapfun(float x, float inMIN, float inMAX, float outMIN, float outMAX) {
	float val = (x - inMIN) * (outMAX - outMIN) / (inMAX - inMIN) + outMIN;
	return val;
}

void initialize_GPIO(){

  //srv.attach(SERVO);

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

	sleep_pins(HIGH);

	delay(5);

}

int disable_steppers(){
	digitalWrite(BR.ENA, LOW);	// Disable Driver
	delay(5);
	digitalWrite(BR.PULSE,LOW); 	// Set Step Pin to Low
	delay(5);

	digitalWrite(WRI.ENA, LOW);	// Disable Driver
	delay(5);
	digitalWrite(WRI.PULSE,LOW); 	// Set Step Pin to Low
	delay(5);

	cstate.stepper_ena=DISARMED;
	return 1;
}

int enable_steppers(){
	digitalWrite(BR.ENA, HIGH);	// Enable Driver
	delay(5);
	digitalWrite(BR.PULSE,HIGH); 	// Set Step Pin to Low
	delay(5);
	digitalWrite(WRI.ENA, HIGH);	// Enable Driver
	delay(5);
	digitalWrite(WRI.PULSE,HIGH); 	// Set Step Pin to Low
	delay(5);
  //cstate.base_theta=get_base_theta(BR_POT);
  cstate.stepper_ena=ARMED;
	return 1;
}

int disable_drive() {
	printf("Disabling drive...\n");
	zero_motors();
	all_wheels_dir(1);
	cstate.drive_state = DISARMED;
	return 1;
}

int enable_drive(){
	printf("Initializing the Drive System!\n");
	cstate.drive_state = ARMED;
	zero_motors();
	all_wheels_dir(1);
	//cstate.LMP_encoder = 0;
	//cstate.RMP_encoder = 0;
	return 1;
 }

int disable_arm() {
	cstate.arm_state = DISARMED;
  return 1;
}

int enable_arm() {
	cstate.arm_state = ARMED;
	return 1;
}

void sleep_pins(int i) {
	digitalWrite(SLEEP1,i);
	digitalWrite(SLEEP2,i);
	digitalWrite(SLEEP3,i);
	digitalWrite(SLEEP4,i);
	digitalWrite(SLEEP5,i);
	digitalWrite(SLEEP6,i);
}


int disarm_rover() {
	sleep_pins(LOW);

	delay(5);
	disable_drive();
	disable_arm();
	disable_steppers();
	cstate.rover_state=DISARMED;
}

int arm_rover() {

	delay(5);
  cstate.rover_state=enable_drive() && enable_arm() && enable_steppers();
	return cstate.rover_state;
}

void rover_reset() {
	arm_default_position();
	zero_motors();
	all_wheels_dir(1);
	rotate_SWR_angles(0,0,0,0);
}

void base_rotate_to_limit(int dir) {
	float step_d=6000000;
	int step_delay=step_d/STREV/SPDBR;

	if(dir==CW) {
		digitalWrite(BR.DIR, CW);
		delay(5);

		while(cstate.BLS_RS!=0) {
			digitalWrite(BR.PULSE,HIGH); //STEP HIGH
			delayMicroseconds(step_delay);
			digitalWrite(BR.PULSE,LOW); //STEP LOW Complete Step
			delayMicroseconds(step_delay);
		}
	} else if (dir==CCW) {
		digitalWrite(BR.DIR, CCW);
		while(cstate.BLS_LS!=0) {
			digitalWrite(BR.PULSE,HIGH); //STEP HIGH
			delayMicroseconds(step_delay);
			digitalWrite(BR.PULSE,LOW); //STEP LOW Complete Step
			delayMicroseconds(step_delay);
		}
	}
}

void arm_default_position() {
	safe_extend();
	base_rotate_to_limit(CCW);
}

void flagpole() {
	move_arm(0, 0, ELBLEN, FORLEN);
}
// Base
float read_bas_length(){
	int raw;
	float length;
	raw = analogRead(BAS_POT);
	length = mapfun(raw, BASMIN, BASMAX, 0, BASLEN)+ACTUATOR_BAS_LENGTH;
	return length;
}

// Elbow
float read_elb_length(){
	int raw;
	float length;
	raw = analogRead(ELB_POT);
	length = mapfun(raw, ELBMIN, ELBMAX, 0, ELBLEN)+ACTUATOR_ELB_LENGTH;
	return length;
}

// Forearm
float read_for_length(){
	int raw;
	float length;
	raw = analogRead(FOR_POT);
	length = mapfun(raw, FORMIN, FORMAX, 0, FORLEN)+ACTUATOR_FOR_LENGTH;
	return length;
}

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

void shake() {

}

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

float read_SWR(Motor MOT){
//	if (MOT.TYPE == SWERVE){
	int raw;
	float theta=0.;
	switch (MOT.INA){
		case LFS_IN1:
			raw = analogRead(lLFP);
			theta = mapfun(raw, LFS_MIN, LFS_MAX, -MAX_RAD, MAX_RAD);
			break;
		case RFS_IN1:
			raw = analogRead(lRFP);
			theta = mapfun(raw, RFS_MIN, RFS_MAX, -MAX_RAD, MAX_RAD);
			break;
		case LRS_IN3:
			raw = analogRead(lLRP);
			theta = mapfun(raw, LRS_MIN, LRS_MAX, -MAX_RAD, MAX_RAD);
			break;
		case RRS_IN3:
			raw = analogRead(lRRP);
			theta = mapfun(raw, RRS_MIN, RRS_MAX, -MAX_RAD, MAX_RAD);
			break;
		}
		return theta;

	//else{
//		printf("Wrong motor type, send Euan nudes. Fix IMMEDIATELY\n");
//		return -1;
	//}
}

void swerve_direction_test() {
	run_DCM_PUL(LFS,1);
	run_DCM_PUL(RFS,1);
	run_DCM_PUL(LRS,1);
	run_DCM_PUL(RRS,1);
}

int engage_reverse() {
	cstate.REVERSE=1;
}

int disengage_reverse() {
	cstate.REVERSE=0;
}

int rotate_SWR_angles(float theta_rf, float theta_lf, float theta_lr, float theta_rr) {
	rotate_SWR(LFS, theta_lf);
	rotate_SWR(RFS, theta_rf);
	rotate_SWR(LRS, theta_lr);
	rotate_SWR(RRS, theta_rr);

}

int rotate_SWR(Motor SWR, float theta_swr) {
	//if (//cstate.drive_state!=ARMED) {
	//	printf("Rover is not ARMED! Could not drive.\n");
  //  return -1;
	//}
  // Map target angle to desired POT reading
/*
	float lfspot=mapfun(tstate.LFS_theta, LFSMIN, LFSMAX, -MAX_RAD, MAX_RAD);
	float rfspot=mapfun(tstate.RFS_theta, RFSMIN, RFSMAX, -MAX_RAD, MAX_RAD);
	float lrspot=mapfun(tstate.LRS_theta, LRSMIN, LRSMAX, -MAX_RAD, MAX_RAD);
	float rrspot=mapfun(tstate.RRS_theta, RRSMIN, RRSMAX, -MAX_RAD, MAX_RAD);
*/

  float pot_reading, in_rad;
  int pot_pin;
  float MAX, MIN;
	if (SWR.LAB==lLFS) {
		pot_pin=LFS_POT;
		MAX=LFS_MAX;
		MIN=LFS_MIN;
	} else if (SWR.LAB==lRFS) {
		pot_pin=RFS_POT;
		MAX=RFS_MAX;
		MIN=RFS_MIN;
	} else if (SWR.LAB==lLRS) {
		pot_pin=LRS_POT;
		MAX=LRS_MAX;
		MIN=LRS_MIN;
	} else if (SWR.LAB==lRRS) {
		pot_pin=RRS_POT;
		MAX=RRS_MAX;
		MIN=RRS_MIN;
	}
  //state_update();
	while(true){
		float pot_reading=analogRead(pot_pin);
		int rad=mapfun(pot_reading, MIN, MAX, -MAX_RAD, MAX_RAD);
		if (pot_reading>=MAX||pot_reading<=MIN) {
			printf("Left front swerve limit reached: %d degrees.");
		} else if (abs(rad-theta_swr)>TOL) {
			if(theta_swr>rad) run_DCM_PUL(SWR, 1);
			else run_DCM_PUL(SWR, -1);
		} else {
			run_DCM_PUL(SWR,0);
		}
/*
		// Right Front (RFS)
		float rfss=analogRead(RFS_POT);
		if (rfss>=RFSMAX||rfss<=RFSMIN) {
			//printf("Right front swerve limit reached: %d degrees.");
		} else if (abs(rfspot - rfss)>TOL) {
			if(rfspot>rfss) run_DCM_PUL(RFS, 1);
			else run_DCM_PUL(RFS, -1);
		} else {
			run_DCM_PUL(RFS,0);
		}

		// Left Rear (LRS)
		float lrss=analogRead(LRS_POT);
		if (lrss>=LRSMAX||lrss<=LRSMIN) {
			//printf("Left rear (LR) swerve limit reached: %d degrees.");
		} else if (abs(lrspot - lrss)>TOL) {
			if(lrspot>rfss) run_DCM_PUL(LRS, 1);
			else run_DCM_PUL(LRS, -1);
		} else {
			run_DCM_PUL(LRS,0);
		}

		// RRS
		float rrss=analogRead(RRS_POT);
		if (rrss>=RRSMAX||rrss<=RRSMIN) {
			//printf("Right rear swerve limit reached: %d degrees.");
		} else if (abs(rrspot - rrss)>TOL) {
			if(rrspot>rrss) run_DCM_PUL(RRS, 1);
			else run_DCM_PUL(RRS, -1);
		} else {
			run_DCM_PUL(RRS,0);
		}
*/
		// Update
	}

	return 0;
}

int get_encoders() {
	/*
  Wire.requestFrom(8,6);
  byte buf[8];
  int c_byte=0;
  while( Wire.available() ) {
		Wire.readBytes(buf, 8);
	  if( c_byte == 8 ) {
			break;
		}
 }
 */
 //cstate.LMP_encoder=;
 //cstate.RMP_encoder=;
 return -1;
}

void toggle_emag() {
	cstate.ELMAG_state=!cstate.ELMAG_state;
}

void grab_FTE() {
	//FTE_get();
}

int update_drive_vals(){
	//float res[2]=get_encoders();

	//cstate.LMP_encoder = rc_get_encoder_pos(1);
	//cstate.RMP_encoder = rc_get_encoder_pos(2);
	// //cstate.LFS_theta = read_SWR(lLFS);
	// //cstate.RFS_theta = read_SWR(lRFS);
	// //cstate.LRS_theta = read_SWR(lLRS);
	// //cstate.RRS_theta = read_SWR_theta(lRRS);
	return 0;
}

void clamp_action(int state, int top_limit, int bot_limit) {

	if(state == 1) { // Open
		// Check if top limit switch is hit
		if(top_limit) {
			digitalWrite(CLA.ENA, 0);
			delay(5);
			digitalWrite(CLA.INA, LOW);
			delay(5);
			digitalWrite(CLA.INB, LOW);
			delay(5);

			return;
		}

		digitalWrite(CLA.ENA,  1);
		delay(5);
		digitalWrite(CLA.INA, HIGH);
		delay(5);
		digitalWrite(CLA.INB, LOW);
		delay(5);

	}
	else if(state == -1) { // Close
		// Check if bot limit switch is hit
		if(bot_limit) {
			digitalWrite(CLA.ENA,  0);
			delay(5);
			digitalWrite(CLA.INA, LOW);
			delay(5);
			digitalWrite(CLA.INB, LOW);
			delay(5);
			return;
		}
		digitalWrite(CLA.ENA,  1);
		delay(5);
		digitalWrite(CLA.INA, LOW);
		delay(5);
		digitalWrite(CLA.INB, HIGH);
		delay(5);
	}
	else { // Chillin
		digitalWrite(CLA.ENA,  0);
		delay(5);
		digitalWrite(CLA.INA, LOW);
		delay(5);
		digitalWrite(CLA.INB, LOW);
		delay(5);
	}
}


/***
STEPPERS
***/


void spin_wrist(int state) {
	if(state == 1) { // Turn left
		if(wristCount == 20) {
			//digitalWrite(WRI.ENA, HIGH);
			return;
		}
		digitalWrite(WRI.ENA, LOW);
		delay(5);

		//wristStepper.step(STPWRI);
		stepper_step(WRI, STPWRI);
		wristCount++;
	}
	else if(state == -1) { // Turn right
		if(wristCount == -20) {
			//digitalWrite(WRI.ENA, HIGH);
			return;
		}
		digitalWrite(WRI.ENA, LOW);
		delay(5);

		//wristStepper.step(-1*STPWRI);
		stepper_step(WRI, STPWRI);
		wristCount--;
	}
	else { // Chillin
		digitalWrite(WRI.ENA, HIGH);
		delay(5);

	}
}


int spin_SM(Motor MOT, int Direction) {
	if (MOT.TYPE != STEP) {
		printf("Wrong Motor Type!\n");
		return -1;
	}

	digitalWrite(MOT.DIR, Direction);
	delay(5);

	//printf("%d\n", Direction);
	return 0;
}

int stepper_step(Motor MOT, int steps) {

	//if(!(cstate.arm_state == ARMED && MOT.TYPE == STEP )) {
	//	printf("Rover not ARMED or Wrong Motor Type! Check yo shit out.\n");
	//	return -1;
	//}

  int top=6000;
	int bottom=1000/STREV/MOT.SPD;
	int steps_left, step_delay = top*bottom;

	// Verify Step Number
	// If steps neeeded, set direction based
	// on the sign of 'steps'
	if(steps==0) {
		printf("No Steps to take");
		return 0;
	} else {
		if (steps>0) spin_SM(MOT, CW);
		else spin_SM(MOT, CCW);
	}

	// Steps left to spin
	steps_left = abs(steps);

	//printf("Running Motor Now!\n");
	while(cstate.BLS_LS == 0 && cstate.BLS_RS ==0 && steps_left > 0){
		//printf("%d\n", mmap_gpio_read(SM1_LIN_STOP_PIN));
		digitalWrite(MOT.PULSE,HIGH); //STEP HIGH
		delayMicroseconds(step_delay);
		digitalWrite(MOT.PULSE,LOW); //STEP LOW Complete Step
		delayMicroseconds(step_delay);
		//printf("%d\n", steps_left);
		steps_left--;
	}

	digitalWrite(MOT.PULSE,LOW);
	delay(5);

	if (cstate.BLS_LS || cstate.BLS_RS) printf("Base stepper limit reached.");

	return 0;
}

float get_base_theta() {
	//float res=analogRead(BR_POT);
	//mapfun(res,0,1023,,90);
	return cstate.base_theta;
}

int theta2steps(float theta) {
	float result=theta-cstate.base_theta;
	if (abs(result) > RAD_TOL) {
		float d = TWO_PI/STREV/BRGR;
		if (theta>cstate.base_theta)
			return round(result/d);
		else
			return round((-1*result)/d);
	}
	return 0;
}


void rotate_carousel(int dir) {
	float step_d=6000000;
	int step_delay=step_d/STREV/SPDBR;

	if(dir==CW) {
 	 digitalWrite(SMP.DIR, CW);	delay(5);

 	 while(cstate.BLS_RS!=0) {
 		 digitalWrite(BR.PULSE,HIGH); //STEP HIGH
 		 delayMicroseconds(step_delay);
 		 digitalWrite(BR.PULSE,LOW); //STEP LOW Complete Step
 		 delayMicroseconds(step_delay);
 	 } while(cstate.BLS_RS!=1) {
 		 digitalWrite(BR.PULSE,HIGH); //STEP HIGH
 		 delayMicroseconds(step_delay);
 		 digitalWrite(BR.PULSE,LOW); //STEP LOW Complete Step
 		 delayMicroseconds(step_delay);
 	 }
 } else if (dir==CCW) {
 	 digitalWrite(SMP.DIR, CCW);	delay(5);
 	 while(cstate.BLS_LS!=0) {
 		 digitalWrite(BR.PULSE,HIGH); //STEP HIGH
 		 delayMicroseconds(step_delay);
 		 digitalWrite(BR.PULSE,LOW); //STEP LOW Complete Step
 		 delayMicroseconds(step_delay);
 	 }
 	 while(cstate.BLS_LS!=1) {
 		 digitalWrite(BR.PULSE,HIGH); //STEP HIGH
 		 delayMicroseconds(step_delay);
 		 digitalWrite(BR.PULSE,LOW); //STEP LOW Complete Step
 		 delayMicroseconds(step_delay);
 	 }
 }


}

/*******************************************************************/

void unfurl(){
	move_arm(0.0, 0.0, 0.0, 0.0);
	move_arm(0.0, 0.0, ELBLEN, 0.0);
	move_arm(0.0, 0.0, ELBLEN, FORLEN);
	move_arm(0.0, BASLEN, ELBLEN, FORLEN);
	move_arm(0.0, 0.0, ELBLEN, FORLEN);
	move_arm(0.0, 0.0, ELBLEN, 0.0);
	move_arm(0.0, 0.0, 0.0, 0.0);
}


int move_arm(float theta_b, float l_bas, float l_elb, float l_for){
/*
	if (//cstate.arm_state!=ARMED) {
		printf("Rover is not ARMED! Could not move arm.\n");
    return -1;
	}
*/
	int complete=0, comp_b = 0, comp_e = 0, comp_f = 0;

	//update_arm_vals();
	//update_drive_vals();

	// Adjust arm segments until final positions reached
	// The sign of the difference between the current state
	// and the desired state is the direction of
	// actuator movement.
	while(!complete){

		// Base
		if (comp_b) {}
    else if (l_elb>=ELBMAX||l_elb<=ELBMIN) {
			printf("Base extension/retraction limit reached.");
			comp_b=1;
		} else if (abs(l_bas - read_bas_length())>TOL) {
			if(l_bas>read_bas_length()) run_DCM_PUL(BAS, 1);
			else run_DCM_PUL(BAS, -1);
		} else {
			run_DCM_PUL(BAS,0);
			comp_b = 1;
		}

		// Elbow
		if (comp_e) {}
    else if (l_elb>=ELBMAX||l_elb<=ELBMIN) {
			printf("Elbow extension/retraction limit reached.");
			comp_e=1;
		} else if (abs(l_elb - read_elb_length())>TOL){
			if(l_elb>read_elb_length()) run_DCM_PUL(ELB, 1);
			else run_DCM_PUL(ELB, -1);
		} else {
			run_DCM_PUL(ELB,0);
			comp_e = 1;
		}

		// Forearm
		if( comp_f ) {}
		else if (l_for>=FORMAX || l_for<=FORMIN) {
			printf("Forearm extension/retraction limit reached.");
      comp_f=1;
		} else if (abs(l_for - read_for_length())>TOL) {
			if (l_for>read_for_length()) run_DCM_PUL(FOR, 1);
			else run_DCM_PUL(FOR, -1);
		} else {
			run_DCM_PUL(FOR,0);
			comp_f = 1;
		}

		// Update
		//delayMicroseconds(UPDINT);
		delay(100);
		if (comp_b && comp_e && comp_f) complete = 1;
	}

	run_DCM_PUL(BAS, 0);
	run_DCM_PUL(FOR, 0);
	run_DCM_PUL(ELB, 0);

	// TODO UNCOMMENT
	//rotate_base(theta_b);

	//cstate.base_theta = theta_b;
	return 0;
}

void rotate_base(float theta) {
	//implement POT

  // Rotate Base - if limit switches active,
	// only allow movement away from switch
  int steps_ = theta2steps(theta);
	if(cstate.BLS_LS==1) {
		if(steps_>0) stepper_step(BR, steps_);
	} else if(cstate.BLS_RS==1) {
		if(steps_<0) stepper_step(BR, steps_);
	} else {
		stepper_step(BR, steps_);
	}
}

void zero_motors() {
	int i=0;
	wheel_pwm(LFP,0);
	wheel_pwm(LMP,0);
	wheel_pwm(LRP,0);
	wheel_pwm(RFP,0);
	wheel_pwm(RMP,0);
	wheel_pwm(RRP,0);
}

void wheel_dir(Motor MOT, int in) {
	MOT.REV=!in;
  digitalWrite(MOT.DIR, in);
	delay(5);
}

void all_wheels_dir(int in) {
	wheel_dir(LFP, in);
	wheel_dir(RFP, in);
	wheel_dir(LMP, in);
	wheel_dir(RMP, in);
	wheel_dir(LRP, in);
	wheel_dir(RRP, in);
	cstate.REVERSE=!in;
}

void wheel_pwm(Motor MOT, float duty) {
	if(duty<0) wheel_dir(MOT,0);
	analogWrite(MOT.PULSE, abs(duty)*255);
	delay(5);
}

void set_motor_duties(float rf_d, float lf_d, float lm_d, float lr_d, float rr_d, float rm_d) {
  wheel_pwm(LFP,lf_d);
	wheel_pwm(LRP,rf_d);
  wheel_pwm(LMP,lm_d);
	wheel_pwm(RMP,rm_d);
	wheel_pwm(LRP,lr_d);
	wheel_pwm(RRP,rr_d);
	delay(100);
}

void all_motors_on() {

	for(int i=0; i < 256; i++) {
		wheel_pwm(LFP,i);
		wheel_pwm(LRP,i);
		wheel_pwm(LMP,i);
		wheel_pwm(RMP,i);
		wheel_pwm(RFP,i);
		wheel_pwm(RRP,i);
		delay(100);
	}


		for(int i=255; i > -1; i--) {
			wheel_pwm(LFP,i);
			wheel_pwm(LRP,i);
			wheel_pwm(LMP,i);
			wheel_pwm(RMP,i);
			wheel_pwm(RFP,i);
			wheel_pwm(RRP,i);
			delay(100);
		}
}

void all_motors_off() {
	wheel_pwm(LFP,0);
	wheel_pwm(LRP,0);
	wheel_pwm(LMP,0);
	wheel_pwm(RMP,0);
	wheel_pwm(LRP,0);
	wheel_pwm(RRP,0);
}

void cycle() {
  if(cstate.REVERSE == 0) {
		wheel_dir(LFP,1);
		wheel_dir(RFP,1);
		wheel_dir(LMP,1);
		wheel_dir(RMP,1);
		wheel_dir(LRP,1);
		wheel_dir(RRP,1);

	} else {
		wheel_dir(LFP,-1);
		wheel_dir(RFP,-1);
		wheel_dir(LMP,-1);
		wheel_dir(RMP,-1);
		wheel_dir(LRP,-1);
		wheel_dir(RRP,-1);

	}


//LM, RF, RR, RM, LF, LR, LM
  delay(1000);

 	digitalWrite(LFP_PWM, HIGH);
	delay(3000);
	digitalWrite(LFP_PWM, LOW);
	delay(5);
	digitalWrite(LMP_PWM, HIGH);
	delay(3000);
 	digitalWrite(LMP_PWM, LOW);
	delay(5);
 	digitalWrite(LRP_PWM, HIGH);
  delay(3000);
  digitalWrite(LRP_PWM, LOW);
	delay(5);
 	digitalWrite(RFP_PWM, HIGH);
	delay(3000);
	digitalWrite(RFP_PWM, LOW);
	delay(5);
	digitalWrite(RMP_PWM, HIGH);
	delay(3000);
 	digitalWrite(RMP_PWM, LOW);
	delay(5);
 	digitalWrite(RRP_PWM, HIGH);
	delay(3000);
	digitalWrite(RRP_PWM, LOW);

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
