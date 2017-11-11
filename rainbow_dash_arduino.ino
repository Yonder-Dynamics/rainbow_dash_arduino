#include <Dash.h>
#include <Defs.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <SoftwareSerial.h>
#include <Stepper.h>
#define COEF 0.5
#define MAX_DUTY 1

std_msgs::Float32MultiArray value;

// Set up publisher
ros::Publisher pubby("MEGA", &value);
ros::NodeHandle nh;

Stepper stepp(200, BR_DIR, BR_PUL);

float train_high=0.75;
float train_low =0.25;

float theta;
unsigned changeFlag;

/* Speed */
float duty=0;
float lr=0;
float ud=0;

/** Part of the arm being controlled. 
0 = Base
1 = Elbow
2 = Forearm
*/
int arm_segment=0;
int arm_duty=0;

int arm_base_duty=0;

/** Coefficients of speed */
float train_left=1;
float train_right=1;

void manualCallback(const std_msgs::Float32MultiArray& msg) { 
  
  lr=msg.data[3];
  duty=msg.data[4];
  /*
  theta=atan(ud/lr);
  duty=cos(theta)*ud;*/
  
  if (duty > MAX_DUTY) duty=MAX_DUTY;
  else if (duty < -MAX_DUTY) duty=-MAX_DUTY;
  value.data=msg.data;
  pubby.publish(&value); 

  drive_motor_duties(duty,duty,duty,duty,duty,duty);
  Serial.println(duty);
  delay(10);
} //manualCallback

// Set up subscriber
ros::Subscriber<std_msgs::Float32MultiArray> subby("manual_guidance", &manualCallback);

void setup() {
  
  Serial.begin(57600);
  Serial.println("Setup complete.");

  stepp.setSpeed(SPDBR);  
  // Rover initialization
  initialize_GPIO();
  systemwide_enable();
  
  // ROS Initialization
  nh.initNode();
  nh.advertise(pubby);
  nh.subscribe(subby);
  
  Serial2.begin(9600);
} //setup

void executeBluetooth(char val) {
 switch (val) {
   case 'l': // LEFT
     train_left=min(train_low, train_high);
     train_right=max(train_low, train_high);
     break;
  case 'u': // UP
     duty=min(MAX_DUTY, duty+MAX_DUTY); 
     train_left=1;
     train_right=1;
     break;
   case 'r': // RIGHT
     train_right=min(train_low, train_high);
     train_left =max(train_low, train_high);
     break;
  case 'd': // DOWN
     duty=max(-MAX_DUTY, duty-MAX_DUTY);
     train_left=1;
     train_right=1;
     break;
   case '5':
     arm_base_duty=max(-1, arm_base_duty-1);
     arm_duty=0;
     stepp.step(-STPBR);
     break;
  case '6':
     arm_base_duty=min(1, arm_base_duty+1);
     stepp.step(STPBR);
     arm_duty=0;
     break;  
   case 's':
     arm_segment=max(0, arm_segment-1);
     arm_duty=0;    
     arm_base_duty=0; 
     break;
  case 'c':
     arm_duty=min(1, arm_duty+1);
     arm_base_duty=0;
     break;  
   case 't':
     arm_duty=max(-1, arm_duty-1);
     arm_base_duty=0;
     break;
  case 'x':
     arm_segment=min(2, arm_segment+1);
     arm_duty=0;
     arm_base_duty=0;
     break;
 }  
 
 changeFlag++;
} // executeBluetooth

void loop() {
  Serial.flush();
  Serial2.flush();
  // Bluetooth  
  if (Serial2.available()) {
    char val=Serial2.read();
    Serial.write(val);
    executeBluetooth(val);
  }
  
  if (changeFlag) {
    arm_motor_duties(0,0,0);
    //arm_base_rotate(arm_base_duty);
    
    
    switch (arm_segment) {
      case 0:
        arm_motor_duties(arm_duty, 0, 0);
        break;
      case 1:
        arm_motor_duties(0, arm_duty, 0);
        break;
      case 2:
        arm_motor_duties(0, 0, arm_duty);
        break;  
    }
    
    
    //arm_base_rotate(arm_base_duty);
    
    drive_motor_duties(duty*train_right,duty*train_left,duty*train_left,duty*train_left,duty*train_right,duty*train_right);
    changeFlag--;   
    Serial.println();
    Serial.println("Drive Train:"); 
    Serial.print("       Duty: "); Serial.println(duty);
    Serial.print(" Left Train: "); Serial.println(train_left);
    Serial.print("Right Train: "); Serial.println(train_right);
    
    Serial.println("Arm:");
    
    Serial.print("   Segment: ");
    Serial.println(arm_segment);
    Serial.print("      Duty: ");
    Serial.println(arm_duty);
    Serial.print(" Base Duty: ");
    Serial.println(arm_base_duty);
  }
  
  nh.spinOnce();
  delay(5);  
  
} // loop

