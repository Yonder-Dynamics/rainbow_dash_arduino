#include <Dash.h>
#include <Defs.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <SoftwareSerial.h>
#define COEF 0.5
#define MAX_DUTY 0.5

std_msgs::Float32MultiArray value;

// Set up publisher
ros::Publisher pubby("MEGA", &value);
ros::NodeHandle nh;

float theta;
unsigned changeFlag;

/* Speed */
float duty=0;

float lr=0;
float ud=0;

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
  delay(10);
} //manualCallback

// Set up subscriber
ros::Subscriber<std_msgs::Float32MultiArray> subby("manual_guidance", &manualCallback);

void setup() {
  
  Serial.begin(57600);
  Serial2.begin(9600);
  Serial.println("Setup complete.");
  
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
     train_left=max(train_left-0.5, 0.5);
     train_right=min(train_right+0.5, 1);
     break;
  case 'u': // UP
     duty=min(MAX_DUTY, duty+MAX_DUTY); 
     train_left=1;
     train_right=1;
     break;
   case 'r': // RIGHT
     train_right=max(train_right-0.5, 0.5);
     train_left =min(train_left+0.5, 1);
     break;
  case 'd': // DOWN
     duty=max(-MAX_DUTY, duty-MAX_DUTY);
     train_left=1;
     train_right=1;
     break;
   case '5':
   
     break;
  case '6':
   
     break;  
   case 's':
   
     break;
  case 't':
   
     break;  
     
   case 'x':
   
     break;
  case 'c':
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
    drive_motor_duties(duty*train_right,duty*train_left,duty*train_left,duty*train_left,duty*train_right,duty*train_right);
    changeFlag--;   
    Serial.println(); 
    Serial.print("       Duty: "); Serial.println(duty);
    Serial.print(" Left Train: "); Serial.println(train_left);
    Serial.print("Right Train: "); Serial.println(train_right);
  }
  
  nh.spinOnce();
  delay(5);  
  
} // loop

