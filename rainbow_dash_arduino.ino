#include <Dash.h>
#include <Defs.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <SoftwareSerial.h>

#define MAX_DUTY 0.5

std_msgs::Float32MultiArray value;

// Set up publisher
ros::Publisher pubby("MEGA", &value);
ros::NodeHandle nh;

float theta;
float duty=0;
float lr=0;
float ud=0;


float v=0, head=0;
char val;

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

void setv(int value) {
   /* 
  if(value==-1) {
    while (v > 0) {
      v-=0.1;
      drive_motor_duties(v,v,v,v,v,v);
      Serial.println(v);
      delay(100);
    }
    v=0;
    drive_motor_duties(v,v,v,v,v,v);

    
  }*/
  if(value==1) {
    while (v > -MAX_DUTY) {
        v-=0.1;
        drive_motor_duties(v,v,v,v,v,v);
        Serial.println(v);
        delay(100);   
    }
         
    v=-MAX_DUTY;
    drive_motor_duties(v,v,v,v,v,v);
  } else if(value==-1) {
    while (v < -0.1) {
        v+=0.1;
        drive_motor_duties(v,v,v,v,v,v);
        Serial.println(v);
        delay(100);   
    }
         
    v=0;
    drive_motor_duties(v,v,v,v,v,v);
  }
  
} //setv

void executeBluetooth(char val) {
 switch (val) {
   case 'l': // LEFT
     break;
  case 'u': // UP
   
     setv(1);
     break;
   case 'r': // RIGHT
     break;
  case 'd': // DOWN
     setv(-1);
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
} // executeBluetooth

void loop() {
  Serial.flush();
  Serial2.flush();
  // Bluetooth  
  if (Serial2.available()) {
    val=Serial2.read();
    Serial.write(val);
    executeBluetooth(val);
  }
  
  nh.spinOnce();
  delay(5);  
  
} // loop

