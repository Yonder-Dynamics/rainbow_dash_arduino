#include <Dash.h>
#include <Defs.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>


float duty_prev=0;
float duty=0;
float lr=0;
float v=0;
char val;
int prevMode=0;
int mode=0; // Bluetooth
std_msgs::Float32MultiArray value;
ros::Publisher pubby("MEGA", &value);

void manualCallback(const std_msgs::Float32MultiArray& msg) { 
  
  lr=msg.data[3];
  duty=msg.data[4];
  value.data=msg.data;
  pubby.publish(&value); 

  drive_motor_duties(duty,duty,duty,duty,duty,duty);
  delay(10);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> subby("manual_guidance", &manualCallback);


void setup() {
  
  //Serial.begin(57600);
  //Serial2.begin(57600);
  //Serial.println(
  initialize_GPIO();//);
  //Serial.println(
  systemwide_enable();//);
  
  nh.initNode();
  nh.advertise(pubby);
  nh.subscribe(subby);
}

void setv(int value) {
    
  if(value==-1) {
    while (v > 0) {
      v-=0.1;
      drive_motor_duties(v,v,v,v,v,v);
      Serial.println(v);
      delay(100);
    }
    
    v=0;
    
  } else { // if 1
    while (v < 1) {
      v+=0.1;
      drive_motor_duties(v,v,v,v,v,v);
      Serial.println(v);
      delay(100);   
    }
    
    v=1;
    
  }
}

void executeBluetooth(char val) {
 switch (val) {
   case '1': // LEFT
   
     break;
  case '2': // UP
     setv(1);
     break;
   case '3': // RIGHT
   
     break;
  case '4': // DOWN
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
 
 
  
}

void loop() {
  // Bluetooth
  /*
  if (Serial2.available()) {
    val=Serial2.read();
    Serial.print(val);
    executeBluetooth(val);
  }*/
  
  if (duty!=duty_prev) {
    //Serial.println("hey");
    duty_prev=duty;
  }
  nh.spinOnce();
  delay(10);  
  
}

