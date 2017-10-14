#include <Dash.h>
#include <Drive.h>
#include <Defs.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <SoftwareSerial.h>

std_msgs::Float32MultiArray value;

// Set up publisher
ros::Publisher pubby("MEGA", &value);
ros::NodeHandle nh;

float theta;
float lr=0;
float ud=0;

void manualCallback(const std_msgs::Float32MultiArray& msg) { 
  /*lr=msg.data[3];
  duty=msg.data[4];
  theta=atan(ud/lr);
  duty=cos(theta)*ud;
  
  if (duty > MAX_DUTY) duty=MAX_DUTY;
  else if (duty < -MAX_DUTY) duty=-MAX_DUTY;
  value.data=msg.data;
  pubby.publish(&value); 

  drive_motor_duties(duty,duty,duty,duty,duty,duty);
  delay(10);
  */
} //manualCallback

// Set up subscriber
ros::Subscriber<std_msgs::Float32MultiArray> subby("manual_guidance", &manualCallback);

void setup() {
  
  Serial.begin(9600);
  Serial2.begin(9600);
  
  // Rover initialization
  initialize_GPIO();
  systemwide_enable();
  
  // ROS Initialization
  //nh.initNode();
  //nh.advertise(pubby);
  //nh.subscribe(subby);
  
  Serial.println("Setup complete.");
} //setup

void loop() {
  Serial.flush();
  Serial2.flush();
  // Bluetooth  
  if (Serial2.available()) {
    char val=Serial2.read();
    Serial.write(val);
    drive(simple_command_to_direction(val));
  }
  
  nh.spinOnce();
  delay(5);  
  
} // loop
