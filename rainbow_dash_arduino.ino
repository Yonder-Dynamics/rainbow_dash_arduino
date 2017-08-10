#include <Dash.h>
#include <Defs.h>

int velocity=0;
char val;

void setup() {
  
  Serial.begin(9600);
  initialize_GPIO();
  systemwide_enable();
  
}

void setVelocity(int value) {
  
  float targetVelocity=0;
  
  if(value==-1) {
    if(velocity==1) targetVelocity=0; //if velocity == 1
    else targetVelocity=-1;        //if velocity == 0 or -1
  } else { // if 1
    if(velocity==-1) targetVelocity=0; // if velocity == -1
    else targetVelocity=1;             // if velocity == 0 or 1
  }
  
  while (targetVelocity < velocity) {
    velocity-=0.1;
    wheel_pwm(LFP, velocity);
    delay(5);  
  } 
  while (targetVelocity >= velocity) {
    velocity+=0.1;
    wheel_pwm(LFP, velocity);
    delay(5);
  }
}

void executeBluetooth(char val) {
 switch (val) {
   case '1': // LEFT
   
     break;
  case '2': // UP
     setVelocity(1);
     break;
   case '3': // RIGHT
   
     break;
  case '4': // DOWN
     setVelocity(-1);
     break;
   case '5':
   
     break;
  case '6':
   
     break;  
   case '7':
   
     break;
  case '8':
   
     break;  
     
   case '9':
   
     break;
  case '10':
   
     break;
 } 
 
 
  
}

void loop() {
  
  // Bluetooth
  if (Serial.available()) {
    val=Serial.read();
    executeBluetooth(val);
  }
  
}
