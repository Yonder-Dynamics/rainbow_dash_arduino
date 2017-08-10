#include <Dash.h>
#include <Defs.h>

int velocity=0;
char val;

void setup() {
  
  Serial.begin(9600);
  Serial.println(initialize_GPIO());
  Serial.println(systemwide_enable());
  
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
    analogWrite(RMP_PWM, velocity);
    delay(5);  
  } 
  while (targetVelocity >= velocity) {
    velocity+=0.1;
    analogWrite(RMP_PWM, velocity);
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

float value=0.5;
  
void loop() {
  
  // Bluetooth
  if (Serial.available()) {
    val=Serial.read();
    Serial.print(val);
    executeBluetooth(val);
  }
  
  Serial.println("...");
  
  value=0.5;
  
  int pwms[] = {LMP_PWM, LRP_PWM, LFP_PWM, RFP_PWM, RMP_PWM/*, RRP_PWM*/};
  
  int i=0;
  for(i=0; i<6; i++) {
   analogWrite(pwms[i], 150);
   delay(1000);
   
   analogWrite(pwms[i], 0); 
  }


  //drive_motor_duties(val, val, val, val, val, val); 
  //delay(1000);
  //value=0;
  //run_DCM_PUL(RMP,value);
  
  //drive_motor_duties(val, val, val, val, val, val); 
  delay(1000);
  
}

