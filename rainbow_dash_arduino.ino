#include <Dash.h>
#include <Defs.h>

float v=0;
char val;
int prevMode=0;
int mode=0; // Bluetooth

void setup() {
  
  Serial.begin(9600);
  Serial.println(initialize_GPIO());
  Serial.println(systemwide_enable());
  
}

void setv(int value) {
    
  if(value==-1) {
    while (v > 0) {
      v-=0.1;
      drive_motor_duties(v,v,v,v,v,v);
      //Serial.println(v);
      delay(100);
    }
    
    v=0;
    
  } else { // if 1
    while (v < 1) {
      v+=0.1;
      drive_motor_duties(v,v,v,v,v,v);
      //Serial.println(v);
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
  
  if (Serial.available()) {
    val=Serial.read();
    Serial.print(val);
    executeBluetooth(val);
  }
  
  
  /*
    switch(val) {
      case '6':
        if (mode!=0) {
          prevMode=mode;
          mode=0;
        }
        else mode=prevMode;
        break;
    }
    if(mode==0) executeBluetooth(val);
  }*/
}

