#include <AFMotor.h>
AF_DCMotor M_FR(3);
AF_DCMotor M_FL(4);
AF_DCMotor M_BR(1);
AF_DCMotor M_BL(2);

float x,z,vr,vl; // for the velocity
String cmd_str;

void setup() 
{
  Serial.begin(115200);
}

void loop() {
 // reception des données provenant de Raspbery
 if (Serial.available())
 {
  cmd_str = Serial.readStringUntil('\n');
  separation();
  right_left_wheels();
  affectation();
  delay(10);
 }
}

 // to split the string into a float
void separation(){      
  
 String x_str;
 String z_str;

 //convert char to string
 char cmd[cmd_str.length() + 1];
 cmd_str.toCharArray( cmd, cmd_str.length() + 1);
 char *c = &cmd[0];

 // recuperation of x_str
  for(int i = 0; i <= cmd_str.length() + 1; i++){
    if (*c != 'a') {
    x_str = x_str + *c;
    c+=1;
    }
    else
      break;
  }

 // recuperation of z_str
  c+=1;
  for(int i = 0; i <= cmd_str.length() + 1; i++){
    if (*c != 'e') {
    z_str = z_str + *c;
    c+=1;
    }
    else
      break;
  }

  x = x_str.toFloat();
  z = z_str.toFloat();
}

void right_left_wheels(){
  double l = 1.0 / 2 ;
  vr = x + z * l;
  vl = x - z * l;

  //conversion en pwm
  vr = vr * 255 / 0.66;
  vl = vl * 255 / 0.66; 
}

// to give the commande to a motor
void affectation(){ 
  
  // move forward_Right_wheels
  if(vr>=0){             
    M_FR.setSpeed(vr);
    M_BR.setSpeed(vr);
    M_FR.run(FORWARD);
    M_BR.run(FORWARD);
  }

  // move forward_left_wheels
  if(vl>=0){            
    M_FL.setSpeed(vl);
    M_BL.setSpeed(vl);
    M_FL.run(FORWARD);
    M_BL.run(FORWARD);
  }

  // move backward_right_wheels
  if(vr<0){             
    M_FR.setSpeed(-vr);
    M_BR.setSpeed(-vr);
    M_FR.run(BACKWARD);
    M_BR.run(BACKWARD);
  }

  
  if(vl<0){             
    M_FL.setSpeed(-vl);
    M_BL.setSpeed(-vl);
    M_FL.run(BACKWARD);
    M_BL.run(BACKWARD);
  }

  
}
