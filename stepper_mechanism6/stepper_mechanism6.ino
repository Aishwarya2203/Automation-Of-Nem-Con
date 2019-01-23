
#include <AltSoftSerial.h>
#include <Stepper.h>
int stepsPerRevolution = 1600; 
AltSoftSerial Aserial;
Stepper myStepper(stepsPerRevolution,2,3);

int val,pval;
void setup() {
  
  Aserial.begin(9600);
  Serial.begin(9600);
  //delay(1000);
}
int Speed=0,prevmillis;
int adder=20;
int dir;
int switchMotor;
void loop() {

  if(Serial.available())
    val=Serial.read();

  if(val==1)
  dir=1;
  else if(val==2)
  dir=-1;

  if(val==3)
  switchMotor=1;
  else if(val==4)
  switchMotor=-1;
  
  //Serial.println(val);
  
  if(val<0){
      val=pval;
  }

  if(val>=30 && val<=255)
  {
    Speed=map(val,50,255,30,1000);  

    
    myStepper.setSpeed(Speed); 
    myStepper.step(dir*2); 

    
  }
  else
      stepsPerRevolution=0;
 
  pval=val;
}

