#include <AltSoftSerial.h>
AltSoftSerial Serial4;
#include<Encoder.h>

float Kp=0.50;
float Kp1=0.50;
float Kp2=0.50;
Encoder Enc1(2, 3);
Encoder Enc2(20,21);
#define actuatePin1 50//PISTON
#define actuatePin2 9
#define regulatorPin1 A8//PRESSURE REGULATOR
#define regulatorPin2 7
#define retractPin1 A5  //RETRACT
#define retractPin2 6
#define gripperPin1 5  //GRIPPER
#define gripperPin2 A4
#define cupPin1 8 //CUP8 A10
#define cupPin2 A9

#define gripSensorNO 51
#define zoneSensorNO 38

#define gripSensorNC 53
#define zoneSensorNC 36

#define flash A3
#define ledTZ1 A7 
#define ledTZ2 A10
#define ledTZ3 A6

#define rstTZ1 41
#define rstTZ2 45
#define rstTZ3 49

float a=0.30,b=0;

const byte serialEn = 29;    // Connect UART output enable of LSA08 to pin 2
const byte jPulse = 27;

const byte serialEn2 = 25;
const byte jPulse2 = 23;

const byte serialEn3 = 22;
const byte jPulse3 = 24;

const byte serialEn4 = 37;
const byte jPulse4 = 35;


int ppositionVal=255 , ppositionVal2=255, ppositionVal3=255,ppositionVal4=255;
int positionVal , positionVal2, positionVal3,positionVal4;
int jPulseVal,jPulseVal2,jPulseVal3,jPulseVal4;
int junction,prevError;
int prevpositionVal4,actualpositionVal4;
int prevpositionVal,actualpositionVal;
int botPosition;
int baseSpeed=50;
int maxSpeeed=255;
int minSpeed=50;
int startSteps=250;
unsigned long prevmillis;
int adder=2;

unsigned long stepsTravelled=0;
unsigned long totalSteps=0;
unsigned long stepsToTravel;
int prevRightMotorSpeed=50;
int prevLeftMotorSpeed=50;
int caseNum=1;
int preVal;
int slowFlag=0;
void TZS();
void AZS();
void stopBot();
int golden;
int statusT1=0,statusT2=0,statusT3=0,statusM2=0;
int zoneSensorNOData,zoneSensorNOData2;
int resetStart=0;
void setup() {
  
  //PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB20A_TXD2 | PIO_PB21A_RXD2, PIO_DEFAULT);
  Serial.begin(230400);
  Serial4.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
 
  pinMode(serialEn,OUTPUT);   
  pinMode(serialEn2,OUTPUT);
  pinMode(serialEn3,OUTPUT);
  pinMode(serialEn4,OUTPUT);
  
  pinMode(jPulse,INPUT);   
  pinMode(jPulse2,INPUT);
  pinMode(jPulse3,INPUT);
  pinMode(jPulse4,INPUT);

  pinMode(zoneSensorNO,INPUT);
  pinMode(gripSensorNO,INPUT);

  pinMode(actuatePin1,OUTPUT);
  pinMode(actuatePin2,OUTPUT);
  pinMode(regulatorPin1,OUTPUT);
  pinMode(regulatorPin2,OUTPUT);
  pinMode(retractPin1,OUTPUT);
  pinMode(retractPin2,OUTPUT);
  pinMode(gripperPin1,OUTPUT);
  pinMode(gripperPin2,OUTPUT);
  pinMode(cupPin1,OUTPUT);
  pinMode(cupPin2,OUTPUT);
  
  pinMode(flash,OUTPUT);
  pinMode(ledTZ1,OUTPUT);
  pinMode(ledTZ2,OUTPUT);
  pinMode(ledTZ3,OUTPUT);
    pinMode(rstTZ1,INPUT);
  pinMode(rstTZ2,INPUT);
  pinMode(rstTZ3,INPUT);
  digitalWrite(serialEn,HIGH);
  digitalWrite(serialEn2,HIGH);
  digitalWrite(serialEn3,HIGH);
  digitalWrite(serialEn4,HIGH);  
  Serial.println("start");
  digitalWrite(rstTZ1,LOW);
  digitalWrite(rstTZ2,LOW);
  digitalWrite(rstTZ3,LOW);

//pass();
initialize();
/*while(true){
   
    Serial.print("TZ1 ");
    Serial.println(digitalRead(rstTZ1));
    
    Serial.print("TZ2 ");
    Serial.println(digitalRead(rstTZ2));

    
    Serial.print("TZ3 ");
    Serial.println(digitalRead(rstTZ3));
    
}*/ 
while(true)
  { 

   if(digitalRead(rstTZ1)==1)
   {Serial.println("TZ1"); 
   digitalWrite(ledTZ1,LOW); 
    MZ1();
    resetStart=1;
    break;
   }
   else if(digitalRead(rstTZ2)==1)
   {Serial.println("TZ2"); 
   digitalWrite(ledTZ2,LOW); 
    MZ1();
    MZ2();
    statusT1=1;
    break;
   }
   else if(digitalRead(rstTZ3)==1)
   {Serial.println("TZ3");  
   digitalWrite(ledTZ3,LOW);
    MZ1();
    MZ2();
    statusT1=1;
    statusM2=3;
    statusT2=1;
    break;
   }
  
  }

    /* digitalWrite(regulatorPin1,HIGH);
     digitalWrite(regulatorPin2,HIGH);
int shootdelay=260;
 while(true)
  { 
    digitalWrite(ledTZ3,LOW);
    if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)    {
      pass(); 
      //]delay(1000);
      shoot(shootdelay);
      
    } 
   if(digitalRead(rstTZ1)==1)
   {shootdelay=shootdelay+1;
   while(true)
   {delay(500);
    if(digitalRead(rstTZ1)==0)
    break;
   }
   }
   else if(digitalRead(rstTZ2)==1)
   {
   shootdelay=shootdelay+2;
   while(true)
   {delay(500);
    if(digitalRead(rstTZ2)==0)
    break;
   }
   }
   else if(digitalRead(rstTZ3)==1)
   {shootdelay=shootdelay-1;
   while(true)
   {delay(500);
    if(digitalRead(rstTZ3)==0)
    break;
   }
   }
  Serial.println(shootdelay);

  }*/
  if(resetStart==1)
  {
  while(true)
  { 
    digitalWrite(ledTZ1,LOW);
    if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)
    {pass(); 
      delay(500);
      TZ1();
      Serial.println("here 2");
      break;

  
  }

}
  }
}

void loop() 
{
  if(digitalRead(zoneSensorNO)==1 && digitalRead(zoneSensorNC)==0 && statusT1==0)
  { digitalWrite(ledTZ1,LOW);
    while(true)
    { 
      if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)
      { pass();  
        //delay(500);
        TZ1();
        break;
      }
    }
  }
  else if(digitalRead(zoneSensorNO)==0 && digitalRead(zoneSensorNC)==1 && statusT1==0 && statusM2==0)
  {
    statusT1=1;
    //delay(500);
    Serial.println("here 5");
    MZ2();
  }
  else if(statusM2==1)
  {
    while(true)
    {digitalWrite(ledTZ2,LOW);
      if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)
      {pass(); 
        //delay(500);
        TZ2();
        Serial.println("here 4");
        statusM2=3;
        break;
       
      }
    }
  }
  else if(digitalRead(zoneSensorNO)==1 && digitalRead(zoneSensorNC)==0 && statusT2==0 && statusT1==1 && statusM2==3)
  {digitalWrite(ledTZ2,LOW);
    while(true)
    {
      if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)
      {pass(); 
      //  delay(500);
        TZ2();
         
        break;
      }
      Serial.println("heref 3");
    }
  }
  else if(digitalRead(zoneSensorNO)==0 && digitalRead(zoneSensorNC)==1 && statusT2==0 && statusT1==1 && statusM2==3)
  {digitalWrite(ledTZ3,LOW);
    statusT2=1;
    while(true)
    {
      if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)
      { pass(); 
        //delay(500);
        TZ3();
        break;       
      }
    }
  }
  else if(digitalRead(zoneSensorNO)==1 && digitalRead(zoneSensorNC)==0 && statusT2==1 && statusT1==1 && statusT3==0 && statusM2==3)
  {digitalWrite(ledTZ3,LOW);
    while(true)
    {
      if(digitalRead(gripSensorNO)==1 && digitalRead(gripSensorNC)==0)
      {pass(); 
        //delay(500);
        TZ3(); 
        
        Serial.println(golden);
        break;       
      }
      
    }
  }


  

}


void pid(int botPosition,int dir,float a,float b)
{       int error_pid =(35-botPosition);
          
        
        int motorSpeed =ceil(a * error_pid +b*(error_pid-prevError)) ;
        prevError=error_pid;
        int rightMotorSpeed = baseSpeed + motorSpeed; 
        int leftMotorSpeed = baseSpeed  - motorSpeed;


       
       if(rightMotorSpeed > maxSpeeed) 
          rightMotorSpeed = maxSpeeed;
       
       if(leftMotorSpeed > maxSpeeed) 
          leftMotorSpeed = maxSpeeed; 
        
       if(rightMotorSpeed < minSpeed) 
          rightMotorSpeed = minSpeed; 
          
       if(leftMotorSpeed < minSpeed) 
          leftMotorSpeed = minSpeed;


     

       if(rightMotorSpeed>prevRightMotorSpeed)
       {
        rightMotorSpeed=accelaratePid(rightMotorSpeed,20,prevRightMotorSpeed); 
       }
       else if(rightMotorSpeed<prevRightMotorSpeed)
       {
        rightMotorSpeed=decelaratePid(rightMotorSpeed,20,prevRightMotorSpeed);
       }
       else if(leftMotorSpeed>prevLeftMotorSpeed)
       {
        leftMotorSpeed=accelaratePid(leftMotorSpeed,20,prevLeftMotorSpeed); 
       }
       else if(leftMotorSpeed<prevLeftMotorSpeed)
       {
        leftMotorSpeed=decelaratePid(leftMotorSpeed,20,prevLeftMotorSpeed); 
       }

       Serial.print("botPosition\t");
       Serial.print("rightMotorSpeed\t");  
       Serial.print("leftMotorSpeed\t");  
       Serial.println("prevLeftMotorSpeed\t");       
       Serial.print(botPosition);
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(rightMotorSpeed);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(leftMotorSpeed);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(prevLeftMotorSpeed);
       Serial.print("\t");
       Serial.println("\t");

     
      if(dir==1){
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==2)
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
        //Serial.println("Here");
      }
      else if(dir==3)
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
        
      }
      else if(dir==4) 
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==5) 
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==6) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==7)
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
        Serial.println("Here");
      }
      else if(dir==8)
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
        
      }
      else if(dir==9) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==10) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==11) 
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==12) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==13)
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
        Serial.println("Here");
      }
      else if(dir==14)
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
        
      }
      else if(dir==15) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(rightMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==16) 
      {
        Serial3.write(byte(leftMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==17) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(leftMotorSpeed));
        Serial1.write(byte(leftMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;
      }
      else if(dir==18) 
      {
        Serial3.write(byte(rightMotorSpeed));
        Serial2.write(byte(rightMotorSpeed));
        Serial1.write(byte(rightMotorSpeed));
        Serial4.write(byte(leftMotorSpeed));
        stepsTravelled++;

      }
      
prevLeftMotorSpeed=leftMotorSpeed;
prevRightMotorSpeed=rightMotorSpeed;
}


void accelarate(int maxBase,int delayStep)
{
    if(millis()-prevmillis>=delayStep && baseSpeed<=maxBase){
      adder=adder*1.4;
      baseSpeed=baseSpeed+adder;
      prevmillis=millis();
    } 
    if(baseSpeed>maxBase)
     baseSpeed=maxBase;
}

int accelaratePid(int maxBase,int delayStep,int current)
{
    if(millis()-prevmillis>=delayStep && current<=maxBase){
      adder=adder*1;
      current=current+adder;
      prevmillis=millis();
    } 
    if(current>maxBase)
     current=maxBase;
    Serial.println("Accelerate"); 
    Serial.println(current);
    Serial.println(maxBase);
     return current;
}

int decelaratePid(int maxBase,int delayStep,int current)
{
    if(millis()-prevmillis>=delayStep && current>=maxBase){
      adder=adder*1;
      current=current-adder;
      prevmillis=millis();
    } 
    if(current<maxBase)
     current=maxBase;
    
     return current;
}

void decelarate(int minBase,int delayStep)
{
 
      if(millis()-prevmillis>=delayStep && baseSpeed>=minBase){
      adder=adder*1.1;
      baseSpeed=baseSpeed-adder;
      prevmillis=millis();
      } 

      if(baseSpeed<50)
      baseSpeed=50;
}


void forward(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ stepsToTravel=16000;
   
  Serial3.write(2);
  Serial2.write(2);
  Serial1.write(1);
  Serial4.write(byte(1));
  long  step1,step2;
  stepsTravelled=0;
  slowFlag=0;
int prevposition2;
  int Val;
  while(true)
  {            
             step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;
              digitalWrite(serialEn,LOW);        
              while(Serial1.available() <= 0);   
              positionVal = Serial1.read();      
              digitalWrite(serialEn,HIGH);  
              jPulseVal=digitalRead(jPulse);     
           
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
          
              
              digitalWrite(serialEn4,LOW);       
              while(Serial4.available() <= 0);  
              actualpositionVal4 = Serial4.read();     
              digitalWrite(serialEn4,HIGH);      
              jPulseVal4=digitalRead(jPulse4);
            

              if(actualpositionVal4==0 && prevpositionVal4==0)
              positionVal4=0;
              else
              {if(prevpositionVal4>actualpositionVal4)
              positionVal4=prevpositionVal4;
              else
              positionVal4=actualpositionVal4;
              }


               prevpositionVal4=actualpositionVal4;
              
              if(positionVal4<=70 && positionVal4>=0)
              positionVal4=70-positionVal4;

             if(positionVal3>70)
             positionVal3=ppositionVal3;
             if(positionVal4>70)
             positionVal4=ppositionVal4;
             if(positionVal3<=70 && positionVal3>=0)
             {
                
                
                if(positionVal4>=0 && positionVal4<=70)
                {
                 
                    if(positionVal3>=30 && positionVal3<=40)
                    {
                      if(positionVal4>=30 && positionVal4<=40)
                      {
                        pid(35,9,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal4-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,9,Kp2,0); //N1
                       else
                       pid(Val,10,Kp2,0); //N2
                       
                      }
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<30 && positionVal4>=0){
                      Val=(positionVal3+positionVal4)/2;
                      pid(Val,8,Kp1,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<=70 && positionVal4>40){
                      Val=(positionVal3+positionVal4)/2; 
                      pid(Val,8,Kp1,0);
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<=70 && positionVal4>40){
                     Val=map(positionVal3-positionVal4,-70,70,0,70); 
                     Serial.println("Here");
                     Serial.println(Val);
                     pid(Val,7,Kp,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<30 && positionVal4>=0){
                      Val=map(positionVal3-positionVal4,-70,70,0,70);
                      pid(Val,7,Kp,0);
                    }
                    else if(positionVal4>=30 && positionVal4<=40){
                       Val=map(positionVal3-35,-35,35,0,70);
                       if(Val>=35)
                       pid(Val,12,Kp2,0);
                       else
                       pid(Val,11,Kp2,0);
                       
                    }
               
               if(positionVal3>=10 && positionVal3<=60 && stepsTravelled<=totalSteps && stepsTravelled>=startSteps && slowFlag==0)
                    accelarate(maxspeed,aStepDelay);    
               
                  

                 
              }  
              else 
              {                   
                  pid(positionVal3,7,Kp,0);
              }
              
             }    

             else if(positionVal4>=0 && positionVal4<=70)
             {
                  pid(positionVal4,13,Kp,0);
             }
             else if(positionVal3>70)
             {
              stopBot();  
             }

               if(stepsTravelled>=totalSteps){
                    decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }
              Serial.println("---------------------FORWARD---------------------------------------------");
              Serial.println(positionVal);

              
             if(positionVal>=30 && positionVal<=40 && slowFlag==1)
              {
                  stopBot();
                  break;
              }
            
        ppositionVal3=positionVal3;
        ppositionVal4=positionVal4;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}
void forward2(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ stepsToTravel=16000;
   
  Serial3.write(2);
  Serial2.write(2);
  Serial1.write(1);
  Serial4.write(byte(1));
  stepsTravelled=0;
  slowFlag=0;
  long  step1,step2;
int prevposition2;
  int Val;
  while(true)
  {          step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;
              digitalWrite(serialEn,LOW);        
              while(Serial1.available() <= 0);   
              positionVal = Serial1.read();      
              digitalWrite(serialEn,HIGH);  
              jPulseVal=digitalRead(jPulse);     
         
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
       
              
              digitalWrite(serialEn4,LOW);       
              while(Serial4.available() <= 0);  
              actualpositionVal4 = Serial4.read();     
              digitalWrite(serialEn4,HIGH);      
              jPulseVal4=digitalRead(jPulse4);
         

              if(actualpositionVal4==0 && prevpositionVal4==0)
              positionVal4=0;
              else
              {if(prevpositionVal4>actualpositionVal4)
              positionVal4=prevpositionVal4;
              else
              positionVal4=actualpositionVal4;
              }


               prevpositionVal4=actualpositionVal4;
              
              if(positionVal4<=70 && positionVal4>=0)
              positionVal4=70-positionVal4;


             if(positionVal3>70)
             positionVal3=ppositionVal3;
             if(positionVal4>70)
             positionVal4=ppositionVal4;
             if(positionVal3<=70 && positionVal3>=0)
             {
                
                
                if(positionVal4>=0 && positionVal4<=70)
                {

                    if(positionVal3>=30 && positionVal3<=40)
                    {
                      if(positionVal4>=30 && positionVal4<=40)
                      {
                        pid(35,9,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal4-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,9,Kp,0); //N1
                       else
                       pid(Val,10,Kp,0); //N2
                       
                      }
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<30 && positionVal4>=0){
                      Val=(positionVal3+positionVal4)/2;
                      pid(Val,8,Kp,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<=70 && positionVal4>40){
                      Val=(positionVal3+positionVal4)/2; 
                      pid(Val,8,Kp,0);
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<=70 && positionVal4>40){
                     Val=map(positionVal3-positionVal4,-70,70,0,70); 
                     Serial.println("Here");
                     Serial.println(Val);
                     pid(Val,7,Kp,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<30 && positionVal4>=0){
                      Val=map(positionVal3-positionVal4,-70,70,0,70);
                      pid(Val,7,Kp,0);
                    }
                    else if(positionVal4>=30 && positionVal4<=40){
                       Val=map(positionVal3-35,-35,35,0,70);
                       if(Val>=35)
                       pid(Val,12,Kp,0);
                       else
                       pid(Val,11,Kp,0);
                       
                    }
               
               if(positionVal3>=10 && positionVal3<=60 && stepsTravelled<=totalSteps && stepsTravelled>=startSteps && slowFlag==0)
                    accelarate(maxspeed,aStepDelay);    

                  

                 
              }  
              else 
              {                   
                  pid(positionVal3,7,Kp,0);
              }
             }    

             else if(positionVal4>=0 && positionVal4<=70)
             {
                  pid(positionVal3,13,Kp,0);
             }
             else if(positionVal3>70)
             {
              stopBot();  
             }

               if(stepsTravelled>=totalSteps){
                    decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }
              Serial.println("---------------------FORWARD 2---------------------------------------------");
              

              
             if(positionVal4>=30 && positionVal4<=40 && slowFlag==1 && jPulseVal4==1)
              {
                  stopBot();
                  break;
              }

        preVal=Val;
        ppositionVal3=positionVal3;
        ppositionVal4=positionVal4;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}

void forward3(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ stepsToTravel=16000;
   
  Serial3.write(2);
  Serial2.write(2);
  Serial1.write(1);
  Serial4.write(byte(1));
  long  step1,step2;
  stepsTravelled=0;
  slowFlag=0;
int prevposition2;
  int Val;
  while(true)
  {            
             step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;
              digitalWrite(serialEn,LOW);        
              while(Serial1.available() <= 0);   
              positionVal = Serial1.read();      
              digitalWrite(serialEn,HIGH);  
              jPulseVal=digitalRead(jPulse);     
    
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
          
              
              digitalWrite(serialEn4,LOW);       
              while(Serial4.available() <= 0);  
              actualpositionVal4 = Serial4.read();     
              digitalWrite(serialEn4,HIGH);      
              jPulseVal4=digitalRead(jPulse4);
         

              if(actualpositionVal4==0 && prevpositionVal4==0)
              positionVal4=0;
              else
              {if(prevpositionVal4>actualpositionVal4)
              positionVal4=prevpositionVal4;
              else
              positionVal4=actualpositionVal4;
              }
              
          
               prevpositionVal4=actualpositionVal4;
              
              if(positionVal4<=70 && positionVal4>=0)
              positionVal4=70-positionVal4;
             
             if(positionVal3>70)
             positionVal3=ppositionVal3;
             if(positionVal4>70)
             positionVal4=ppositionVal4;

             if(positionVal3<=70 && positionVal3>=0)
             {
                
                
                if(positionVal4>=0 && positionVal4<=70)
                {

                    if(positionVal3>=30 && positionVal3<=40)
                    {
                      if(positionVal4>=30 && positionVal4<=40)
                      {
                        pid(35,9,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal4-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,9,Kp2,0); //N1
                       else
                       pid(Val,10,Kp2,0); //N2
                       
                      }
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<30 && positionVal4>=0){
                      Val=(positionVal3+positionVal4)/2;
                      pid(Val,8,Kp1,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<=70 && positionVal4>40){
                      Val=(positionVal3+positionVal4)/2; 
                      pid(Val,8,Kp1,0);
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<=70 && positionVal4>40){
                     Val=map(positionVal3-positionVal4,-70,70,0,70); 
                     //Serial.println("Here");
                     //Serial.println(Val);
                     pid(Val,7,Kp,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<30 && positionVal4>=0){
                      Val=map(positionVal3-positionVal4,-70,70,0,70);
                      pid(Val,7,Kp,0);
                    }
                    else if(positionVal4>=30 && positionVal4<=40){
                       Val=map(positionVal3-35,-35,35,0,70);
                       if(Val>=35)
                       pid(Val,12,Kp2,0);
                       else
                       pid(Val,11,Kp2,0);
                       
                    }
               
               if(positionVal3>=10 && positionVal3<=60 && stepsTravelled<=totalSteps && stepsTravelled>=startSteps && slowFlag==0)
                    accelarate(maxspeed,aStepDelay);    

                  

                 
              }  
              else 
              {                   
                  pid(positionVal3,7,Kp,0);
              }
             }    

             else if(positionVal4>=0 && positionVal4<=70)
             {
                  pid(positionVal4,13,Kp,0);
             }
             else if(positionVal3>70)
             {
              stopBot();  
             }

               if(stepsTravelled>=totalSteps){
                    //decelarate(minspeed,dStepDelay);
                  //  slowFlag=1;
               }
              Serial.println("---------------------FORWARD 3---------------------------------------------");
              Serial.println(positionVal);

              
             if(positionVal>=10 && positionVal<=60 /*&& slowFlag==1*/)
              {   Enc1.write(0);
                  Enc2.write(0);
                  forward(20,20,230,50,2800);
                  break;
              }

        preVal=Val;
        ppositionVal3=positionVal3;
        ppositionVal4=positionVal4;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}



void backward(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ stepsToTravel=16000;
   
  Serial3.write(1);
  Serial2.write(1);
  Serial1.write(2);
  Serial4.write(byte(2));
  stepsTravelled=0;
  slowFlag=0;
  long  step1,step2;
int prevposition2;
  int Val;
  while(true)
  {           step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;
              digitalWrite(serialEn,LOW);        
              while(Serial1.available() <= 0);   
              positionVal = Serial1.read();      
              digitalWrite(serialEn,HIGH);  
              jPulseVal=digitalRead(jPulse);     
            
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
              
              
              digitalWrite(serialEn4,LOW);       
              while(Serial4.available() <= 0);  
              actualpositionVal4 = Serial4.read();     
              digitalWrite(serialEn4,HIGH);      
              jPulseVal4=digitalRead(jPulse4);
              

              if(actualpositionVal4==0 && prevpositionVal4==0)
              positionVal4=0;
              else
              {if(prevpositionVal4>actualpositionVal4)
              positionVal4=prevpositionVal4;
              else
              positionVal4=actualpositionVal4;
              }
              


               prevpositionVal4=actualpositionVal4;
              
              if(positionVal3<=70 && positionVal3>=0)
              positionVal3=70-positionVal3;
             
             if(positionVal3>70)
             positionVal3=ppositionVal3;
             if(positionVal4>70)
             positionVal4=ppositionVal4;

             if(positionVal4<=70 && positionVal4>=0)
             {
                
                
                if(positionVal3>=0 && positionVal3<=70)
                {

                    if(positionVal4>=30 && positionVal4<=40)
                    {
                      if(positionVal3>=30 && positionVal3<=40)
                      {
                        pid(35,15,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal3-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,15,Kp2,0); //N1
                       else
                       pid(Val,16,Kp2,0); //N2
                       
                      }
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<30 && positionVal4>=0){
                      Val=(positionVal3+positionVal4)/2;
                      pid(Val,14,Kp1,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<=70 && positionVal4>40){
                      Val=(positionVal3+positionVal4)/2; 
                      pid(Val,14,Kp1,0);
                    }
                    else if(positionVal4<30 && positionVal4>=0 && positionVal3<=70 && positionVal3>40){
                     Val=map(positionVal4-positionVal3,-70,70,0,70); 
                     Serial.println("Here");
                     Serial.println(Val);
                     pid(Val,13,Kp,0);
                    }
                    else if(positionVal4<=70 && positionVal4>40 && positionVal3<30 && positionVal3>=0){
                      Val=map(positionVal4-positionVal3,-70,70,0,70);
                      pid(Val,13,Kp,0);
                    }
                    else if(positionVal3>=30 && positionVal3<=40){
                       Val=positionVal4;
                       if(Val>=35)
                       pid(Val,18,Kp2,0);
                       else
                       pid(Val,17,Kp2,0);
                       Serial.println(Val);
                       
                    }
               
               if(positionVal4>=10 && positionVal4<=60 && stepsTravelled<=totalSteps && stepsTravelled>=startSteps && slowFlag==0 )
                    accelarate(maxspeed,aStepDelay);    

                  

                 
              }  
              else 
              {                   
                   pid(positionVal4,13,Kp,0);
              }
             }    

             
             else if(positionVal4>70)
             {
              stopBot();  
             }

               if(stepsTravelled>=totalSteps){
                    decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }
              Serial.println("---------------------BACKWARD---------------------------------------------");
              

              
             if(positionVal4>=30 && positionVal4<=40 && slowFlag==1 && jPulseVal4==1)
              {
                  stopBot();
                  break;
              }

        preVal=Val;
        ppositionVal3=positionVal3;
        ppositionVal4=positionVal4;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}

void backward2(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ stepsToTravel=16000;
   
  Serial3.write(1);
  Serial2.write(1);
  Serial1.write(2);
  Serial4.write(byte(2));
  stepsTravelled=0;
  slowFlag=0;
  long  step1,step2;
int prevposition2;
  int Val;
  while(true)
  {           step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;
              digitalWrite(serialEn,LOW);        
              while(Serial1.available() <= 0);   
              positionVal = Serial1.read();      
              digitalWrite(serialEn,HIGH);  
              jPulseVal=digitalRead(jPulse);     
            
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
              
              
              digitalWrite(serialEn4,LOW);       
              while(Serial4.available() <= 0);  
              actualpositionVal4 = Serial4.read();     
              digitalWrite(serialEn4,HIGH);      
              jPulseVal4=digitalRead(jPulse4);
              

              if(actualpositionVal4==0 && prevpositionVal4==0)
              positionVal4=0;
              else
              {if(prevpositionVal4>actualpositionVal4)
              positionVal4=prevpositionVal4;
              else
              positionVal4=actualpositionVal4;
              }
              

               prevpositionVal4=actualpositionVal4;
              
              if(positionVal3<=70 && positionVal3>=0)
              positionVal3=70-positionVal3;

            if(positionVal3>70)
             positionVal3=ppositionVal3;
             if(positionVal4>70)
             positionVal4=ppositionVal4;
             if(positionVal4<=70 && positionVal4>=0)
             {
                
                
                if(positionVal3>=0 && positionVal3<=70)
                {

                    if(positionVal4>=30 && positionVal4<=40)
                    {
                      if(positionVal3>=30 && positionVal3<=40)
                      {
                        pid(35,15,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal3-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,15,Kp2,0); //N1
                       else
                       pid(Val,16,Kp2,0); //N2
                       
                      }
                    }
                    else if(positionVal3<30 && positionVal3>=0 && positionVal4<30 && positionVal4>=0){
                      Val=(positionVal3+positionVal4)/2;
                      pid(Val,14,Kp1,0);
                    }
                    else if(positionVal3<=70 && positionVal3>40 && positionVal4<=70 && positionVal4>40){
                      Val=(positionVal3+positionVal4)/2; 
                      pid(Val,14,Kp1,0);
                    }
                    else if(positionVal4<30 && positionVal4>=0 && positionVal3<=70 && positionVal3>40){
                     Val=map(positionVal4-positionVal3,-70,70,0,70); 
                     Serial.println("Here");
                     Serial.println(Val);
                     pid(Val,13,Kp,0);
                    }
                    else if(positionVal4<=70 && positionVal4>40 && positionVal3<30 && positionVal3>=0){
                      Val=map(positionVal4-positionVal3,-70,70,0,70);
                      pid(Val,13,Kp,0);
                    }
                    else if(positionVal3>=30 && positionVal3<=40){
                       Val=positionVal4;
                       if(Val>=35)
                       pid(Val,18,Kp2,0);
                       else
                       pid(Val,17,Kp2,0);
                       Serial.println(Val);
                      
                       
                    }
               
               if(positionVal4>=20 && positionVal4<=50 && stepsTravelled>=startSteps && stepsTravelled<=totalSteps && slowFlag==0)
                    accelarate(maxspeed,aStepDelay);    

                if(stepsTravelled>=totalSteps){
                    //decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }
                  

                 
              }  
              else 
              {                   
                   pid(positionVal4,13,Kp,0);
              }
             }    

             
             else if(positionVal4>70)
             {
              stopBot();  
             }

               /*if(stepsTravelled>=totalSteps){
                    decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }*/
              Serial.println("---------------------BACKWARD 2---------------------------------------------");
              

              
             if(positionVal>=30 && positionVal<=40 && slowFlag==1)
              {  Enc1.write(0);
                  Enc2.write(0);
                  backward(20,20,230,50,2400);
                  
                  break;
              }

        preVal=Val;
                ppositionVal3=positionVal3;
        ppositionVal4=positionVal4;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}

void right(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ 
  stepsToTravel=16000;
  long  step1,step2;
  Serial3.write(1);
  Serial2.write(2);
  Serial1.write(2);
  Serial4.write(byte(1));
  stepsTravelled=0;
  slowFlag=0;
//int prevposition;
  int Val;
  while(true)
  {          step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;
              digitalWrite(serialEn,LOW);        
              while(Serial1.available() <= 0);   
              actualpositionVal=Serial1.read();      
              digitalWrite(serialEn,HIGH);  
              jPulseVal=digitalRead(jPulse);     
            

              digitalWrite(serialEn2,LOW);       
              while(Serial2.available() <= 0);  
              positionVal2 = Serial2.read();     
              digitalWrite(serialEn2,HIGH);      
              jPulseVal2=0;//digitalRead(jPulse2);

            
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
              
              if(positionVal2<=70 && positionVal2>=0)
              positionVal2=70-positionVal2;

 
              if(actualpositionVal==0 && prevpositionVal==0)
              positionVal=0;
              else
              {if(prevpositionVal>actualpositionVal)
              positionVal=prevpositionVal;
              else
              positionVal=actualpositionVal;
              }
              
    
               prevpositionVal=actualpositionVal;
 
             /*if(positionVal>70)
             positionVal=ppositionVal;
             if(positionVal2>70)
             positionVal2=ppositionVal2;*/
             if(positionVal<=70 && positionVal>=0)
             {
                
                
                if(positionVal2>=0 && positionVal2<=70)
                {

                    if(positionVal>=30 && positionVal<=40)
                    {
                      if(positionVal2>=30 && positionVal2<=40)
                      {
                        pid(35,3,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal2-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,3,Kp2,0); //N1
                       else
                       pid(Val,4,Kp2,0); //N2
                       
                      }
                    }
                    else if(positionVal<30 && positionVal>=0 && positionVal2<30 && positionVal2>=0){
                      Val=(positionVal+positionVal2)/2;
                      pid(Val,2,Kp1,0);
                    }
                    else if(positionVal<=70 && positionVal>40 && positionVal2<=70 && positionVal2>40){
                      Val=(positionVal+positionVal2)/2; 
                      pid(Val,2,Kp1,0);
                    }
                    else if(positionVal<30 && positionVal>=0 && positionVal2<=70 && positionVal2>40){
                     Val=map(positionVal-positionVal2,-70,70,0,70); 
                     pid(Val,1,Kp,0);
                    }
                    else if(positionVal<=70 && positionVal>40 && positionVal2<30 && positionVal2>=0){
                      Val=map(positionVal-positionVal2,-70,70,0,70);
                      pid(Val,1,Kp,0);
                    }
                    else if(positionVal2>=30 && positionVal2<=40){
                       Val=map(positionVal-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,6,Kp2,0);
                       else
                       pid(Val,5,Kp2,0);
                       
                    }
               
               

                  

                 
              }  
              else 
              {                   
                   pid(positionVal,1,Kp,0);
              }
              if(positionVal>=10 && positionVal<=60 && stepsTravelled<=totalSteps && stepsTravelled>=startSteps && slowFlag==0){
                    accelarate(maxspeed,aStepDelay);    
                    
               }
             }    

             
             else if(positionVal>70)
             {
              stopBot();  
             }

               if(stepsTravelled>=totalSteps){
                    decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }
              Serial.println("---------------------RIGHT---------------------------------------------");
              

              
              if(positionVal3>=30 && positionVal3<=40 && slowFlag==1)
              {
                  stopBot();
                  break;
              }

        preVal=Val;
        ppositionVal=positionVal;
        ppositionVal2=positionVal;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}

void right2(int aStepDelay,int dStepDelay,int maxspeed,int minspeed,long  totalSteps)
{ 
  stepsToTravel=16000;
  long  step1,step2; 
  Serial3.write(1);
  Serial2.write(2);
  Serial1.write(2);
  Serial4.write(byte(1));
  stepsTravelled=0;
  slowFlag=0;
int prevposition2;
  int Val;
  while(true)
  {
             step1=abs(Enc1.read());
             step2=abs(Enc2.read());
             stepsTravelled=(abs(step1)+abs(step2))/2;  
              positionVal=35;      
             
              positionVal2 =35;     
              
              digitalWrite(serialEn3,LOW);       
              while(Serial3.available() <= 0);  
              positionVal3 = Serial3.read();     
              digitalWrite(serialEn3,HIGH);      
              jPulseVal3=digitalRead(jPulse3);
              
              if(positionVal2<=70 && positionVal2>=0)
              positionVal2=70-positionVal2;



             if(positionVal<=70 && positionVal>=0)
             {
                
                
                if(positionVal2>=0 && positionVal2<=70)
                {

                    if(positionVal>=30 && positionVal<=40)
                    {
                      if(positionVal2>=30 && positionVal2<=40)
                      {
                        pid(35,3,Kp,0);
                      }
                      else
                      {
                       
                       Val=map(positionVal2-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,3,Kp,0); //N1
                       else
                       pid(Val,4,Kp,0); //N2
                       
                      }
                    }
                    else if(positionVal<30 && positionVal>=0 && positionVal2<30 && positionVal2>=0){
                      Val=(positionVal+positionVal2)/2;
                      pid(Val,2,Kp,0);
                    }
                    else if(positionVal<=70 && positionVal>40 && positionVal2<=70 && positionVal2>40){
                      Val=(positionVal+positionVal2)/2; 
                      pid(Val,2,Kp,0);
                    }
                    else if(positionVal<30 && positionVal>=0 && positionVal2<=70 && positionVal2>40){
                     Val=map(positionVal-positionVal2,-70,70,0,70); 
                     pid(Val,1,Kp,0);
                    }
                    else if(positionVal<=70 && positionVal>40 && positionVal2<30 && positionVal2>=0){
                      Val=map(positionVal-positionVal2,-70,70,0,70);
                      pid(Val,1,Kp,0);
                    }
                    else if(positionVal2>=30 && positionVal2<=40){
                       Val=map(positionVal-35,-35,35,0,70);
                       if(Val>0)
                       pid(Val,6,Kp,0);
                       else
                       pid(Val,5,Kp,0);
                       
                    }
               
               if(positionVal>=20 && positionVal<=50 && stepsTravelled<=totalSteps && stepsTravelled>=startSteps )
                    accelarate(maxspeed,aStepDelay);    

                  

                 
              }  
              else 
              {                   
                   pid(positionVal,1,Kp,0);
              }
             }    

             
             else if(positionVal>70)
             {
              stopBot();  
             }

               if(stepsTravelled>=totalSteps){
                    decelarate(minspeed,dStepDelay);
                    slowFlag=1;
               }
              Serial.println("---------------------RIGHT 2---------------------------------------------");
              

              
              if(positionVal3>=30 && positionVal3<=40 && slowFlag==1)
              {
                  stopBot();
                  break;
              }

        preVal=Val;
       Serial.print(totalSteps);  
       Serial.print("\t");
       Serial.print("\t");
       Serial.print(stepsTravelled);
       Serial.print("\t");
       Serial.println("\t");
  
   }
}


void stopBot()
{     
      Serial4.write(byte(0));
      Serial1.write(byte(0));
      Serial2.write(byte(0));
      Serial3.write(byte(0)); 
      Serial.println("Stop Bot");
      baseSpeed=50;
      prevRightMotorSpeed=50;
      prevLeftMotorSpeed=50;
      Enc1.write(0);
      Enc2.write(0);
      
}

void shoot(int time_delay)
{
   digitalWrite(cupPin1,LOW);
   digitalWrite(cupPin2,LOW);
    delay(500);
   
   digitalWrite(flash,LOW);      
  
   digitalWrite(retractPin1,LOW);
   digitalWrite(retractPin2,LOW); 
   
   digitalWrite(actuatePin1,HIGH);
   digitalWrite(actuatePin2,HIGH);
   
   delay(time_delay);
   
   digitalWrite(flash,HIGH); 
   
   digitalWrite(gripperPin1,HIGH);
   digitalWrite(gripperPin2,HIGH);
  
   digitalWrite(actuatePin2,LOW);
   digitalWrite(actuatePin1,LOW);
  
}
void initialize()
{
    stopBot();

    digitalWrite(actuatePin2,LOW);
    digitalWrite(actuatePin1,LOW);
    
    digitalWrite(regulatorPin2,LOW);
    digitalWrite(regulatorPin1,LOW);
    
    digitalWrite(retractPin1,LOW);
    digitalWrite(retractPin2,LOW);

    digitalWrite(gripperPin1,HIGH);
    digitalWrite(gripperPin2,HIGH);

    digitalWrite(cupPin1,LOW);
    digitalWrite(cupPin2,LOW);

    digitalWrite(flash,HIGH);

   digitalWrite(ledTZ1,HIGH); 
   digitalWrite(ledTZ2,HIGH); 
   digitalWrite(ledTZ3,HIGH);
   delay(100);
  
}
void MZ1()
{
   forward(20,20,50,50,500);
   //delay(500);
   
   right(20,20,230,50,6400);
   //delay(500);
   
   forward2(20,20,50,50,1000);
   
   Serial.println("MZ1");
}

void TZ1()
{   
    forward(20,20,230,50,3000);
    delay(1000);
    digitalWrite(ledTZ1,HIGH);
    shoot(273);
    //delay(1000);
    
    backward(20,20,230,50,3200);
    delay(500);
    
    Serial.println("TZ1");
}

void TZ2()
{   
    forward(20,20,230,50,3000);
    delay(1000);
    digitalWrite(ledTZ2,HIGH);
    shoot(265);
    //delay( 1000);
    
    backward(20,20,230,50,3000);
    delay(500);
    
    Serial.println("TZ2");
}

void TZ3()
{   
    digitalWrite(regulatorPin1,HIGH);
    digitalWrite(regulatorPin2,HIGH);
    
    forward3(20,20,230,50,3000);
    delay(1000);
    digitalWrite(ledTZ3,HIGH);
    shoot(258);
    //delay(1000);
    
    backward2(20,20,210,50,3000);
    
    Serial.println("TZ3");
    ++golden;
}

void MZ2()
{
    right2(20,20,100,50,3000);
    delay(500);
    
    statusM2=1;
    
    Serial.println("MZ2");
}
void pass()
{
         digitalWrite(gripperPin1,HIGH);
         digitalWrite(gripperPin2,HIGH);
         delay(1250);

         digitalWrite(gripperPin1,LOW);
         digitalWrite(gripperPin2,LOW);
         delay(100);
         digitalWrite(cupPin1,HIGH);
         digitalWrite(cupPin2,HIGH);
         delay(1000);
}

