#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#define ENCA3 3
#define enbpinb3 2
#define ENCA1 19
#define enbpinb1 18
#define ENCA2 21
#define enbpinb2 20
#define PWM1 6
#define INm1 5
char blueToothVal;
#define PWM3  13
#define INm31 11
#define INm32 12

#define PWM2 8
#define INm2 10

volatile int posi1 = 0;
volatile int posi2 = 0;
volatile int posi3 = 0;
double Degree1;
double Degree2;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA1,INPUT_PULLUP);
  pinMode(enbpinb1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,RISING);
  pinMode(ENCA2,INPUT_PULLUP);
  pinMode(enbpinb2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3),readEncoder3,RISING);
  
  pinMode(PWM1,OUTPUT);
  //pinMode(INm11,OUTPUT);
   pinMode(INm1,OUTPUT);
    pinMode(PWM2,OUTPUT);
   pinMode(INm2,OUTPUT);
    pinMode(INm31,OUTPUT);
   pinMode(INm32,OUTPUT);
   pinMode(PWM3,OUTPUT);
   //pinMode(INm22,OUTPUT);
  Serial.println("target pos"); 
}

void loop() 
{
Degree1=360;
//Motor_postion_control(Degree1, PWM2, INm2,posi2);
Serial.print("    "); 
Motor_postion_control(Degree1*2, PWM1, INm1,posi1);
 Serial.println(""); 
  //setMotorH(1,255,PWM2,INm21,INm22);
//Motor_postion_control1(Degree1, PWM1, INm1,posi3);
//setMotorH(1,255,PWM3,INm31,INm32);
 
  
}
void setMotorH(int dir1, int pwmVal1, int pwm1, int in11, int in12)
{
  analogWrite(pwm1,pwmVal1);
  if(dir1 == 1){
    digitalWrite(in11,HIGH);
    digitalWrite(in12,LOW);
   }
  else if(dir1 == -1){
    digitalWrite(in11,LOW);
    digitalWrite(in12,HIGH);
    
  }
  else{
    digitalWrite(in11,LOW);
    digitalWrite(in12,LOW);
    analogWrite(pwm1,0);
  
  }
}
void setMotor(int dir, int pwmVal, int pwm, int in1)
{
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
   }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    
  }
  else{
    analogWrite(pwm,0);
  
  }
}

void readEncoder1(){
  int b1 = digitalRead(enbpinb1);
  if(b1 > 0){
    posi1++;
  }
  else{
    posi1--;
  }
}

void readEncoder2(){
  int b2 = digitalRead(enbpinb2);
  if(b2 > 0){
    posi2++;
  }
  else{
    posi2--;
  }
}
void readEncoder3(){
  int b2 = digitalRead(enbpinb3);
  if(b2 > 0){
    posi3++;
  }
  else{
    posi3--;
  }
}
void Motor_postion_control(double degree,int PWMm,int INM11, int posi)
{  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  long prevT = 0;
  float eprev = 0;
  float eintegral = 0;
  
  double de= degree ;
  double t1= (600*degree);
  double t2 = t1/360;

  int target =t2;


  // PID constants
  float kp = 6.8;
  float kd = 0.025;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 125 ){
    pwr = 125;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWMm,INM11);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");

 
}

void Motor_postion_control1(double degree1,int PWMm1,int INM1, int posi1)
{  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  long prevT1 = 0;
  float eprev1= 0;
  float eintegral1 = 0;
  
  double de1= degree1 ;
  double t11= (600*degree1);
  double t21 = t11/360;

  int target1 =t21;


  // PID constants
  float kp1 = 6.8;
  float kd1 = 0.025;
  float ki1 = 0;

  // time difference
  long currT1 = micros();
  float deltaT1 = ((float) (currT1 - prevT1))/( 1.0e6 );
  prevT1 = currT1;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos1 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
  }
  
  // error
  int e1 = target1 - pos1;

  // derivative
  float dedt1 = (e1-eprev1)/(deltaT1);

  // integral
  eintegral1 = eintegral1 + e1*deltaT1;

  // control signal
  float u1 = kp1*e1 + kd1*dedt1 + ki1*eintegral1;

  // motor power
  float pwr1 = fabs(u1);
  if( pwr1 > 255 ){
    pwr1 = 255;
  }

  // motor direction
  int dir1 = 1;
  if(u1<0){
    dir1 = -1;
  }

  // signal the motor
  setMotorH(dir1,pwr1,PWM3,INm31,INm32);


  // store previous error
  eprev1 = e1;

  Serial.print(target1);
  Serial.print(" ");
  Serial.print(pos1);
  Serial.print(" ");

 
}/*
void Motor_postion_control2(double degree2,int PWMm2,int INM21,int INM22, int posi2)
{  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  long prevT2 = 0;
  float eprev2= 0;
  float eintegral2 = 0;
  
  double de2= degree2 ;
  double t12= (600*degree2);
  double t22 = t12/360;

  int target2 =t22;


  // PID constants
  float kp2 = 6.8;
  float kd2 = 0.025;
  float ki2 = 0;

  // time difference
  long currT2 = micros();
  float deltaT2 = ((float) (currT2 - prevT2))/( 1.0e6 );
  prevT2 = currT2;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos2 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos2 = posi2;
  }
  
  // error
  int e2 = target2 - pos2;

  // derivative
  float dedt2 = (e2-eprev2)/(deltaT2);

  // integral
  eintegral2 = eintegral2 + e2*deltaT2;

  // control signal
  float u2 = kp2*e2 + kd2*dedt2 + ki2*eintegral2;

  // motor power
  float pwr2 = fabs(u2);
  if( pwr2 > 125 ){
    pwr2 = 125;
  }

  // motor direction
  int dir2 = 1;
  if(u2<0){
    dir2 = -1;
  }

  // signal the motor
  setMotorH(dir2,pwr2,PWMm2,INM21,INM22);


  // store previous error
  eprev2 = e2;

  Serial.print(target2);
  Serial.print(" ");
  Serial.print(pos2);
  Serial.print(" ");

 
}*/


void forward (void)
{
  Motor_postion_control1(0, PWM1, INm1,posi3);
Motor_postion_control(180, PWM2, INm2,posi2);
Serial.print("    "); 
Motor_postion_control(0, PWM1, INm1,posi1);
 Serial.println(""); 
  //setMotorH(1,255,PWM2,INm21,INm22);
//Motor_postion_control1(Degree1, PWM1, INm1,posi3);
//setMotorH(1,255,PWM3,INm31,INm32);
}
void STOP(void)
{
  Motor_postion_control(0, PWM2, INm2,posi2);
Serial.print("    "); 
Motor_postion_control(360*2, PWM1, INm1,posi1);
}
void right()
{
  Motor_postion_control1(180, PWM1, INm1,posi3);
}
void left()
{
  Motor_postion_control1(-180, PWM1, INm1,posi3);
}

void bluetooth()
{
 while (Serial1.available())                                    // Read bluetooth commands over Serial1 // Warning: If an error with Serial1 occurs, make sure Arduino Mega 2560 is Selected
 {  
   
    blueToothVal = Serial1.read();
                                 //  convert the string 'str' into an integer and assign it to blueToothVal
    Serial.print("BlueTooth Value ");
    Serial.println(blueToothVal);    

// **************************************************************************************************************************************************

 switch (blueToothVal) 
 {
      case 'a':                                
         forward ();
        break;

      case 'b':                 
         STOP();
        break;

      case 'c':         
         right();
        break;
       case 'd':                     
        left();
        break;
        
      case 'e':                                            
        
        break; 

      case 6:                 
        
        break;
      
      case 7:        
        
        break;  
 }
 }}
 /*
void blue(void)
{
if (Serial.available() > 0) {
    command = Serial.read();
    Stop(); //Initialize with motors stoped.
    switch(command)
    {
      case 'a':
        forward();
        break;
      case 'c':
        back();
        break;
      case 'd':
        left();
        break;
      case 'b':
        right();
        break;
        case 's':
        Stop();
        break;
    }
    Serial.print(command);
  }
  }
*/
