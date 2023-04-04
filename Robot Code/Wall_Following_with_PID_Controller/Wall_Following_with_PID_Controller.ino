#include <HCSR04.h>

#define left_front_trig 26
#define left_front_echo 28
#define left_back_trig 30
#define left_back_echo 32
#define right_front_trig 34
#define right_front_echo 36
#define right_back_trig 38
#define right_back_echo 40
#define front_trig 22
#define front_echo 24

#define forward_pin_A  50 
#define backward_pin_A 51 
#define enA 6  
#define forward_pin_B 53 
#define backward_pin_B 52  
#define enB 7 

int d_reference = 4 ;

int  basespeed=56;
int previouserror=0;

int LMV;
int RMV;

float Kp=1.2;  
float Kd=0.08; 
float Ki=0.01;
float control_signal;

float d_left_front;
float d_left_back;
float d_right_front;
float d_right_back;
float d_front;

void setup() 
{
  //Do nothing
}

void loop() 
{
  leftwallfollowing();
}



void leftwallfollowing()
{
  HCSR04 hc_front(front_trig, front_echo);
  d_front=hc_front.dist();
  if ((d_front==0.00) || (d_front>60))
  {
    d_front=60;
  }

  if (d_front<=10)
  {
    motor_control(1,70);
    motor_control(2,-70);
    delay(15);
    motor_control(1,70);
    motor_control(2,70);
    delay(3);
    motor_control(1,0);
    motor_control(2,0);
  }

  HCSR04 hc_left_back(left_back_trig, left_back_echo);
  d_left_back=hc_left_back.dist();
  if ((d_left_back==0.00) || (d_left_back>60))
  {
    d_left_back=60;
  }

  if (d_left_back<=10)
  {
    motor_control(1,70);
    motor_control(2,0);
    delay(15);
    motor_control(1,0);
    motor_control(2,0);
  }


  HCSR04 hc_left_front(left_front_trig, left_front_echo);
  d_left_front=hc_left_front.dist();
  if ((d_left_front==0.00) || (d_left_front>80))
  {
    d_left_front=80;
  }


  float error=0.3*(d_reference-d_left_front);
  float delta=error-previouserror;

  previouserror=error;
  
  control_signal=(Kp*error+Kd*delta)/2;

  LMV=basespeed+control_signal;
  RMV=basespeed-control_signal;

  if ((d_left_front>=80) && (d_left_back>=80))
  {
    LMV=basespeed;
    RMV=basespeed;
  }
  
  motor_control(1,LMV);
  motor_control(2,RMV);
  
}


void motor_control(int M_name , int value)
{
  pinMode(forward_pin_A,OUTPUT);pinMode(backward_pin_A,OUTPUT);pinMode(enA,OUTPUT);
  pinMode(forward_pin_B,OUTPUT);pinMode(backward_pin_B,OUTPUT);pinMode(enB,OUTPUT);
  if (M_name == 1)
  {
    if (value >=0){digitalWrite(backward_pin_A , LOW);digitalWrite(forward_pin_A , HIGH); analogWrite(enA , value);}
    else{digitalWrite(backward_pin_A , HIGH);digitalWrite(forward_pin_A , LOW);analogWrite(enA , -value);}
  }
  else if (M_name == 2)
  {
    if (value >=0){digitalWrite(backward_pin_B , LOW);digitalWrite(forward_pin_B , HIGH);analogWrite(enB , value);}
    else{digitalWrite(backward_pin_B , HIGH);digitalWrite(forward_pin_B , LOW);analogWrite(enB , -value);}
  }
}
