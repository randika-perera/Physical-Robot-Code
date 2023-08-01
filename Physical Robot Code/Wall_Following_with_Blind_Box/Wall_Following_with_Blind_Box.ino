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

int IR_sensor0_pin = 0; 
int IR_sensor1_pin = 1; 
int IR_sensor2_pin = 2; 
int IR_sensor3_pin = 3; 
int IR_sensor4_pin = 4; 
int threshold = 500;

int d_reference = 5 ;  //try to reduce more

int basespeed=56;
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

int turn_delay=200;
int move_delay=100;


void setup() 
{
  pinMode(IR_sensor0_pin,INPUT);
  pinMode(IR_sensor1_pin,INPUT);
  pinMode(IR_sensor2_pin,INPUT);
  pinMode(IR_sensor3_pin,INPUT);
  pinMode(IR_sensor4_pin,INPUT);
}

void loop() 
{
  // put left wall following into setup
}


int digital_read(int pin)
{
  int IR_analog_value=analogRead(pin);
  if (IR_analog_value<threshold)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------

void leftwallfollowing_with_blindbox()
{
{
    //----------------------wall following code---------------------------------------------------------------------------------------------------------------
  HCSR04 hc_front(front_trig, front_echo);
  d_front=hc_front.dist();
  if ((d_front==0.00) || (d_front>60))
  {
    d_front=60;
  }

  if (d_front<=20)
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

  if (d_left_back<=20)
  {
    motor_control(1,70);
    motor_control(2,0);
    delay(15);
    motor_control(1,0);
    motor_control(2,0);
  }

  //wall following readings from this sensor
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

  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  int IR0=digital_read(IR_sensor0_pin);
  int IR1=digital_read(IR_sensor1_pin);
  int IR2=digital_read(IR_sensor2_pin);
  int IR3=digital_read(IR_sensor3_pin);
  int IR4=digital_read(IR_sensor4_pin);
  if ((IR0==1)||(IR1==1)||(IR2==1)||(IR3==1)||(IR4==1))
  {
    //line_following()  connect to line following code
  }
  //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  HCSR04 hc_left_front(left_front_trig, left_front_echo);
  d_left_front=hc_left_front.dist();
  if ((d_left_front==0.00) || (d_left_front>80))
  {
    d_left_front=80;
  }
  
  if (d_left_front>=20)
  {
    stop_robot();
    HCSR04 hc_front(front_trig, front_echo);
    d_front=hc_front.dist();
    if ((d_front==0.00) || (d_front>80))
    {
       d_front=80;
    }
    if (d_front==80)
    {
      move_backwards();
      turn_right();
      move_forwards()
    }
    else
    {
      turn_left();
      move_forwards();
      int IR0=digital_read(IR_sensor0_pin);
  int IR1=digital_read(IR_sensor1_pin);
  int IR2=digital_read(IR_sensor2_pin);
  int IR3=digital_read(IR_sensor3_pin);
  int IR4=digital_read(IR_sensor4_pin);
  if ((IR0==1)||(IR1==1)||(IR2==1)||(IR3==1)||(IR4==1))
  {// move to line following}
  }
  else
  {
    move_backwards();
    turn_right();
    move_forwards();
  }
    }
  }
  }
  
 
      
 



//-------------------------------------------------------------------------------------------------------------------------------------------------------------

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

void turn_left()
{
  motor_control(1,-100);
  motor_control(2,100);
  delay(turn_delay);
  stop_robot();
}

void turn_right()
{
  motor_control(1,115);   //CHECK WHY LEFT RIGHT SPEEDS DIFFERENT IN MAZE
  motor_control(2,-115);
  delay(turn_delay);
  stop_robot();
}

void move_forwards()
{
  motor_control(1,100);
  motor_control(2,100);
  delay(move_delay);
  stop_robot();
}

void move_backwards()
{
  motor_control(1,-100);
  motor_control(2,-100);
  delay(move_delay);
  stop_robot();
}


void stop_robot()
{
  motor_control(1,0);
  motor_control(2,0);
}
